# navigation/training_session.py
#
# Laptop-side helper to run a TRAINING session:
# - Connect to the Raspberry Pi over HTTP
# - Put the FSM into TRAINING state
# - Read WASD/F commands from the terminal
# - Send TRAIN w/a/s/d/f lines to the Pi
# - Log ZED poses and room-label events, then build and save a navigation map
#
# NOTE:
# This script now opens the ZED directly during training to log poses
# and room labels for building a navigation map.


from __future__ import annotations

import csv
import os
import threading
import time
from dataclasses import dataclass
from math import hypot
from typing import Dict, List, Optional

from . import config
from .config import PI_HOST, PI_PORT
from .map_loader import Edge, MapData, Node, save_map
from .net_client import (
    PiConnection,
    connect_to_pi,
    close_connection,
    send_state,
    send_train,
)
from .zed_interface import ZedInterface


@dataclass
class PoseSample:
    t: float     # timestamp in seconds
    x: float     # ground-plane X
    z: float     # ground-plane Z (NOT Y!)
    yaw: float   # heading (rad, from ZED)


@dataclass
class RoomEvent:
    t: float
    room_name: str


def build_map_from_data(
    poses: List[PoseSample],
    room_events: List[RoomEvent],
    dist_threshold: float = 0.5,
    max_room_node_dist: float = 1.0,
) -> MapData:
    """
    Downsample poses into nodes/edges and attach room labels.
    """
    if not poses:
        print("Warning: no poses were recorded; returning empty map.")
        return MapData(nodes=[], edges=[], rooms={})

    if not room_events:
        print("Warning: no rooms were labelled during training; rooms dict will be empty.")

    nodes: List[Node] = []
    edges: List[Edge] = []

    nodes.append(Node(id=0, x=poses[0].x, z=poses[0].z))
    last_node_index = 0
    last_node_pose = poses[0]

    for p in poses[1:]:
        d = hypot(p.x - last_node_pose.x, p.z - last_node_pose.z)
        if d >= dist_threshold:
            new_id = len(nodes)
            nodes.append(Node(id=new_id, x=p.x, z=p.z))
            edges.append(Edge(src=last_node_index, dst=new_id, dist=d))
            edges.append(Edge(src=new_id, dst=last_node_index, dist=d))
            last_node_index = new_id
            last_node_pose = p

    rooms: Dict[str, int] = {}
    for event in room_events:
        pose_for_room = min(poses, key=lambda p: abs(p.t - event.t))

        nearest_id: Optional[int] = None
        nearest_dist = float("inf")
        for node in nodes:
            dn = hypot(node.x - pose_for_room.x, node.z - pose_for_room.z)
            if dn < nearest_dist:
                nearest_dist = dn
                nearest_id = node.id

        if nearest_id is None:
            print(f"Warning: could not find nearest node for room '{event.room_name}'")
            continue

        if nearest_dist > max_room_node_dist:
            print(
                f"Warning: room '{event.room_name}' is {nearest_dist:.2f} m away from nearest node"
            )

        if event.room_name in rooms:
            print(
                f"Room '{event.room_name}' already assigned; overriding with latest event at t={event.t:.2f}s."
            )

        rooms[event.room_name] = nearest_id

    return MapData(nodes=nodes, edges=edges, rooms=rooms)

def run_training_cli() -> None:
    """
    Command-line training client with ZED pose logging and room labelling.

    Workflow:
      1) Connects to the Pi (PI_HOST, PI_PORT).
      2) Opens ZED, enables tracking, starts background pose logging.
      3) Sends STATE TRAINING.
      4) Repeatedly asks the user for a command:
             w/a/s/d/f -> TRAIN command to Pi
             name <RoomName> or NAME_ROOM <RoomName> -> log room label event
             q -> quit client (no auto TRAIN f)
      5) Builds and saves navigation map + pose/room logs when training ends.
      6) Closes the HTTP client and ZED on exit.
    """
    print(f"Connecting to Pi at {PI_HOST}:{PI_PORT} ...")
    sock: Optional[PiConnection] = None

    poses: List[PoseSample] = []
    room_events: List[RoomEvent] = []

    zed = ZedInterface()
    pose_thread_stop = threading.Event()
    pose_thread: Optional[threading.Thread] = None

    def pose_logger() -> None:
        last_error_time = 0.0
        while not pose_thread_stop.is_set():
            try:
                x, _, z, heading = zed.get_pose_with_heading()
                poses.append(PoseSample(t=time.time(), x=x, z=z, yaw=heading))
            except Exception as e:
                now = time.time()
                if now - last_error_time > 5.0:
                    print(f"[ZED] Warning: failed to grab pose: {e}")
                    last_error_time = now
            time.sleep(0.1)    # ~10 Hz

    try:
        # ZED setup
        print("[TRAIN] Opening ZED camera ...")
        zed.open()
        print("[TRAIN] Enabling ZED tracking without prior area file.")
        zed.enable_tracking(load_area=False, area_path=None)
        time.sleep(1.0)    # let tracking stabilize

        pose_thread = threading.Thread(target=pose_logger, daemon=True)
        pose_thread.start()

        sock = connect_to_pi(PI_HOST, PI_PORT)
        print("Connected to Pi.")

        # Put FSM into TRAINING state
        send_state(sock, "TRAINING")
        print("Sent: STATE TRAINING")

        print("\n--- TRAINING MODE ---")
        print("Use keys:")
        print("  w = forward")
        print("  s = backward")
        print("  a = turn left")
        print("  d = turn right")
        print("  f = finish training (stop)")
        print("  name <RoomName> = label current position with room name")
        print("  q = quit this client")
        print("-----------------------\n")

        while True:
            raw = input("Command [w/a/s/d/f/name/q]: ").strip()
            if not raw:
                continue

            lower = raw.lower()

            if lower == "q":
                print("Quitting training client (no TRAIN f sent automatically).")
                break

            parts = raw.split(maxsplit=1)
            if parts and parts[0].upper() == "NAME_ROOM":
                if len(parts) < 2 or not parts[1].strip():
                    print("NAME_ROOM requires a room name, e.g. NAME_ROOM Kitchen")
                    continue
                room_name = parts[1].strip()
                t_event = time.time()
                room_events.append(RoomEvent(t=t_event, room_name=room_name))
                print(f"Logged room '{room_name}' at t={t_event:.2f}s.")
                continue

            if lower.startswith("name "):
                room_name = raw.split(None, 1)[1].strip()
                if not room_name:
                    print("Name command requires a room name, e.g. name Kitchen")
                    continue
                t_event = time.time()
                room_events.append(RoomEvent(t=t_event, room_name=room_name))
                print(f"Logged room '{room_name}' at t={t_event:.2f}s.")
                continue

            key = lower[0]

            if key in ("w", "a", "s", "d", "f"):
                # Send TRAIN command to Pi
                send_train(sock, key)
                print(f"Sent: TRAIN {key}")

                if key == "f":
                    print("Finish command sent. TRAINING should end on the Pi.")
                    break
            else:
                print(f"Invalid command: {raw!r}. Use w/a/s/d/f/name/q.")

    except Exception as e:
        print(f"[ERROR] Training session failed: {e}")

    finally:
        pose_thread_stop.set()
        if pose_thread is not None:
            pose_thread.join(timeout=2.0)

        # Save pose CSV if we have data
        if poses:
            os.makedirs(os.path.dirname(config.TRAJECTORY_CSV_PATH), exist_ok=True)
            print(f"[TRAIN] Writing poses to {config.TRAJECTORY_CSV_PATH}")
            with open(config.TRAJECTORY_CSV_PATH, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["t", "x", "z", "yaw"])
                for p in poses:
                    writer.writerow([p.t, p.x, p.z, p.yaw])
        else:
            print("[TRAIN] No poses recorded; skipping trajectory CSV.")

        # Save room events CSV optionally
        room_events_path = os.path.join(
            os.path.dirname(config.MAP_FILE_PATH), "room_events.csv"
        )
        if room_events:
            os.makedirs(os.path.dirname(room_events_path), exist_ok=True)
            print(f"[TRAIN] Writing room events to {room_events_path}")
            with open(room_events_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["t", "room_name"])
                for ev in room_events:
                    writer.writerow([ev.t, ev.room_name])
        else:
            print("[TRAIN] No room events logged.")

        # Build and save map
        map_data = build_map_from_data(poses, room_events)
        if map_data.nodes:
            os.makedirs(os.path.dirname(config.MAP_FILE_PATH), exist_ok=True)
            save_map(config.MAP_FILE_PATH, map_data)
            print(
                f"Saved navigation map with {len(map_data.nodes)} nodes, "
                f"{len(map_data.edges)} edges and {len(map_data.rooms)} rooms to "
                f"{config.MAP_FILE_PATH}"
            )
        else:
            print("No nodes built; map not saved.")

        if sock is not None:
            print("Closing connection to Pi.")
            close_connection(sock)

        try:
            zed.close()
        except Exception as e:
            print(f"[ZED] Warning while closing camera: {e}")

        print("Training client stopped.")


if __name__ == "__main__":
    run_training_cli()
