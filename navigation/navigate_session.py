# navigation/navigate_session.py
#
# Laptop-side NAVIGATE session:
# - Connects to the Pi over HTTP.
# - Loads saved map (nodes, edges, rooms).
# - Opens ZED, loads .area, enables tracking (relocalization).
# - Uses ZED pose + map to plan a path with Dijkstra.
# - Converts path to WASD+time segments.
# - Sends STATE NAVIGATE then SEG commands to the Pi.
# - Monitors pose until it reaches the goal, then sends EVENT goal_reached.

from __future__ import annotations

import time
import math
from typing import Optional

from .config import (
    PI_HOST,
    PI_PORT,
    MAP_FILE_PATH,
    AREA_FILE_PATH,
    V_NAV,
    OMEGA_NAV,
    GOAL_RADIUS,
)
from .net_client import (
    connect_to_pi,
    close_connection,
    send_state,
    send_seg,
    send_event,
)
from .map_loader import (
    load_map,
    get_room_node,
    get_node_position,
)
from .planner import (
    find_closest_node,
    plan_path,
)
from .segments import (
    path_to_segments,
    Segment,
)
from .zed_interface import ZedInterface


def _distance_2d(x1: float, z1: float, x2: float, z2: float) -> float:
    return math.hypot(x2 - x1, z2 - z1)


def navigate_to_room(room_name: str) -> None:
    """
    Execute a NAVIGATE mission to a given room.

    Args:
        room_name: Name of the target room as stored in map rooms
                   (e.g. "Kitchen", "Base", etc.)

    High-level flow:
        1) Connect to Pi.
        2) Load map (nodes, edges, rooms).
        3) Open ZED, load area file, enable tracking.
        4) Get current pose, find closest start node.
        5) Get goal node from room_name.
        6) Plan shortest path with Dijkstra.
        7) Convert path to (WASD, T) segments.
        8) Send STATE NAVIGATE and SEG commands to Pi.
        9) Poll ZED pose until within GOAL_RADIUS of goal.
       10) Send EVENT goal_reached to Pi.
    """
    # 1) Connect to Pi
    print(f"[NAV] Connecting to Pi at {PI_HOST}:{PI_PORT} ...")
    sock = connect_to_pi(PI_HOST, PI_PORT)
    print("[NAV] Connected to Pi.")

    # 2) Load map
    print(f"[NAV] Loading map from {MAP_FILE_PATH} ...")
    m = load_map(MAP_FILE_PATH)
    nodes = m.nodes
    edges = m.edges
    # Tell the Pi that a map is available so NAVIGATE/Dock states are allowed
    send_event(sock, "has_map_ready")

    # 3) Open ZED, load area, enable tracking
    zed = ZedInterface()
    print("[NAV] Opening ZED camera ...")
    zed.open()

    print(f"[NAV] Enabling tracking with area file: {AREA_FILE_PATH}")
    zed.enable_tracking(load_area=True, area_path=AREA_FILE_PATH)

    # Let tracking stabilize a bit
    time.sleep(1.0)

    # 4) Get current pose -> find closest start node
    x_curr, y_curr, z_curr, heading = zed.get_pose_with_heading()
    print(f"[NAV] Current pose: x={x_curr:.3f}, z={z_curr:.3f}, heading={heading:.3f} rad")

    start_id = find_closest_node(nodes, x_curr, z_curr)
    if start_id is None:
        raise RuntimeError("No nodes in map to start from.")

    print(f"[NAV] Closest start node id: {start_id}")

    # 5) Get goal node from room_name
    goal_id = get_room_node(m, room_name)
    if goal_id is None:
        raise RuntimeError(f"Room '{room_name}' not found in map rooms.")

    goal_pos = get_node_position(m, goal_id)
    if goal_pos is None:
        raise RuntimeError(f"Goal node id {goal_id} not found in nodes list.")

    x_goal, z_goal = goal_pos
    print(f"[NAV] Goal '{room_name}' -> node {goal_id} at x={x_goal:.3f}, z={z_goal:.3f}")

    # 6) Plan path with Dijkstra
    print("[NAV] Planning path with Dijkstra ...")
    node_path = plan_path(nodes, edges, start_id, goal_id)
    print(f"[NAV] Planned node path: {node_path}")

    # 7) Convert node path to segments (WASD, T)
    print("[NAV] Converting path to WASD segments ...")
    segments = path_to_segments(
        nodes=nodes,
        path=node_path,
        v_forward=V_NAV,
        omega_turn=OMEGA_NAV,
        initial_heading=heading,    # we pass current heading from ZED
    )

    for seg in segments:
        print(f"[NAV] Segment: {seg.direction} for {seg.duration:.3f} s")

    if not segments:
        print("[NAV] No movement needed (start is at goal).")
        # Still send goal_reached to make Pi/app consistent
        send_event(sock, "goal_reached")
        # Clean up and return
        zed.close()
        close_connection(sock)
        return

    # 8) Send STATE NAVIGATE to Pi
    print("[NAV] Sending STATE NAVIGATE to Pi.")
    send_state(sock, "NAVIGATE")

    # Send all segments to Pi, one by one
    print("[NAV] Sending SEG commands to Pi.")
    for seg in segments:
        send_seg(sock, seg.direction, seg.duration)
        # We don't necessarily need to sleep here; the Pi runs its own timer
        # using nav_time_remaining, but you MAY add a small delay to avoid
        # flooding the HTTP endpoint if desired.
        time.sleep(0.05)

    # 9) Poll ZED pose until within GOAL_RADIUS of goal
    print(f"[NAV] Monitoring position until within {GOAL_RADIUS:.2f} m of goal ...")
    try:
        while True:
            x_curr, y_curr, z_curr, heading = zed.get_pose_with_heading()
            dist = _distance_2d(x_curr, z_curr, x_goal, z_goal)
            print(f"[NAV] Distance to goal: {dist:.3f} m", end="\r", flush=True)

            if dist <= GOAL_RADIUS:
                print(f"\n[NAV] Goal '{room_name}' reached (dist={dist:.3f} m).")
                break

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n[NAV] KeyboardInterrupt while monitoring goal distance.")

    # 10) Notify Pi that goal is reached
    print("[NAV] Sending EVENT goal_reached to Pi.")
    send_event(sock, "goal_reached")

    # Cleanup
    print("[NAV] Closing ZED camera.")
    zed.close()

    print("[NAV] Closing HTTP client to Pi.")
    close_connection(sock)

    print("[NAV] Navigate session complete.")


if __name__ == "__main__":
    # Example: navigate to "Kitchen"
    navigate_to_room("Kitchen")
