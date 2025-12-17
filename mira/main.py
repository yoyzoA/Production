# mira/main.py
#
# Raspberry Pi main process:
# - Listens for HTTP commands from the laptop.
# - Updates FSM events and shared commands.
# - Calls state behaviors (IDLE / TRAINING / NAVIGATE / DOCKING).
# - Runs RoverPID.update() in a loop.
#
# Run this on the Pi from inside the 'mira' folder:
# cd /path/to/mira
# python3 main.py

from __future__ import annotations

import json
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Deque, Tuple
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# FSM: use the definitions from mira/fsm/fsm.py
from fsm.fsm import State, next_state

# State behaviors
from states.idle import idle_step, stop_all_motion
from states.training import start_training, training_step, is_training_done
from states.navigate import start_navigate, navigate_step

# PID controller for the rover
from pid import RoverPID


# ---------------------------------------------------------------------------
# Events: inputs to the FSM (must match fields used in fsm/fsm.py)
# ---------------------------------------------------------------------------

@dataclass
class Events:
    # From laptop / system:
    has_map: bool = False            # True when map is available (set by you)
    app_cmd_train: bool = False      # STATE TRAINING
    app_cmd_nav: bool = False        # STATE NAVIGATE
    app_cmd_dock: bool = False       # STATE DOCKING
    battery_low: bool = False        # could be set from sensor / event

    # From training/navigation:
    training_done: bool = False      # set when TRAINING is finished
    goal_reached: bool = False       # from EVENT goal_reached (laptop)
    dock_reached: bool = False       # from EVENT dock_reached (laptop or sensor)

    # Extra local command (not used by FSM directly but convenient):
    app_cmd_idle: bool = False       # STATE IDLE

    def clear_transient(self) -> None:
        """
        Clear one-shot events that should not persist forever.
        Persistent flags like 'has_map' and 'battery_low'
        are NOT cleared here.
        """
        self.app_cmd_train = False
        self.app_cmd_nav = False
        self.app_cmd_dock = False
        self.app_cmd_idle = False
        self.goal_reached = False
        self.dock_reached = False
        # has_map and battery_low remain as long-term flags


# ---------------------------------------------------------------------------
# Shared control between network thread and main loop
# ---------------------------------------------------------------------------

@dataclass
class SharedControl:
    """
    Shared state between network thread and main control loop.
    """
    events: Events

    # TRAINING: last key received from laptop ('w','a','s','d','f') or None
    training_key: Optional[str] = None

    # NAVIGATE/DOCKING: current NAV command ('w','a','s','d' or None)
    nav_key: Optional[str] = None
    nav_time_remaining: float = 0.0    # seconds remaining for current nav command

    # Queue of upcoming NAV/DOCKING segments (key, duration_s)
    nav_queue: Deque[Tuple[str, float]] = field(default_factory=deque)

    # Optional: for TRAIN timeout (not fully used yet)
    last_train_time: float = 0.0


# ---------------------------------------------------------------------------
# HTTP server handling (Pi side)
# ---------------------------------------------------------------------------

def process_command(line: str, shared: SharedControl) -> None:
    """
    Parse a single command line and update events / shared commands.
    Expected commands (from laptop):
        STATE <NAME>
        TRAIN <KEY>
        SEG <KEY> <DURATION_S>
        EVENT <NAME>
    """
    parts = line.split()
    if not parts:
        return

    cmd = parts[0].upper()
    ev = shared.events

    # -------------------- STATE commands --------------------
    # STATE TRAINING / NAVIGATE / IDLE / DOCKING
    if cmd == "STATE" and len(parts) >= 2:
        state_name = parts[1].upper()
        if state_name == "TRAINING":
            ev.app_cmd_train = True
        elif state_name == "NAVIGATE":
            ev.app_cmd_nav = True
        elif state_name == "IDLE":
            ev.app_cmd_idle = True
        elif state_name == "DOCKING":
            ev.app_cmd_dock = True
        return

    # -------------------- TRAIN commands --------------------
    # TRAIN w/a/s/d/f
    if cmd == "TRAIN" and len(parts) >= 2:
        k = parts[1].lower()
        if k in ("w", "a", "s", "d", "f"):
            shared.training_key = k
            shared.last_train_time = time.time()
        return

    # -------------------- SEG commands ----------------------
    # SEG w/a/s/d duration
    if cmd == "SEG" and len(parts) >= 3:
        k = parts[1].lower()
        try:
            T = float(parts[2])
        except ValueError:
            print(f"[Pi] Invalid SEG duration: {parts[2]}")
            return

        if k in ("w", "a", "s", "d") and T > 0.0:
            shared.nav_queue.append((k, T))
            print(f"[Pi] Queued SEG {k} for {T:.3f}s")
        return

    # -------------------- EVENT commands --------------------
    # EVENT goal_reached / dock_reached / training_done / has_map_ready ...
    if cmd == "EVENT" and len(parts) >= 2:
        name = parts[1]
        if name == "goal_reached":
            ev.goal_reached = True
        elif name == "dock_reached":
            ev.dock_reached = True
        elif name == "training_done":
            # If you ever send this from laptop, mark training done
            ev.training_done = True
        elif name == "has_map_ready":
            # Optional: laptop can tell Pi the map is now available
            ev.has_map = True
        return

    # Unknown command -> ignore (or log)
    print(f"[Pi] Unknown command: {line}")


def start_server(shared: SharedControl, host: str = "0.0.0.0", port: int = 5000) -> None:
    """
    Start a lightweight HTTP server that accepts commands from the laptop.
    POST /command with a plain-text body containing one or more newline-
    separated commands will be parsed and forwarded to process_command().
    GET /health returns 200 OK for connectivity checks.
    This function returns immediately after starting a daemon thread.
    """
    class CommandHandler(BaseHTTPRequestHandler):
        # Capture shared control via closure
        def _send_json(self, status_code: int, payload: dict) -> None:
            data = json.dumps(payload).encode("utf-8")
            self.send_response(status_code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, format: str, *args) -> None:    # type: ignore[override]
            # Silence default HTTP logging; we print our own messages when commands arrive.
            return

        def do_GET(self) -> None:    # type: ignore[override]
            if self.path == "/health":
                self._send_json(200, {"status": "ok"})
            else:
                self._send_json(404, {"error": "not found"})

        def do_POST(self) -> None:    # type: ignore[override]
            if self.path != "/command":
                self._send_json(404, {"error": "not found"})
                return

            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                length = 0

            raw_body = self.rfile.read(length) if length > 0 else b""
            body_text = raw_body.decode("utf-8", errors="ignore")
            lines = [ln.strip() for ln in body_text.splitlines() if ln.strip()]

            for line in lines:
                print(f"[Pi] Received: {line}")
                process_command(line, shared)

            self._send_json(200, {"received": len(lines)})

    def server_thread():
        server = ThreadingHTTPServer((host, port), CommandHandler)
        print(f"[Pi] HTTP server listening on http://{host}:{port}")
        server.serve_forever()

    threading.Thread(target=server_thread, daemon=True).start()


# ---------------------------------------------------------------------------
# Main control loop
# ---------------------------------------------------------------------------

def main_loop() -> None:
    """
    Main control loop on the Pi:
      - runs FSM using Events
      - calls state behaviors (idle/training/navigate/docking)
      - runs RoverPID.update()
    """
    rover = RoverPID()
    events = Events()
    shared = SharedControl(events=events)

    # You may set events.has_map = True here if you want to pretend
    # that the map already exists (for testing NAVIGATE/DOCKING).
    # events.has_map = True

    # Start HTTP server for laptop connection
    start_server(shared, host="0.0.0.0", port=5000)

    state = State.IDLE
    print("[Pi] Starting in IDLE state.")

    last_time = time.time()

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            # ----------------- Update NAV segment timer -----------------
            if shared.nav_time_remaining > 0.0:
                shared.nav_time_remaining -= dt
                if shared.nav_time_remaining <= 0.0:
                    # Segment duration over -> stop NAV command
                    shared.nav_key = None
                    shared.nav_time_remaining = 0.0

            # If no active NAV segment, start the next queued one
            if state in (State.NAVIGATE, State.DOCKING) and shared.nav_key is None and shared.nav_queue:
                next_key, next_duration = shared.nav_queue.popleft()
                shared.nav_key = next_key
                shared.nav_time_remaining = next_duration
                print(f"[Pi] Starting SEG {next_key} for {next_duration:.3f}s")

            # ----------------- Compute next FSM state -------------------
            new_state = next_state(state, events)

            # ----------------- Handle state transitions -----------------
            if new_state != state:
                print(f"[Pi] State change: {state.name} -> {new_state.name}")

                # EXIT actions for old state
                if state == State.TRAINING:
                    # Stop rover when leaving training
                    rover.set_target(0.0, 0.0)

                # ENTER actions for new state
                if new_state == State.IDLE:
                    stop_all_motion(rover)
                    rover.set_target(0.0, 0.0)
                    shared.nav_key = None
                    shared.nav_time_remaining = 0.0
                    shared.nav_queue.clear()

                elif new_state == State.TRAINING:
                    start_training(rover)
                    shared.training_key = None
                    shared.nav_key = None
                    shared.nav_time_remaining = 0.0
                    shared.nav_queue.clear()
                    events.training_done = False

                elif new_state == State.NAVIGATE:
                    start_navigate(rover)
                    shared.nav_key = None
                    shared.nav_time_remaining = 0.0
                    shared.nav_queue.clear()

                elif new_state == State.DOCKING:
                    print("[Pi] Entered DOCKING state (reusing NAV commands).")
                    shared.nav_key = None
                    shared.nav_time_remaining = 0.0
                    shared.nav_queue.clear()
                    # You may call a dedicated start_docking(rover) here later.

                state = new_state

            # ----------------- Per-state behavior -----------------------
            if state == State.IDLE:
                idle_step(rover)

            elif state == State.TRAINING:
                # Consume one training key (if any)
                key = shared.training_key
                shared.training_key = None
                training_step(rover, key)

                # If training has finished (e.g.
                if is_training_done():
                    events.training_done = True

            elif state == State.NAVIGATE:
                cmd = shared.nav_key
                navigate_step(rover, cmd)

            elif state == State.DOCKING:
                # For now, DOCKING uses the same navigate_step with SEG commands
                cmd = shared.nav_key
                navigate_step(rover, cmd)

            # ----------------- Update PID control -----------------------
            rover.update()

            # ----------------- Clear transient events -------------------
            events.clear_transient()

            # Control loop ~50 Hz
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[Pi] KeyboardInterrupt, stopping main loop.")
    finally:
        print("[Pi] Stopping rover and cleaning up.")
        rover.stop()
        rover.cleanup()
        print("[Pi] Done.")


if __name__ == "__main__":
    main_loop()
