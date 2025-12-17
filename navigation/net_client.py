# navigation/net_client.py
#
# Simple HTTP client for talking to the Raspberry Pi.
# Protocol: POST /command with line-based UTF-8 text commands:
#
# STATE <NAME>
# TRAIN <KEY>
# SEG <KEY> <DURATION_S>
# EVENT <NAME>
#
# where:
# - <NAME> is e.g.
# - <KEY> is one of: w, a, s, d, f
# - <DURATION_S> is a float in seconds (e.g.
# Clients send each command as a separate HTTP request, which keeps the
# Pi listener simple and firewall-friendly.

from __future__ import annotations

import http.client
from dataclasses import dataclass

try:
    # We expect config.py to define PI_HOST and PI_PORT
    from .config import PI_HOST, PI_PORT
except ImportError:
    # Fallback defaults if config.py is not present yet.
    # PI_HOST = "192.168.0.100"
    # PI_PORT = 5000
    PI_HOST = "127.0.0.1"
    PI_PORT = 5000

@dataclass
class PiConnection:
    """
    Minimal HTTP client wrapper for sending commands to the Pi.

    Usage:
        conn = PiConnection(host, port)
        conn.connect()    # performs /health check
        conn.send_state("TRAINING")
        ...
        conn.close()      # no-op for symmetry

    Can also be used as a context manager:

        with PiConnection(host, port) as conn:
            conn.send_state("NAVIGATE")
            ...

    """
    host: str = PI_HOST
    port: int = PI_PORT
    timeout: float = 5.0    # seconds

    _connected: bool = False

    # ------------- Connection management -------------

    def connect(self) -> None:
        """Ensure the Pi HTTP endpoint is reachable."""
        if self._connected:
            return    # already connected
        self._check_health()
        self._connected = True

    def _check_health(self) -> None:
        conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
        try:
            conn.request("GET", "/health")
            resp = conn.getresponse()
            resp.read()    # drain response
            if resp.status >= 400:
                raise RuntimeError(f"Pi health check failed with status {resp.status}")
        except OSError as e:
            raise RuntimeError(
                f"Could not reach Pi at http://{self.host}:{self.port}: {e}"
            ) from e
        finally:
            conn.close()

    def close(self) -> None:
        """No persistent connection to close; reset connection flag."""
        self._connected = False

    # Context manager support: "with PiConnection(...) as conn: ..."
    def __enter__(self) -> PiConnection:
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ------------- Low-level send -------------

    def _ensure_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("PiConnection is not connected. Call connect() first.")

    def _post_command(self, text: str) -> None:
        """
        POST a single command line to the Pi's /command endpoint.
        """
        payload = (text.rstrip("\r\n") + "\n").encode("utf-8")
        conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
        try:
            conn.request(
                "POST",
                "/command",
                body=payload,
                headers={"Content-Type": "text/plain; charset=utf-8"},
            )
            resp = conn.getresponse()
            resp.read()    # drain response body
            if resp.status >= 300:
                raise RuntimeError(
                    f"Pi returned HTTP {resp.status} for command {text!r}"
                )
        except OSError as e:
            raise RuntimeError(f"Failed to send command to Pi: {e}") from e
        finally:
            conn.close()

    def send_line(self, text: str) -> None:
        """
        Send one line (terminated by '\n') to the Pi over HTTP.
        """
        self._ensure_connected()
        self._post_command(text)

    # ------------- High-level protocol helpers -------------

    def send_state(self, state_name: str) -> None:
        """
        Send a STATE command, e.g. STATE TRAINING, STATE NAVIGATE, etc.
        """
        state_name = state_name.strip().upper()
        self.send_line(f"STATE {state_name}")

    def send_train(self, key: str) -> None:
        """
        Send a TRAIN command with a WASD/F key, e.g. TRAIN w, TRAIN a, TRAIN f.
        """
        k = key.strip().lower()
        if k not in ("w", "a", "s", "d", "f"):
            raise ValueError(f"Invalid TRAIN key: {key}")
        self.send_line(f"TRAIN {k}")

    def send_seg(self, key: str, duration_s: float) -> None:
        """
        Send a SEG command for NAVIGATE segments:
            SEG w 1.2
            SEG a 0.5
        """
        k = key.strip().lower()
        if k not in ("w", "a", "s", "d"):
            raise ValueError(f"Invalid SEG key: {key}")

        if duration_s < 0.0:
            raise ValueError("duration_s must be non-negative")

        # Format with 3 decimal places for readability
        self.send_line(f"SEG {k} {duration_s:.3f}")

    def send_event(self, event_name: str) -> None:
        """
        Send an EVENT command, e.g. EVENT goal_reached.
        """
        name = event_name.strip()
        if not name:
            raise ValueError("event_name cannot be empty")
        self.send_line(f"EVENT {name}")


# ----------------- Functional helpers (used by other modules) ----------------- #

def connect_to_pi(
    host: str = PI_HOST,
    port: int = PI_PORT,
    timeout: float = 5.0,
) -> PiConnection:
    """
    Convenience function to create and connect a PiConnection in one call.

    Example:
        conn = connect_to_pi()
        send_state(conn, "TRAINING")
        ...
        close_connection(conn)
    """
    conn = PiConnection(host=host, port=port, timeout=timeout)
    conn.connect()
    return conn


def close_connection(conn: PiConnection) -> None:
    """
    Close a PiConnection (wrapper to match the older functional API).
    """
    conn.close()


def send_state(conn: PiConnection, state_name: str) -> None:
    """
    Functional wrapper for conn.send_state(...).
    """
    conn.send_state(state_name)


def send_train(conn: PiConnection, key: str) -> None:
    """
    Functional wrapper for conn.send_train(...).
    """
    conn.send_train(key)


def send_seg(conn: PiConnection, key: str, duration_s: float) -> None:
    """
    Functional wrapper for conn.send_seg(...).
    """
    conn.send_seg(key, duration_s)


def send_event(conn: PiConnection, event_name: str) -> None:
    """
    Functional wrapper for conn.send_event(...).
    """
    conn.send_event(event_name)


# Optional manual test
if __name__ == "__main__":
    # This will try to connect to PI_HOST
    print(f"Connecting to Pi at {PI_HOST}:{PI_PORT} ...")
    try:
        conn = connect_to_pi()
        send_state(conn, "IDLE")
        send_event(conn, "test_ping")
        close_connection(conn)
        print("Sent test commands OK.")
    except Exception as e:
        print("Failed to send test commands:", e)
