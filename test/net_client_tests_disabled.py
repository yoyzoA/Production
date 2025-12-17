# test/test_net_client.py

import os
import sys

import pytest

# Make sure project root is on sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import navigation.net_client as net_client
from navigation.net_client import PiConnection


# PiConnection high-level protocol helpers (no real network)

def test_send_state_uppercases_and_calls_send_line():
    conn = PiConnection(host="pi.local", port=5001, timeout=3.0)

    captured = []

    def fake_send_line(text: str):
        captured.append(text)

    # Replace the network part with our fake collector
    conn.send_line = fake_send_line

    conn.send_state("training")
    conn.send_state("NaViGaTe")

    assert captured == ["STATE TRAINING", "STATE NAVIGATE"]


def test_send_train_accepts_valid_keys_and_rejects_invalid():
    conn = PiConnection(host="pi.local", port=5001, timeout=3.0)

    captured = []
    conn.send_line = lambda text: captured.append(text)

    # Valid keys (case-insensitive, stripped)
    conn.send_train("w")
    conn.send_train("A ")
    conn.send_train("F")

    assert captured == ["TRAIN w", "TRAIN a", "TRAIN f"]

    # Invalid key should raise
    with pytest.raises(ValueError):
        conn.send_train("x")


def test_send_seg_formats_duration_and_validates():
    conn = PiConnection(host="pi.local", port=5001, timeout=3.0)

    captured = []
    conn.send_line = lambda text: captured.append(text)

    # Valid: duration should be rounded to 3 decimals
    conn.send_seg("w", 1.23456)
    assert captured == ["SEG w 1.235"]

    # Negative duration should be rejected
    with pytest.raises(ValueError):
        conn.send_seg("w", -0.1)


def test_send_event_trims_and_rejects_empty():
    conn = PiConnection(host="pi.local", port=5001, timeout=3.0)

    captured = []
    conn.send_line = lambda text: captured.append(text)

    conn.send_event("  goal_reached  ")
    assert captured == ["EVENT goal_reached"]

    with pytest.raises(ValueError):
        conn.send_event("   ")


# Functional wrappers around a connection object

def test_functional_wrappers_call_methods():
    class DummyConn:
        def __init__(self):
            self.called = {
                "state": None,
                "train": None,
                "seg": None,
                "event": None,
                "closed": False,
            }

        def send_state(self, name: str):
            self.called["state"] = name

        def send_train(self, key: str):
            self.called["train"] = key

        def send_seg(self, key: str, duration: float):
            self.called["seg"] = (key, duration)

        def send_event(self, name: str):
            self.called["event"] = name

        def close(self):
            self.called["closed"] = True

    conn = DummyConn()

    net_client.send_state(conn, "IDLE")
    net_client.send_train(conn, "w")
    net_client.send_seg(conn, "a", 1.0)
    net_client.send_event(conn, "goal_reached")
    net_client.close_connection(conn)

    assert conn.called["state"] == "IDLE"
    assert conn.called["train"] == "w"
    assert conn.called["seg"] == ("a", 1.0)
    assert conn.called["event"] == "goal_reached"
    assert conn.called["closed"] is True


# ----------------------------------------------------------------------
# connect_to_pi: test host/port/timeout wiring with a dummy class
# ----------------------------------------------------------------------

def test_connect_to_pi_uses_pi_host_port_and_calls_connect(monkeypatch):
    # Dummy replacement for PiConnection so we don't hit real network or HTTP.
    class DummyPiConn:
        def __init__(self, host: str, port: int, timeout: float):
            self.host = host
            self.port = port
            self.timeout = timeout
            self.connect_called = False

        def connect(self):
            self.connect_called = True

    # Override host/port defaults and PiConnection class
    monkeypatch.setattr(net_client, "PI_HOST", "fakehost")
    monkeypatch.setattr(net_client, "PI_PORT", 6000)
    monkeypatch.setattr(net_client, "PiConnection", DummyPiConn)

    conn = net_client.connect_to_pi()

    assert isinstance(conn, DummyPiConn)
    assert conn.host == "fakehost"
    assert conn.port == 6000
    assert hasattr(conn, "timeout")
    assert conn.connect_called is True
