# test/test_full_navigation_commands.py
#
# High-level integration test:
# Map (nodes/edges) -> plan_path -> path_to_segments -> SEG commands
#
# This simulates going from Base to Kitchen in a tiny apartment-like map
# and checks the exact command strings that would be sent to the Pi.
#
# No real network is used: we call PiConnection.send_state/send_seg on a
# FakeConnection object that only implements send_line().

import os
import sys
import math

import pytest

# Ensure project root ("fyp") is on sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.planner import Node, Edge, find_closest_node, plan_path
from navigation.segments import path_to_segments
from navigation.net_client import PiConnection


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------

def _apartment_nodes_edges():
    """
    Same apartment-style layout used in other tests:

        Kitchen (3)
             ^
             |
    Base(0)-1-2--Bedroom(4)

    All edges length 1.0
    """
    nodes = [
        Node(id=0, x=0.0, z=0.0),     # Base
        Node(id=1, x=1.0, z=0.0),     # Junction
        Node(id=2, x=2.0, z=0.0),     # Corridor
        Node(id=3, x=2.0, z=1.0),     # Kitchen
        Node(id=4, x=2.0, z=-1.0),    # Bedroom
    ]
    edges = [
        Edge(src=0, dst=1, dist=1.0),
        Edge(src=1, dst=2, dist=1.0),
        Edge(src=2, dst=3, dist=1.0),
        Edge(src=2, dst=4, dist=1.0),
    ]
    return nodes, edges


class FakeConnection:
    """
    Minimal PiConnection-like object that only records lines sent.

    We will call PiConnection.send_state/send_seg on this object, which
    will use its send_line() method instead of the real network.
    """

    def __init__(self):
        self.lines = []

    def send_line(self, text: str):
        self.lines.append(text)


# ----------------------------------------------------------------------
# Test
# ----------------------------------------------------------------------

def test_full_command_sequence_base_to_kitchen():
    nodes, edges = _apartment_nodes_edges()

    # Current pose is near Base
    current_x, current_z = 0.05, -0.02
    start_id = find_closest_node(nodes, current_x, current_z)
    assert start_id == 0    # Base

    goal_id = 3    # Kitchen

    # 1) Plan path
    path = plan_path(nodes, edges, start_id=start_id, goal_id=goal_id)

    # Shortest expected path: 0 -> 1 -> 2 -> 3
    assert path == [0, 1, 2, 3]

    # 2) Convert path to motion segments
    v_forward = 1.0            # m/s
    omega_turn = math.pi / 2    # 90 deg/s
    initial_heading = 0.0      # facing +x

    segments = path_to_segments(
        nodes=nodes,
        path=path,
        v_forward=v_forward,
        omega_turn=omega_turn,
        initial_heading=initial_heading,
    )

    # Expected directions: forward, forward, left turn, forward
    directions = [s.direction for s in segments]
    assert directions == ["w", "w", "a", "w"]

    # Durations: each straight edge is 1m at 1m/s => 1s
    # Left turn is 90deg at 90deg/s => 1s
    durations = [s.duration for s in segments]
    assert pytest.approx(durations[0], rel=1e-6) == 1.0    # 0->1
    assert pytest.approx(durations[1], rel=1e-6) == 1.0    # 1->2
    assert pytest.approx(durations[2], rel=1e-6) == 1.0    # turn
    assert pytest.approx(durations[3], rel=1e-6) == 1.0    # 2->3

    # 3) Turn segments into text commands using PiConnection helpers,
    # but on a FakeConnection so no real network is used.
    conn = FakeConnection()

    # STATE NAVIGATE
    PiConnection.send_state(conn, "navigate")

    # SEG commands for each segment
    for seg in segments:
        # Only motion segments (w/a/d).
        # adds other types, you can extend this mapping.
        PiConnection.send_seg(conn, seg.direction, seg.duration)

    # 4) Check the final list of command strings
    commands = conn.lines

    assert commands[0] == "STATE NAVIGATE"
    # SEG durations are formatted to 3 decimals inside send_seg
    assert commands[1:] == [
        "SEG w 1.000",
        "SEG w 1.000",
        "SEG a 1.000",
        "SEG w 1.000",
    ]
