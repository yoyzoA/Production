# test/test_navigation_scenario.py
#
# Integration-style tests for the navigation pipeline on the laptop:
# MapData (nodes/edges/rooms) -> plan_path -> path_to_segments
#
# This does NOT touch the Pi, sockets, HTTP, or hardware.
# - navigation.planner
# - navigation.segments
# - navigation.map_loader (MapData container)

import os
import sys
import math

import pytest

# Ensure project root ("fyp") is on sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.planner import Node, Edge, find_closest_node, plan_path
from navigation.segments import path_to_segments, Segment
from navigation.map_loader import MapData


# ----------------------------------------------------------------------
# Build a small "apartment-style" map used in all tests
# ----------------------------------------------------------------------

def _build_apartment_map() -> MapData:
    """
    Simple apartment-like layout:

        Kitchen (3)
             ^
             |
    Base(0)-1-2--Bedroom(4)

    All edges are length 1.0 for simplicity.

    Rooms mapping (name -> node id):
      - "Base" -> 0
      - "Kitchen" -> 3
      - "Bedroom" -> 4
    """
    nodes = [
        Node(id=0, x=0.0, z=0.0),     # Base
        Node(id=1, x=1.0, z=0.0),     # Junction
        Node(id=2, x=2.0, z=0.0),     # Corridor end
        Node(id=3, x=2.0, z=1.0),     # Kitchen
        Node(id=4, x=2.0, z=-1.0),    # Bedroom
    ]
    edges = [
        Edge(src=0, dst=1, dist=1.0),
        Edge(src=1, dst=2, dist=1.0),
        Edge(src=2, dst=3, dist=1.0),
        Edge(src=2, dst=4, dist=1.0),
    ]
    rooms = {
        "Base": 0,
        "Kitchen": 3,
        "Bedroom": 4,
    }
    return MapData(nodes=nodes, edges=edges, rooms=rooms)


def _check_path_contiguous(nodes, edges, path):
    """Helper: assert that each step in path follows a real edge."""
    # Build adjacency for easy lookup
    adj = {n.id: set() for n in nodes}
    for e in edges:
        adj[e.src].add(e.dst)
        adj[e.dst].add(e.src)

    for a, b in zip(path, path[1:]):
        assert b in adj[a], f"Path uses non-existent edge {a}->{b}"


# ----------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------

def test_end_to_end_base_to_kitchen_segments():
    """
    Full pipeline:
      - start pose near Base
      - use find_closest_node to pick start node
      - plan_path to Kitchen
      - convert to segments
    """
    m = _build_apartment_map()

    # Pretend ZED / rover is currently near Base
    current_x, current_z = 0.05, -0.02
    start_id = find_closest_node(m.nodes, current_x, current_z)
    assert start_id == m.rooms["Base"]

    goal_id = m.rooms["Kitchen"]

    # 1) Plan path
    path = plan_path(m.nodes, m.edges, start_id=start_id, goal_id=goal_id)

    # Expected shortest path: 0 -> 1 -> 2 -> 3
    assert path[0] == start_id
    assert path[-1] == goal_id
    _check_path_contiguous(m.nodes, m.edges, path)
    assert path == [0, 1, 2, 3]

    # 2) Convert path to segments
    v_forward = 0.5            # m/s
    omega_turn = math.pi/2     # 90 deg/s
    initial_heading = 0.0      # facing +x along corridor

    segments = path_to_segments(
        nodes=m.nodes,
        path=path,
        v_forward=v_forward,
        omega_turn=omega_turn,
        initial_heading=initial_heading,
    )

    # We expect:
    # 0->1: forward
    # 1->2: forward
    # 2->3: left turn, then forward
    directions = [s.direction for s in segments]
    assert directions == ["w", "w", "a", "w"]

    # All durations must be positive
    for s in segments:
        assert isinstance(s, Segment)
        assert s.duration > 0.0

    # Total forward distance should be 3m.
    # With v_forward=0.5 m/s, total forward time = 3 / 0.5 = 6s.
    forward_time = sum(s.duration for s in segments if s.direction == "w")
    assert pytest.approx(forward_time, rel=1e-6) == 6.0


def test_bedroom_to_kitchen_path_and_segments():
    """
    Go from Bedroom to Kitchen:
      - shortest path should be: 4 -> 2 -> 3
      - segments: right-turn (to face corridor), forward, left-turn, forward.
    """
    m = _build_apartment_map()

    start_id = m.rooms["Bedroom"]
    goal_id = m.rooms["Kitchen"]

    # Initial heading: facing -z (south) inside Bedroom
    initial_heading = -math.pi / 2.0

    path = plan_path(m.nodes, m.edges, start_id=start_id, goal_id=goal_id)
    _check_path_contiguous(m.nodes, m.edges, path)

    assert path[0] == start_id
    assert path[-1] == goal_id
    assert path == [4, 2, 3]

    v_forward = 1.0
    omega_turn = math.pi / 2.0    # 90deg/s

    segments = path_to_segments(
        nodes=m.nodes,
        path=path,
        v_forward=v_forward,
        omega_turn=omega_turn,
        initial_heading=initial_heading,
    )

    directions = [s.direction for s in segments]

    # Expect:
    # 4->2: small heading adjustment, then forward along corridor (+x/up)
    # 2->3: left turn then forward (+z)
    #
    # Exact sequence can vary slightly depending on your path_to_segments
    # implementation; we check core properties instead of exact pattern.
    assert "w" in directions    # must have some forward motion
    assert any(d in ("a", "d") for d in directions)    # must have at least one turn

    # Durations should all be positive
    for s in segments:
        assert s.duration > 0.0


def test_find_closest_node_matches_room_nodes():
    """
    For several points near each room, find_closest_node should match
    the room's node id from MapData.rooms.
    """
    m = _build_apartment_map()

    # Points near Base, Kitchen, Bedroom
    near_base    = (0.1,  0.0)
    near_kitchen = (2.0,  1.1)
    near_bedroom = (2.0, -1.2)

    assert find_closest_node(m.nodes, *near_base)    == m.rooms["Base"]
    assert find_closest_node(m.nodes, *near_kitchen) == m.rooms["Kitchen"]
    assert find_closest_node(m.nodes, *near_bedroom) == m.rooms["Bedroom"]
