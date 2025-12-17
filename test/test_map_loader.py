# test/test_map_loader.py
#
# Tests for navigation.map_loader:
# - MapData container
# - save_map / load_map JSON round-trip
# - room helpers: get_room_node / set_room_node / get_node_position

import os
import sys

import pytest

# Ensure project root ("fyp") is on sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.planner import Node, Edge
from navigation.map_loader import (
    MapData,
    save_map,
    load_map,
    get_room_node,
    set_room_node,
    get_node_position,
)


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------

def _simple_map() -> MapData:
    """
    Tiny line:
        0 --1m-- 1

    Rooms:
        "Base" -> 0
        "Room1" -> 1
    """
    nodes = [
        Node(id=0, x=0.0, z=0.0),
        Node(id=1, x=1.0, z=0.0),
    ]
    edges = [
        Edge(src=0, dst=1, dist=1.0),
    ]
    rooms = {"Base": 0, "Room1": 1}
    return MapData(nodes=nodes, edges=edges, rooms=rooms)


def _apartment_map() -> MapData:
    """
    Apartment-style layout:

        Kitchen (3)
             ^
             |
    Base(0)-1-2--Bedroom(4)

    All edges length 1.0

    Rooms:
        "Base"    -> 0
        "Junction"-> 1
        "Corridor"-> 2
        "Kitchen" -> 3
        "Bedroom" -> 4
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
    rooms = {
        "Base": 0,
        "Junction": 1,
        "Corridor": 2,
        "Kitchen": 3,
        "Bedroom": 4,
    }
    return MapData(nodes=nodes, edges=edges, rooms=rooms)


# ----------------------------------------------------------------------
# Basic round-trip tests
# ----------------------------------------------------------------------

def test_save_and_load_roundtrip_simple(tmp_path):
    m = _simple_map()
    path = tmp_path / "map_simple.json"

    save_map(path, m)
    loaded = load_map(path)

    # Nodes, edges, and rooms should match
    assert loaded.nodes == m.nodes
    assert loaded.edges == m.edges
    assert loaded.rooms == m.rooms


def test_save_and_load_roundtrip_apartment(tmp_path):
    m = _apartment_map()
    path = tmp_path / "map_apartment.json"

    save_map(path, m)
    loaded = load_map(path)

    # Same number of nodes/edges/rooms
    assert len(loaded.nodes) == len(m.nodes)
    assert len(loaded.edges) == len(m.edges)
    assert loaded.rooms == m.rooms

    # Node IDs and positions should match
    original_positions = {n.id: (n.x, n.z) for n in m.nodes}
    loaded_positions = {n.id: (n.x, n.z) for n in loaded.nodes}
    assert loaded_positions == original_positions

    # Edges: compare after sorting, allow tiny float differences on dist
    def edge_key(e: Edge):
        return (e.src, e.dst, e.dist)

    orig_edges_sorted = sorted(m.edges, key=edge_key)
    loaded_edges_sorted = sorted(loaded.edges, key=edge_key)

    assert len(orig_edges_sorted) == len(loaded_edges_sorted)
    for e_orig, e_loaded in zip(orig_edges_sorted, loaded_edges_sorted):
        assert e_orig.src == e_loaded.src
        assert e_orig.dst == e_loaded.dst
        assert pytest.approx(e_orig.dist, rel=1e-6) == e_loaded.dist


def test_save_and_load_empty_map(tmp_path):
    empty = MapData(nodes=[], edges=[], rooms={})
    path = tmp_path / "empty.json"

    save_map(path, empty)
    loaded = load_map(path)

    assert loaded.nodes == []
    assert loaded.edges == []
    assert loaded.rooms == {}


# ----------------------------------------------------------------------
# Room / position helpers
# ----------------------------------------------------------------------

def test_get_room_node_and_get_node_position_simple():
    m = _simple_map()

    base_id = get_room_node(m, "Base")
    room1_id = get_room_node(m, "Room1")

    assert base_id == 0
    assert room1_id == 1

    # Positions should match underlying nodes
    base_pos = get_node_position(m, base_id)
    room1_pos = get_node_position(m, room1_id)

    assert base_pos == (0.0, 0.0)
    assert room1_pos == (1.0, 0.0)


def test_set_room_node_updates_mapping():
    m = _apartment_map()

    # Initially, "Kitchen" is mapped to node 3
    assert get_room_node(m, "Kitchen") == 3

    # Move "Kitchen" to Bedroom node (id 4) for some weird reason
    set_room_node(m, "Kitchen", 4)

    assert get_room_node(m, "Kitchen") == 4
    # Bedroom still mapped to 4 as well; multiple room names can share a node
    assert get_room_node(m, "Bedroom") == 4


def test_get_node_position_for_all_apartment_nodes():
    m = _apartment_map()

    for node in m.nodes:
        pos = get_node_position(m, node.id)
        assert pos == (node.x, node.z)
