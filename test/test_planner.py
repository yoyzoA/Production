# test/test_planner.py

import os
import sys
import math

import pytest

# Ensure "navigation" package is importable no matter how pytest sets sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.planner import (
    Node,
    Edge,
    find_closest_node,
    plan_path,
    _build_adjacency,
)


# ---------- Basic tests for find_closest_node (existing) ----------

def test_find_closest_node_basic():
    nodes = [
        Node(id=0, x=0.0, z=0.0),
        Node(id=1, x=1.0, z=0.0),
        Node(id=2, x=2.0, z=0.0),
    ]

    # Very close to node 0
    assert find_closest_node(nodes, 0.1, 0.0) == 0

    # In the middle between node 1 and 2 but closer to 2
    assert find_closest_node(nodes, 1.8, 0.0) == 2

    # Above node 1, should still pick node 1
    assert find_closest_node(nodes, 1.0, 0.5) == 1


def test_find_closest_node_empty_list_returns_none():
    assert find_closest_node([], 0.0, 0.0) is None


# ---------- Basic tests for _build_adjacency (existing) ----------

def test_build_adjacency_undirected_and_skips_invalid_edges():
    nodes = [
        Node(id=0, x=0.0, z=0.0),
        Node(id=1, x=1.0, z=0.0),
        Node(id=2, x=2.0, z=0.0),
    ]

    valid_edge = Edge(src=0, dst=1, dist=1.0)
    invalid_edge = Edge(src=0, dst=99, dist=123.0)    # 99 doesn't exist

    adj = _build_adjacency(nodes, [valid_edge, invalid_edge])

    # Adjacency should have entries for ALL node IDs, even if isolated
    assert set(adj.keys()) == {0, 1, 2}

    # Edge 0 <-> 1 should be present in both directions
    neighbors_0 = dict(adj[0])
    neighbors_1 = dict(adj[1])
    neighbors_2 = dict(adj[2])

    assert 1 in neighbors_0
    assert math.isclose(neighbors_0[1], 1.0)

    assert 0 in neighbors_1
    assert math.isclose(neighbors_1[0], 1.0)

    # Node 2 has no neighbors in this simple graph
    assert neighbors_2 == {}

    # The invalid edge (0 -> 99) should have been ignored


# ---------- Basic tests for plan_path (existing) ----------

def _simple_line_graph():
    """Helper: 0 --1m-- 1 --1m-- 2"""
    nodes = [
        Node(id=0, x=0.0, z=0.0),
        Node(id=1, x=1.0, z=0.0),
        Node(id=2, x=2.0, z=0.0),
    ]
    edges = [
        Edge(src=0, dst=1, dist=1.0),
        Edge(src=1, dst=2, dist=1.0),
    ]
    return nodes, edges


def test_plan_path_simple_line():
    nodes, edges = _simple_line_graph()

    path = plan_path(nodes, edges, start_id=0, goal_id=2)

    assert path == [0, 1, 2]


def test_plan_path_prefers_shorter_path():
    # Graph:
    # 0 --1-- 1 --1-- 3
    # \ ^
    # \-----5--------/
    nodes = [
        Node(id=0, x=0.0, z=0.0),
        Node(id=1, x=1.0, z=0.0),
        Node(id=2, x=0.0, z=1.0),    # unused node
        Node(id=3, x=2.0, z=0.0),
    ]
    edges = [
        Edge(src=0, dst=1, dist=1.0),
        Edge(src=1, dst=3, dist=1.0),
        Edge(src=0, dst=3, dist=5.0),    # longer direct edge
    ]

    path = plan_path(nodes, edges, start_id=0, goal_id=3)

    # Dijkstra should pick the cheaper 0 -> 1 -> 3 (cost 2.0) not 0 -> 3 (cost 5.0)
    assert path == [0, 1, 3]


def test_plan_path_start_equals_goal():
    nodes, edges = _simple_line_graph()

    path = plan_path(nodes, edges, start_id=1, goal_id=1)

    # Should return a trivial path with a single node
    assert path == [1]


def test_plan_path_raises_on_unreachable_goal():
    # Two disconnected components: (0-1) and (2-3)
    nodes = [
        Node(id=0, x=0.0, z=0.0),
        Node(id=1, x=1.0, z=0.0),
        Node(id=2, x=10.0, z=0.0),
        Node(id=3, x=11.0, z=0.0),
    ]
    edges = [
        Edge(src=0, dst=1, dist=1.0),
        Edge(src=2, dst=3, dist=1.0),
    ]

    with pytest.raises(ValueError):
        plan_path(nodes, edges, start_id=0, goal_id=3)


# ======================================================================
# MORE REALISTIC: "APARTMENT" GRAPH SCENARIOS
# ======================================================================

def _apartment_graph():
    """
    Build a tiny 'apartment-like' graph:

        (Kitchen=3)
              ^
              |
      Base=0--1--2--(Bedroom=4)

    Nodes:
      0: Base      at (0,0)
      1: Junction  at (1,0)
      2: Corridor2 at (2,0)
      3: Kitchen   at (2,1)
      4: Bedroom   at (2,-1)
    All edges have distance 1.0
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
    return nodes, edges


def _path_cost(edges, path):
    """Compute total cost of a path using the edge list."""
    # Build lookup: (src, dst) -> dist (undirected)
    lookup = {}
    for e in edges:
        lookup[(e.src, e.dst)] = e.dist
        lookup[(e.dst, e.src)] = e.dist

    cost = 0.0
    for a, b in zip(path, path[1:]):
        cost += lookup[(a, b)]
    return cost


@pytest.mark.parametrize(
    "start, goal, expected_cost",
    [
        (0, 3, 3.0),    # Base -> Kitchen : 0-1-2-3
        (0, 4, 3.0),    # Base -> Bedroom : 0-1-2-4
        (4, 3, 2.0),    # Bedroom -> Kitchen : 4-2-3
    ],
)
def test_apartment_graph_paths_have_minimal_cost(start, goal, expected_cost):
    nodes, edges = _apartment_graph()
    path = plan_path(nodes, edges, start_id=start, goal_id=goal)

    # Path must start and end correctly
    assert path[0] == start
    assert path[-1] == goal

    # Path must be contiguous: every step follows an existing edge
    adj = _build_adjacency(nodes, edges)
    for a, b in zip(path, path[1:]):
        neighbors = [n for (n, _) in adj[a]]
        assert b in neighbors, f"Edge {a}->{b} not in graph"

    # Total path cost must match expected minimal cost
    cost = _path_cost(edges, path)
    assert math.isclose(cost, expected_cost, rel_tol=1e-6)

def test_apartment_find_closest_node_near_rooms():
    nodes, _ = _apartment_graph()

    # Close to Base
    assert find_closest_node(nodes, 0.1, 0.1) == 0

    # Close to Kitchen
    assert find_closest_node(nodes, 2.0, 1.2) == 3

    # Close to Bedroom
    assert find_closest_node(nodes, 2.1, -1.1) == 4
