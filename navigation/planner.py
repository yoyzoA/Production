# navigation/planner.py
#
# Path planning over a simple graph using Dijkstra's algorithm.
# This module is PURE logic: no ZED, no networking, no Pi.
#
# It expects:
# - A list of Node objects with id, x, z
# - A list of Edge objects with src, dst, dist (meters)
#
# Main functions:
# - find_closest_node(nodes, x, z) -> node_id
# - plan_path(nodes, edges, start_id, goal_id) -> list of node_ids
#
# The graph is treated as UNDIRECTED (edges in both directions).

from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
import math
import heapq


@dataclass
class Node:
    """
    One node in the navigation graph.

    Attributes:
        id: Integer ID, unique per node. Used in edges and planning.
        x:  X coordinate in world frame [m].
        z:  Z coordinate in world frame [m].
    """
    id: int
    x: float    # [m]
    z: float    # [m]


@dataclass
class Edge:
    """
    One edge in the navigation graph.

    Attributes:
        src:  Source node id.
        dst:  Destination node id.
        dist: Cost of travelling from src to dst [m].
    """
    src: int        # node id
    dst: int        # node id
    dist: float     # [m]


def _build_adjacency(nodes: List[Node], edges: List[Edge]) -> Dict[int, List[Tuple[int, float]]]:
    """
    Build an adjacency list from nodes and edges.

    Returns:
        adj: dict mapping node_id -> list of (neighbor_id, distance).
    """
    # Initialize adjacency for all node IDs
    adj: Dict[int, List[Tuple[int, float]]] = {node.id: [] for node in nodes}

    for e in edges:
        # Treat graph as undirected: add both src->dst and dst->src
        if e.src in adj and e.dst in adj:
            adj[e.src].append((e.dst, e.dist))
            adj[e.dst].append((e.src, e.dist))
        else:
            # If an edge refers to a non-existing node, we silently skip it.
            # You can raise an error here if you want strict checking.
            pass

    return adj


def find_closest_node(nodes: List[Node], x: float, z: float) -> Optional[int]:
    """
    Find the id of the node closest to the given (x, z) position.

    Args:
        nodes: list of Node objects.
        x, z: current position in world coordinates [m].

    Returns:
        node_id of the closest node, or None if nodes is empty.
    """
    if not nodes:
        return None

    best_id = nodes[0].id
    best_d2 = (nodes[0].x - x) ** 2 + (nodes[0].z - z) ** 2

    for node in nodes[1:]:
        d2 = (node.x - x) ** 2 + (node.z - z) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_id = node.id

    return best_id


def plan_path(nodes: List[Node], edges: List[Edge], start_id: int, goal_id: int) -> List[int]:
    """
    Compute the shortest path between start_id and goal_id using Dijkstra's algorithm.

    Args:
        nodes: list of Node objects.
        edges: list of Edge objects (graph connections).
        start_id: id of the start node.
        goal_id: id of the goal node.

    Returns:
        List of node ids representing the shortest path [start_id, ..., goal_id].

    Raises:
        ValueError: if no path exists between start and goal.
    """
    if start_id == goal_id:
        return [start_id]

    # Build adjacency list
    adj = _build_adjacency(nodes, edges)

    if start_id not in adj:
        raise ValueError(f"start_id {start_id} is not present in the graph")
    if goal_id not in adj:
        raise ValueError(f"goal_id {goal_id} is not present in the graph")

    # Dijkstra: distances and parents
    dist: Dict[int, float] = {node.id: math.inf for node in nodes}
    prev: Dict[int, Optional[int]] = {node.id: None for node in nodes}

    dist[start_id] = 0.0

    # Priority queue of (distance, node_id)
    heap: List[Tuple[float, int]] = [(0.0, start_id)]
    visited: Dict[int, bool] = {}

    while heap:
        current_dist, u = heapq.heappop(heap)

        if u in visited:
            continue
        visited[u] = True

        if u == goal_id:
            break    # we reached the goal

        for v, w in adj.get(u, []):
            if v in visited:
                continue

            alt = current_dist + w
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(heap, (alt, v))

    # Reconstruct path from goal back to start
    if dist[goal_id] == math.inf:
        raise ValueError(f"No path from node {start_id} to node {goal_id}")

    path: List[int] = []
    current = goal_id
    while current is not None:
        path.append(current)
        current = prev[current]

    path.reverse()
    return path


# Optional: simple test when running this file directly
if __name__ == "__main__":
    # Tiny example graph to test
    test_nodes = [
        Node(id=0, x=0.0, z=0.0),
        Node(id=1, x=1.0, z=0.0),
        Node(id=2, x=2.0, z=0.0),
    ]
    test_edges = [
        Edge(src=0, dst=1, dist=1.0),
        Edge(src=1, dst=2, dist=1.0),
    ]

    start = 0
    goal = 2
    path = plan_path(test_nodes, test_edges, start, goal)
    print("Test path:", path)    # expected [0, 1, 2]

    closest = find_closest_node(test_nodes, 0.1, 0.0)
    print("Closest to (0.1, 0.0):", closest)
