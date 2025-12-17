# navigation/map_loader.py
#
# Load and save the navigation map from a JSON file.
#
# Expected JSON format (example):
#
# {
# "nodes": [
# {"id": 0, "x": 0.00, "z": 0.00},
# {"id": 1, "x": 1.00, "z": 0.20}
# ],
# "edges": [
# {"src": 0, "dst": 1, "dist": 1.02}
# ],
# "rooms": {
# "Base": {"node_id": 0},
# "Kitchen": {"node_id": 1}
# }
# }
#
# - nodes: list of graph nodes with unique id and (x,z) coordinates [m]
# - edges: connections with distance [m]
# - rooms: mapping from room name to a node_id in the graph

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
import json
import os

from .planner import Node, Edge


@dataclass
class MapData:
    """Container for navigation map data."""
    nodes: List[Node]
    edges: List[Edge]
    # rooms: maps room name -> node_id
    rooms: Dict[str, int] = field(default_factory=dict)


def load_map(path: str) -> MapData:
    """
    Load map data from a JSON file.

    Args:
        path: path to the JSON map file.

    Returns:
        MapData object with nodes, edges, and rooms mapping.

    Raises:
        FileNotFoundError: if the file does not exist.
        ValueError: if the file content is invalid.
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Map file not found: {path}")

    with open(path, "r") as f:
        data = json.load(f)

    # Parse nodes
    raw_nodes = data.get("nodes", [])
    nodes: List[Node] = []
    for n in raw_nodes:
        try:
            node_id = int(n["id"])
            x = float(n["x"])
            z = float(n["z"])
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f"Invalid node entry in map file: {n}") from e

        nodes.append(Node(id=node_id, x=x, z=z))

    # Parse edges
    raw_edges = data.get("edges", [])
    edges: List[Edge] = []
    for e in raw_edges:
        try:
            src = int(e["src"])
            dst = int(e["dst"])
            dist = float(e["dist"])
        except (KeyError, TypeError, ValueError) as ex:
            raise ValueError(f"Invalid edge entry in map file: {e}") from ex

        edges.append(Edge(src=src, dst=dst, dist=dist))

    # Parse rooms (room_name -> node_id)
    raw_rooms = data.get("rooms", {})
    rooms: Dict[str, int] = {}
    for room_name, info in raw_rooms.items():
        try:
            node_id = int(info["node_id"])
        except (KeyError, TypeError, ValueError) as ex:
            raise ValueError(f"Invalid room entry in map file: {room_name}: {info}") from ex

        rooms[str(room_name)] = node_id

    return MapData(nodes=nodes, edges=edges, rooms=rooms)


def save_map(path: str, map_data: MapData) -> None:
    """
    Save MapData to a JSON file.

    Args:
        path: path to JSON file to write.
        map_data: MapData instance.
    """
    data = {
        "nodes": [
            {"id": n.id, "x": n.x, "z": n.z}
            for n in map_data.nodes
        ],
        "edges": [
            {"src": e.src, "dst": e.dst, "dist": e.dist}
            for e in map_data.edges
        ],
        "rooms": {
            room_name: {"node_id": node_id}
            for room_name, node_id in map_data.rooms.items()
        },
    }

    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def get_room_node(map_data: MapData, room_name: str) -> Optional[int]:
    """
    Get the node_id associated with a room name.

    Args:
        map_data: MapData object.
        room_name: name of the room (e.g. "Kitchen").

    Returns:
        node_id if found, otherwise None.
    """
    return map_data.rooms.get(room_name)


def set_room_node(map_data: MapData, room_name: str, node_id: int) -> None:
    """
    Associate a room name with a node_id in the map.

    Args:
        map_data: MapData object (modified in-place).
        room_name: room label (e.g. "Kitchen").
        node_id: id of the node that represents this room.
    """
    map_data.rooms[room_name] = node_id


def get_node_position(map_data: MapData, node_id: int) -> Optional[Tuple[float, float]]:
    """
    Get the (x, z) coordinates of a node.

    Args:
        map_data: MapData object.
        node_id: id of the node.

    Returns:
        (x, z) tuple if node exists, otherwise None.
    """
    for n in map_data.nodes:
        if n.id == node_id:
            return (n.x, n.z)
    return None


# Optional: quick test
if __name__ == "__main__":
    # Small in-memory example
    nodes = [Node(id=0, x=0.0, z=0.0), Node(id=1, x=1.0, z=0.0)]
    edges = [Edge(src=0, dst=1, dist=1.0)]
    rooms = {"Base": 0, "Kitchen": 1}

    m = MapData(nodes=nodes, edges=edges, rooms=rooms)
    test_path = "test_map.json"

    save_map(test_path, m)
    print(f"Saved test map to {test_path}")

    m2 = load_map(test_path)
    print("Loaded map:", m2)

    print("Kitchen node:", get_room_node(m2, "Kitchen"))
    print("Node 1 position:", get_node_position(m2, 1))
