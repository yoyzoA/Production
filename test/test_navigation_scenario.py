# test/test_navigation_scenario.py

import os
import sys
import math

# ensure root on path
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.planner import Node, Edge, plan_path
from navigation.segments import path_to_segments
from navigation.map_loader import MapData
import navigation.net_client as net_client


class FakeConnection:
    """Fake PiConnection that just records lines sent."""
    def __init__(self):
        self.lines = []

    def send_line(self, text: str):
        self.lines.append(text)

    # wrappers to mimic PiConnection API
    def send_state(self, name: str):
        net_client.PiConnection.send_state(self, name)

    def send_seg(self, key: str, duration: float):
        net_client.PiConnection.send_seg(self, key, duration)


def test_full_base_to_kitchen_sequence():
    # 1) Build tiny map: Base (0,0) -> (1,0) -> Kitchen (2,0)
    nodes = [
        Node(id=0, x=0.0, z=0.0),    # Base
        Node(id=1, x=1.0, z=0.0),
        Node(id=2, x=2.0, z=0.0),    # Kitchen
    ]
    edges = [
        Edge(src=0, dst=1, dist=1.0),
        Edge(src=1, dst=2, dist=1.0),
        # assume undirected graph; planner will treat them accordingly
    ]
    rooms = {"Base": 0, "Kitchen": 2}
    m = MapData(nodes=nodes, edges=edges, rooms=rooms)

    # 2) Plan from Base to Kitchen
    start_id = m.rooms["Base"]
    goal_id = m.rooms["Kitchen"]
    path = plan_path(m.nodes, m.edges, start_id, goal_id)
    assert path == [0, 1, 2]

    # 3) Convert path to segments
    v_forward = 0.5        # m/s
    omega_turn = math.pi    # rad/s
    initial_heading = 0.0

    segs = path_to_segments(
        nodes=m.nodes,
        path=path,
        v_forward=v_forward,
        omega_turn=omega_turn,
        initial_heading=initial_heading,
    )

    # Two forward segments of 1m each at 0.5 m/s => 2s each
    assert len(segs) == 2
    assert all(s.direction == "w" for s in segs)

    # 4) "Send" these segments to fake PiConnection
    conn = FakeConnection()

    # Start navigating
    net_client.PiConnection.send_state(conn, "navigate")
    for seg in segs:
        net_client.PiConnection.send_seg(conn, seg.direction, seg.duration)

    # 5) Check protocol lines look realistic
    # First line is STATE NAVIGATE, then 2x SEG w <duration>
    assert conn.lines[0] == "STATE NAVIGATE"
    assert conn.lines[1].startswith("SEG w ")
    assert conn.lines[2].startswith("SEG w ")
