# test/test_segments.py

import os
import sys
import math

import pytest

# Ensure project root is on sys.path so 'navigation' package is importable
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.planner import Node    # reuse Node dataclass
from navigation.segments import Segment, path_to_segments, _normalize_angle


def _nodes_for_path(coords):
    """
    Helper: create Nodes with ids 0..n-1 from list of (x, z) pairs.
    """
    return [Node(id=i, x=x, z=z) for i, (x, z) in enumerate(coords)]


# ----------------------------------------------------------------------
# BASIC TESTS (the original ones)
# ----------------------------------------------------------------------

# ---------- Tests for _normalize_angle ----------

def test_normalize_angle_within_range():
    # Values already in [-pi, pi] should not change much
    for angle in [-math.pi, -1.0, 0.0, 1.0, math.pi]:
        a = _normalize_angle(angle)
        assert -math.pi <= a <= math.pi


def test_normalize_angle_wraps_large_positive():
    # 3*pi should wrap to +pi
    a = _normalize_angle(3.0 * math.pi)
    assert pytest.approx(a, rel=1e-6) == math.pi


def test_normalize_angle_wraps_large_negative():
    # -4*pi should wrap to 0
    a = _normalize_angle(-4.0 * math.pi)
    assert pytest.approx(a, rel=1e-6) == 0.0


# ---------- Tests for path_to_segments (basic) ----------

def test_path_to_segments_straight_line_no_turns():
    # Simple path: 0 -> 1 -> 2 along +x axis, 1m apart
    nodes = _nodes_for_path([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
    path = [0, 1, 2]

    v_forward = 1.0    # m/s
    omega_turn = math.pi    # rad/s (not used because no turns)
    initial_heading = 0.0    # facing +x initially

    segs = path_to_segments(
        nodes=nodes,
        path=path,
        v_forward=v_forward,
        omega_turn=omega_turn,
        initial_heading=initial_heading,
    )

    # Expect two forward segments, each for distance=1m at 1m/s => 1s
    assert len(segs) == 2
    for seg in segs:
        assert isinstance(seg, Segment)
        assert seg.direction == "w"
        assert pytest.approx(seg.duration, rel=1e-6) == 1.0


def test_path_to_segments_left_turn_then_forward():
    # Path: go +x, then turn left and go +z
    # Nodes: (0,0) -> (1,0) -> (1,1)
    nodes = _nodes_for_path([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    path = [0, 1, 2]

    v_forward = 1.0
    omega_turn = math.pi / 2.0    # 90 deg per second
    initial_heading = 0.0    # facing +x

    segs = path_to_segments(
        nodes=nodes,
        path=path,
        v_forward=v_forward,
        omega_turn=omega_turn,
        initial_heading=initial_heading,
    )

    # Expected:
    # - first segment: forward 1m => 'w', 1s
    # - second node pair: need +90deg turn => 'a', 1s
    # - then forward 1m => 'w', 1s
    directions = [s.direction for s in segs]
    durations = [s.duration for s in segs]

    assert directions == ["w", "a", "w"]
    assert pytest.approx(durations[0], rel=1e-6) == 1.0
    assert pytest.approx(durations[1], rel=1e-6) == 1.0
    assert pytest.approx(durations[2], rel=1e-6) == 1.0


def test_path_to_segments_right_turn_then_forward():
    # Path: go +x, then turn right and go -z
    # Nodes: (0,0) -> (1,0) -> (1,-1)
    nodes = _nodes_for_path([(0.0, 0.0), (1.0, 0.0), (1.0, -1.0)])
    path = [0, 1, 2]

    v_forward = 1.0
    omega_turn = math.pi / 2.0    # 90 deg per second
    initial_heading = 0.0    # facing +x

    segs = path_to_segments(
        nodes=nodes,
        path=path,
        v_forward=v_forward,
        omega_turn=omega_turn,
        initial_heading=initial_heading,
    )

    directions = [s.direction for s in segs]
    durations = [s.duration for s in segs]

    # Expected: forward, then 90deg right turn, then forward
    assert directions == ["w", "d", "w"]
    assert all(d > 0 for d in durations)


def test_path_to_segments_single_node_gives_no_segments():
    nodes = _nodes_for_path([(0.0, 0.0)])
    path = [0]

    segs = path_to_segments(
        nodes=nodes,
        path=path,
        v_forward=1.0,
        omega_turn=math.pi,
        initial_heading=0.0,
    )

    assert segs == []


def test_path_to_segments_zero_distance_segment_is_skipped():
    # Two nodes at the same coordinates: distance = 0 => no forward segment
    nodes = _nodes_for_path([(0.0, 0.0), (0.0, 0.0)])
    path = [0, 1]

    segs = path_to_segments(
        nodes=nodes,
        path=path,
        v_forward=1.0,
        omega_turn=math.pi,
        initial_heading=0.0,
    )

    # dx=dz=0 => dist=0, so no 'w' segment; also heading doesn't change => no turn
    assert segs == []


# ----------------------------------------------------------------------
# MORE REALISTIC / COMPLEX SCENARIOS
# ----------------------------------------------------------------------

def test_path_to_segments_initial_heading_misaligned():
    """
    Robot starts facing +z (north) but first movement is +x (east).
    We expect a right turn ('d') then forward.
    """
    nodes = _nodes_for_path([(0.0, 0.0), (1.0, 0.0)])    # move east
    path = [0, 1]

    v_forward = 1.0
    omega_turn = math.pi / 2.0    # 90 deg/s
    initial_heading = math.pi / 2.0    # facing +z (north)

    segs = path_to_segments(
        nodes=nodes,
        path=path,
        v_forward=v_forward,
        omega_turn=omega_turn,
        initial_heading=initial_heading,
    )

    # Need a -90deg turn (right) then forward
    assert len(segs) == 2
    assert segs[0].direction == "d"
    # 90deg / (pi/2 rad/s) => 1s
    assert pytest.approx(segs[0].duration, rel=1e-6) == 1.0
    assert segs[1].direction == "w"
    # 1m / 1m/s => 1s
    assert pytest.approx(segs[1].duration, rel=1e-6) == 1.0


def test_path_to_segments_zigzag_apartment_path():
    """
    Simulate a small 'apartment' zig-zag:
      (0,0)=Base -> (1,0) -> (2,0) -> (2,1)=Kitchen -> (2,0) -> (2,-1)=Bedroom

    Expect:
      w (0->1),
      w (1->2),
      a, w (2->Kitchen),
      d, w (Kitchen->corridor),
      w (corridor->Bedroom)
    """
    nodes = _nodes_for_path([
        (0.0, 0.0),     # 0 Base
        (1.0, 0.0),     # 1 Junction
        (2.0, 0.0),     # 2 Corridor end
        (2.0, 1.0),     # 3 Kitchen
        (2.0, -1.0),    # 4 Bedroom
    ])
    path = [0, 1, 2, 3, 2, 4]

    v_forward = 1.0
    omega_turn = math.pi / 2.0    # 90deg/s
    initial_heading = 0.0    # facing +x

    segs = path_to_segments(
        nodes=nodes,
        path=path,
        v_forward=v_forward,
        omega_turn=omega_turn,
        initial_heading=initial_heading,
    )

    directions = [s.direction for s in segs]

    # Expected: forward, forward, left-turn, forward, right-turn, forward, forward
    assert directions == ["w", "w", "a", "w", "d", "w", "w"]

    # All forward segments should be positive duration
    forward_durations = [s.duration for s in segs if s.direction == "w"]
    assert all(d > 0 for d in forward_durations)


def test_path_to_segments_respects_min_turn_angle():
    """
    If the angle difference is tiny (< min_turn_angle), we should not generate
    a turn segment, only forwards.
    """
    # (0,0) -> (1,0) -> (2, tiny_z)
    tiny_z = 1e-4
    nodes = _nodes_for_path([(0.0, 0.0), (1.0, 0.0), (2.0, tiny_z)])
    path = [0, 1, 2]

    v_forward = 1.0
    omega_turn = math.pi
    initial_heading = 0.0

    segs = path_to_segments(
        nodes=nodes,
        path=path,
        v_forward=v_forward,
        omega_turn=omega_turn,
        initial_heading=initial_heading,
        min_turn_angle=1e-3,    # bigger than the tiny angle we introduce
    )

    # Should only be forward segments (no 'a'/'d')
    directions = [s.direction for s in segs]
    assert all(d == "w" for d in directions)


def test_path_to_segments_respects_min_forward_dist():
    """
    If the forward distance is tiny (< min_forward_dist), we should skip
    that forward segment completely.
    """
    # Move almost nowhere in second step
    nodes = _nodes_for_path([(0.0, 0.0), (1e-4, 0.0)])
    path = [0, 1]

    segs = path_to_segments(
        nodes=nodes,
        path=path,
        v_forward=1.0,
        omega_turn=math.pi,
        initial_heading=0.0,
        min_forward_dist=1e-3,    # threshold larger than actual distance
    )

    # No forward segment should be produced
    assert segs == []


def test_path_to_segments_raises_on_non_positive_speeds():
    nodes = _nodes_for_path([(0.0, 0.0), (1.0, 0.0)])
    path = [0, 1]

    with pytest.raises(ValueError):
        path_to_segments(
            nodes=nodes,
            path=path,
            v_forward=0.0,
            omega_turn=math.pi,
            initial_heading=0.0,
        )

    with pytest.raises(ValueError):
        path_to_segments(
            nodes=nodes,
            path=path,
            v_forward=1.0,
            omega_turn=0.0,
            initial_heading=0.0,
        )


def test_path_to_segments_raises_on_unknown_node_id():
    nodes = _nodes_for_path([(0.0, 0.0), (1.0, 0.0)])
    # Node id 2 does not exist in nodes
    path = [0, 2]

    with pytest.raises(ValueError):
        path_to_segments(
            nodes=nodes,
            path=path,
            v_forward=1.0,
            omega_turn=math.pi,
            initial_heading=0.0,
        )
