# navigation/segments.py
#
# Convert a path of graph nodes into timed WASD segments for the rover.
#
# Input:
# - nodes: list of Node(id, x, z) (from planner.py / map_loader.py)
# - path: list of node ids [n0, n1, n2, ...] (output of planner.plan_path)
# - v_forward: forward linear speed [m/s]
# - omega_turn: angular speed for turning in place [rad/s]
# - initial_heading: optional starting heading angle [rad] in same frame as nodes
#
# Output:
# - list of Segment(direction, duration), where direction is 'w','a','s','d'.
#
# We follow this convention:
# - 'w' : move forward
# - 's' : move backward (not used in basic forward-only paths)
# - 'a' : turn left in place
# - 'd' : turn right in place
#
# Heading convention:
# - We measure heading in the (x,z) plane using atan2(dz, dx).
# - initial_heading is optional; if None, we assume we start already aligned
# with the first segment (so no initial turn).

from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import math

from .planner import Node


@dataclass
class Segment:
    """One motion segment: WASD direction for a given duration [s]."""
    direction: str     # 'w', 'a', 's', or 'd'
    duration: float    # seconds


def _normalize_angle(angle: float) -> float:
    """
    Normalize an angle to the range [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def path_to_segments(
    nodes: List[Node],
    path: List[int],
    v_forward: float,
    omega_turn: float,
    initial_heading: Optional[float] = None,
    min_forward_dist: float = 1e-3,
    min_turn_angle: float = 1e-3,
) -> List[Segment]:
    """
    Convert a list of node ids into timed WASD segments.

    Args:
        nodes: list of Node(id, x, z).
        path: list of node ids representing the planned path [n0, n1, ..., nk].
        v_forward: forward linear speed [m/s] to use for 'w' movement.
        omega_turn: angular speed [rad/s] for turning in place ('a'/'d').
        initial_heading: starting heading angle [rad] in world frame.
                         If None, we assume we are already aligned with the
                         first segment, so no initial turn is generated.
        min_forward_dist: distances below this are ignored (no forward segment).
        min_turn_angle: turn angles below this are ignored (no turn segment).

    Returns:
        List of Segment(direction, duration) to execute sequentially.

    Raises:
        ValueError if the path refers to unknown node ids or if v_forward/omega_turn
        are non-positive.
    """
    if len(path) < 2:
        # No movement needed
        return []

    if v_forward <= 0.0:
        raise ValueError("v_forward must be positive")
    if omega_turn <= 0.0:
        raise ValueError("omega_turn must be positive")

    # Build lookup: node_id -> Node
    node_by_id: Dict[int, Node] = {n.id: n for n in nodes}

    # Verify all nodes in path exist
    for node_id in path:
        if node_id not in node_by_id:
            raise ValueError(f"Node id {node_id} in path not found in nodes list")

    segments: List[Segment] = []

    # Current heading
    # If initial_heading is None, we will "snap" heading to the first segment direc...
    # without generating a turn segment.
    current_heading: Optional[float] = initial_heading

    # Iterate over consecutive node pairs
    for i in range(len(path) - 1):
        nid_from = path[i]
        nid_to = path[i + 1]

        n_from = node_by_id[nid_from]
        n_to = node_by_id[nid_to]

        dx = n_to.x - n_from.x
        dz = n_to.z - n_from.z

        # Compute desired heading for this segment
        desired_heading = math.atan2(dz, dx)    # angle in [ -pi, pi ]

        if current_heading is None:
            # For the very first segment, if initial_heading is not provided,
            # we assume the robot is already aligned with the path and just
            # set our internal heading without turning.
            current_heading = desired_heading
        else:
            # Compute smallest angular difference
            dtheta = _normalize_angle(desired_heading - current_heading)

            # If the required turn is big enough, generate a turn segment
            if abs(dtheta) > min_turn_angle:
                turn_duration = abs(dtheta) / omega_turn

                if dtheta > 0:
                    # Positive angle => turn left ('a')
                    segments.append(Segment(direction='a', duration=turn_duration))
                else:
                    # Negative angle => turn right ('d')
                    segments.append(Segment(direction='d', duration=turn_duration))

                # Update heading
                current_heading = desired_heading

        # Now generate forward motion for this step
        dist = math.hypot(dx, dz)

        if dist > min_forward_dist:
            t_forward = dist / v_forward
            segments.append(Segment(direction='w', duration=t_forward))
        # If dist is very small, we skip adding a forward segment

    return segments


# Optional: quick test when run directly
if __name__ == "__main__":
    # Simple 2-step path: (0,0) -> (1,0) -> (1,1)
    test_nodes = [
        Node(id=0, x=0.0, z=0.0),
        Node(id=1, x=1.0, z=0.0),
        Node(id=2, x=1.0, z=1.0),
    ]
    test_path = [0, 1, 2]

    segs = path_to_segments(
        nodes=test_nodes,
        path=test_path,
        v_forward=0.2,         # 0.2 m/s
        omega_turn=math.pi,    # 180 deg/s
        initial_heading=0.0    # facing +x initially
    )

    print("Segments:")
    for s in segs:
        print(s)
