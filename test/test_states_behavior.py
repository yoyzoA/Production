# test/test_states_behavior.py
#
# Tests Pi-side state behaviors:
# - mira.states.idle.behavior
# - mira.states.training.behavior
# - mira.states.navigate.behavior
#
# We DO NOT use real RoverPID or GPIO here.
# - We fake the "pid" module so imports succeed.
# - We pass a FakeRoverPID object into the behavior functions and
# check the wheel targets / cleanup flags.

import os
import sys
import types

import pytest

# Ensure project root ("fyp") is on sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# ----------------------------------------------------------------------
# Fake "pid" module so from pid import RoverPID works on laptop
# ----------------------------------------------------------------------

fake_pid = types.ModuleType("pid")


class DummyRoverPID:
    """Placeholder type for annotations inside behavior modules."""
    pass


fake_pid.RoverPID = DummyRoverPID
# Only register if not already present, so we don't clash with real Pi env
sys.modules.setdefault("pid", fake_pid)

# Now we can safely import the behaviors
from mira.states.idle import behavior as idle_behavior
from mira.states.training import behavior as training_behavior
from mira.states.navigate import behavior as navigate_behavior


# ----------------------------------------------------------------------
# Test helper: FakeRoverPID that records commands
# ----------------------------------------------------------------------

class FakeRoverPID:
    def __init__(self):
        self.targets = []        # list[(v_left, v_right)]
        self.cleanup_called = False

    def set_target(self, v_left: float, v_right: float):
        self.targets.append((v_left, v_right))

    def cleanup(self):
        self.cleanup_called = True

    @property
    def last_target(self):
        return self.targets[-1] if self.targets else None


# ----------------------------------------------------------------------
# IDLE state behavior
# ----------------------------------------------------------------------

def test_idle_step_stops_motion():
    rover = FakeRoverPID()

    idle_behavior.idle_step(rover)

    assert rover.last_target == (0.0, 0.0)


# ----------------------------------------------------------------------
# TRAINING state behavior
# ----------------------------------------------------------------------

def test_start_training_resets_flag_and_stops():
    rover = FakeRoverPID()

    # Force flag to True, then start_training should reset it.
    training_behavior.set_training_done()
    assert training_behavior.is_training_done() is True

    training_behavior.start_training(rover)

    assert training_behavior.is_training_done() is False
    assert rover.last_target == (0.0, 0.0)    # stop_training_motion


@pytest.mark.parametrize(
    "key, expected",
    [
        ("w", "forward"),
        ("s", "backward"),
        ("a", "left"),
        ("d", "right"),
    ],
)
def test_training_step_moves_for_each_key(key, expected):
    rover = FakeRoverPID()

    # Reset training state before each test
    training_behavior.start_training(rover)
    rover.targets.clear()

    training_behavior.training_step(rover, key=key)

    # TRAINING_SPEED from module
    s = training_behavior.TRAINING_SPEED

    if expected == "forward":
        assert rover.last_target == (s, s)
    elif expected == "backward":
        assert rover.last_target == (-s, -s)
    elif expected == "left":
        assert rover.last_target == (-s, +s)
    elif expected == "right":
        assert rover.last_target == (+s, -s)


def test_training_step_finish_sets_done_and_stops():
    rover = FakeRoverPID()

    training_behavior.start_training(rover)
    rover.targets.clear()

    # 'f' should finish training
    training_behavior.training_step(rover, key="f")

    assert training_behavior.is_training_done() is True
    # training_step('f') calls stop_training_motion -> (0, 0)
    assert rover.last_target == (0.0, 0.0)


def test_training_step_invalid_key_does_not_change_motion():
    rover = FakeRoverPID()

    training_behavior.start_training(rover)
    rover.targets.clear()

    # First, move forward so we have a non-zero target
    training_behavior.training_step(rover, key="w")
    first_target = rover.last_target

    # Now send an invalid key
    training_behavior.training_step(rover, key="x")

    # For invalid key, behavior just prints a message and does NOT
    # call any motion function, so last_target stays the same.
    assert rover.last_target == first_target


# ----------------------------------------------------------------------
# NAVIGATE state behavior
# ----------------------------------------------------------------------

def test_start_navigate_stops_motion():
    rover = FakeRoverPID()

    navigate_behavior.start_navigate(rover)

    assert rover.last_target == (0.0, 0.0)


@pytest.mark.parametrize(
    "command, expected",
    [
        ("w", "forward"),
        ("s", "backward"),
        ("a", "left"),
        ("d", "right"),
    ],
)
def test_navigate_step_key_mapping(command, expected):
    rover = FakeRoverPID()

    navigate_behavior.start_navigate(rover)
    rover.targets.clear()

    navigate_behavior.navigate_step(rover, command)

    s = navigate_behavior.NAV_SPEED

    if expected == "forward":
        assert rover.last_target == (s, s)
    elif expected == "backward":
        assert rover.last_target == (-s, -s)
    elif expected == "left":
        assert rover.last_target == (-s, +s)
    elif expected == "right":
        assert rover.last_target == (+s, -s)


def test_navigate_step_invalid_stops_motion():
    rover = FakeRoverPID()

    navigate_behavior.start_navigate(rover)
    rover.targets.clear()

    # Move first so we know it changes
    navigate_behavior.navigate_step(rover, "w")
    assert rover.last_target is not None
    assert rover.last_target != (0.0, 0.0)

    # Now send invalid command -> stop_motion
    navigate_behavior.navigate_step(rover, "x")
    assert rover.last_target == (0.0, 0.0)


def test_cleanup_gpio_calls_rover_cleanup():
    rover = FakeRoverPID()
    assert rover.cleanup_called is False

    navigate_behavior.cleanup_gpio(rover)

    assert rover.cleanup_called is True
