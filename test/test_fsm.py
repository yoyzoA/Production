# test/test_fsm.py

import os
import sys

import pytest

# Ensure project root ("fyp") is on sys.path so 'mira' is importable
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from mira.fsm.fsm import State, Events, next_state


# ----------------------------------------------------------------------
# BASIC ONE-STEP TRANSITION TESTS (existing)
# ----------------------------------------------------------------------

# ---------- IDLE state ----------
def test_idle_to_training_on_app_cmd_train():
    ev = Events(app_cmd_train=True)
    assert next_state(State.IDLE, ev) == State.TRAINING


def test_idle_to_navigate_requires_has_map():
    # app_cmd_nav but no map => stay IDLE
    ev = Events(app_cmd_nav=True, has_map=False)
    assert next_state(State.IDLE, ev) == State.IDLE

    # app_cmd_nav with map => NAVIGATE
    ev = Events(app_cmd_nav=True, has_map=True)
    assert next_state(State.IDLE, ev) == State.NAVIGATE


def test_idle_to_docking_on_battery_low_or_app_cmd_dock():
    # Needs has_map AND (battery_low OR app_cmd_dock)
    ev = Events(has_map=True, battery_low=True)
    assert next_state(State.IDLE, ev) == State.DOCKING

    ev = Events(has_map=True, app_cmd_dock=True)
    assert next_state(State.IDLE, ev) == State.DOCKING

    # If no map, even with low battery, stay IDLE
    ev = Events(has_map=False, battery_low=True)
    assert next_state(State.IDLE, ev) == State.IDLE


def test_idle_stays_idle_when_no_events():
    ev = Events()
    assert next_state(State.IDLE, ev) == State.IDLE


# ---------- TRAINING state ----------
def test_training_to_idle_on_app_cmd_idle_or_training_done_or_battery_low():
    # app_cmd_idle
    ev = Events(app_cmd_idle=True)
    assert next_state(State.TRAINING, ev) == State.IDLE

    # training_done
    ev = Events(training_done=True)
    assert next_state(State.TRAINING, ev) == State.IDLE

    # battery_low
    ev = Events(battery_low=True)
    assert next_state(State.TRAINING, ev) == State.IDLE


def test_training_stays_training_when_no_exit_event():
    ev = Events()    # all False
    assert next_state(State.TRAINING, ev) == State.TRAINING


# ---------- NAVIGATE state ----------
def test_navigate_to_idle_on_app_cmd_idle_or_goal_reached():
    # app_cmd_idle
    ev = Events(app_cmd_idle=True)
    assert next_state(State.NAVIGATE, ev) == State.IDLE

    # goal_reached
    ev = Events(goal_reached=True)
    assert next_state(State.NAVIGATE, ev) == State.IDLE


def test_navigate_to_docking_on_battery_low_or_app_cmd_dock_with_map():
    # battery_low + has_map
    ev = Events(has_map=True, battery_low=True)
    assert next_state(State.NAVIGATE, ev) == State.DOCKING

    # app_cmd_dock + has_map
    ev = Events(has_map=True, app_cmd_dock=True)
    assert next_state(State.NAVIGATE, ev) == State.DOCKING

    # If has_map is False, stay NAVIGATE even if battery_low/app_cmd_dock
    ev = Events(has_map=False, battery_low=True)
    assert next_state(State.NAVIGATE, ev) == State.NAVIGATE


def test_navigate_stays_navigate_when_no_exit_event():
    ev = Events()
    assert next_state(State.NAVIGATE, ev) == State.NAVIGATE


# ---------- DOCKING state ----------
def test_docking_to_idle_on_app_cmd_idle_or_dock_reached():
    # app_cmd_idle
    ev = Events(app_cmd_idle=True)
    assert next_state(State.DOCKING, ev) == State.IDLE

    # dock_reached
    ev = Events(dock_reached=True)
    assert next_state(State.DOCKING, ev) == State.IDLE


def test_docking_stays_docking_when_no_exit_event():
    ev = Events()
    assert next_state(State.DOCKING, ev) == State.DOCKING




def _run_scenario(initial_state, events_sequence):
    """
    Helper: apply a sequence of Events, returning the list of states visited.
    states[0] is the initial_state, then one entry after each event.
    """
    states = [initial_state]
    state = initial_state
    for ev in events_sequence:
        state = next_state(state, ev)
        states.append(state)
    return states


def test_full_train_nav_dock_cycle():
    """
    Scenario:
      IDLE
        -- app_cmd_train --> TRAINING
        -- training_done (and map available) --> IDLE
        -- app_cmd_nav + has_map --> NAVIGATE
        -- battery_low + has_map --> DOCKING
        -- dock_reached --> IDLE
    """
    events_seq = [
        Events(app_cmd_train=True),                       # IDLE -> TRAINING
        Events(training_done=True, has_map=True),         # TRAINING -> IDLE
        Events(app_cmd_nav=True, has_map=True),           # IDLE -> NAVIGATE
        Events(battery_low=True, has_map=True),           # NAVIGATE -> DOCKING
        Events(dock_reached=True),                        # DOCKING -> IDLE
    ]

    states = _run_scenario(State.IDLE, events_seq)

    assert states == [
        State.IDLE,        # initial
        State.TRAINING,    # after app_cmd_train
        State.IDLE,        # after training_done
        State.NAVIGATE,    # after app_cmd_nav + has_map
        State.DOCKING,     # after battery_low + has_map
        State.IDLE,        # after dock_reached
    ]


def test_battery_low_without_map_then_map_available():
    """
    Scenario:
      - IDLE + battery_low but no map: should stay IDLE.
      - Later, same event but has_map=True: now should go to DOCKING.
    """
    events_seq = [
        Events(battery_low=True, has_map=False),    # should NOT leave IDLE
        Events(battery_low=True, has_map=True),     # now should go to DOCKING
    ]

    states = _run_scenario(State.IDLE, events_seq)

    assert states == [
        State.IDLE,       # initial
        State.IDLE,       # battery_low but no map
        State.DOCKING,    # battery_low + has_map
    ]


def test_nav_command_ignored_when_no_map_then_works_with_map():
    """
    Scenario:
      - From IDLE, app_cmd_nav with has_map=False -> stay IDLE.
      - Then app_cmd_nav with has_map=True -> go to NAVIGATE.
    """
    events_seq = [
        Events(app_cmd_nav=True, has_map=False),
        Events(app_cmd_nav=True, has_map=True),
    ]

    states = _run_scenario(State.IDLE, events_seq)

    assert states == [
        State.IDLE,        # initial
        State.IDLE,        # nav cmd ignored (no map)
        State.NAVIGATE,    # nav cmd accepted (has map)
    ]


def test_training_to_docking_if_battery_low_mid_training():
    """
    Scenario:
      - Start in TRAINING.
      - battery_low event arrives => should go straight to IDLE (per one-step rules).
      - Then from IDLE, a docking command with map available moves to DOCKING.
    """
    events_seq = [
        Events(battery_low=True),                      # TRAINING -> IDLE
        Events(app_cmd_dock=True, has_map=True),       # IDLE -> DOCKING
    ]

    states = _run_scenario(State.TRAINING, events_seq)

    assert states == [
        State.TRAINING,    # initial
        State.IDLE,        # after battery_low
        State.DOCKING,     # after app_cmd_dock + has_map
    ]
