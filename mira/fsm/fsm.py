from enum import Enum, auto
from dataclasses import dataclass

class State(Enum):
    # auto gives the obj the next available number
    IDLE = auto()
    TRAINING = auto()
    NAVIGATE = auto()
    DOCKING = auto()

@dataclass
class Events:
    has_map: bool = False
    app_cmd_train: bool = False
    app_cmd_nav: bool = False
    app_cmd_dock: bool = False
    app_cmd_idle: bool = False
    battery_low: bool = False
    training_done: bool = False
    goal_reached: bool = False
    dock_reached: bool = False

def next_state(current: State, ev: Events) -> State:
    if current == State.IDLE:
        if ev.app_cmd_train:
            return State.TRAINING
        if ev.app_cmd_nav and ev.has_map:
            return State.NAVIGATE
        if ev.has_map and (ev.battery_low or ev.app_cmd_dock):
            return State.DOCKING
        return State.IDLE

    if current == State.TRAINING:
        if ev.app_cmd_idle:
            return State.IDLE
        if ev.training_done:
            return State.IDLE
        if ev.battery_low:
            return State.IDLE
        return State.TRAINING

    if current == State.NAVIGATE:
        if ev.app_cmd_idle:
            return State.IDLE
        if ev.goal_reached:
            return State.IDLE
        if ev.has_map and (ev.battery_low or ev.app_cmd_dock):
            return State.DOCKING
        return State.NAVIGATE

    if current == State.DOCKING:
        if ev.app_cmd_idle:
            return State.IDLE
        if ev.dock_reached:
            return State.IDLE
        return State.DOCKING

    return current
