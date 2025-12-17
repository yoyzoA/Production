# navigation/config.py
#
# Central place for all configurable constants used by the navigation package.
# Edit these to match your network, file locations, and rover behavior.

from __future__ import annotations

import os
import math

# ------------------ Network (Laptop -> Raspberry Pi) ------------------ #

# IP / hostname of the Raspberry Pi.
# Change this to your Pi's address (e.g.
# PI_HOST: str = "192.168.1.42"
PI_HOST = "127.0.0.1"
PI_PORT = 5000

# HTTP port on which the Pi will listen for navigation commands.
# PI_PORT: int = 5000


# ------------------------- Data file locations ------------------------ #

# Base directory for all navigation-related files on the laptop.
# You can change this or make it an absolute path if you prefer.
BASE_DATA_DIR: str = os.path.join(os.path.dirname(__file__), "data")

# Make sure the directory exists (safe to call every time).
os.makedirs(BASE_DATA_DIR, exist_ok=True)

# Where TRAINING will save the ZED trajectory (poses over time).
TRAJECTORY_CSV_PATH: str = os.path.join(BASE_DATA_DIR, "trajectory.csv")

# Where TRAINING will save the ZED area memory.
AREA_FILE_PATH: str = os.path.join(BASE_DATA_DIR, "area_memory.area")

# Where your map-building script will save the navigation map (nodes, edges, ro...
MAP_FILE_PATH: str = os.path.join(BASE_DATA_DIR, "map.json")


# ---------------------- Motion / control parameters ------------------- #

# Training speed (manual WASD) - used conceptually on the Pi side.
# Units: meters per second.
V_TRAIN: float = 0.15    # you can tune this after testing

# Navigation forward speed - used in segments.path_to_segments()
# Units: meters per second.
V_NAV: float = 0.20    # tune based on rover stability and space

# Navigation turn rate - used in segments.path_to_segments()
# Units: radians per second.
# Example below = 45 deg/s.
OMEGA_NAV: float = math.radians(45.0)

# How close (in meters) we consider "goal reached" in NAVIGATE.
GOAL_RADIUS: float = 0.25    # 25 cm radius around the goal node
# Optional: how long (in seconds) without a TRAIN command before the Pi
# should treat it as "no command" and stop the rover (used on Pi side logic).
TRAIN_TIMEOUT: float = 0.3


# --------------------- Misc / debugging options ----------------------- #

# If True, extra debug prints can be enabled in various modules.
DEBUG: bool = True
