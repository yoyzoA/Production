# navigation/zed_interface.py
#
# Thin wrapper around the ZED camera (pyzed.sl) for:
# - TRAINING: log poses -> trajectory.csv, save .area
# - NAVIGATE: load .area, get current pose + heading
#
# This module is used on the LAPTOP side only.

from __future__ import annotations

import time
import math
import csv
import os
from typing import List, Tuple, Optional

import numpy as np
import pyzed.sl as sl


class ZedInterface:
    """
    Simple wrapper for ZED camera operations needed by this project.

    Main methods:

        open()
        close()

        enable_tracking(load_area: bool, area_path: Optional[str])
        disable_tracking()

        start_logging_trajectory(start_time: Optional[float] = None)
        log_pose_step()
        stop_and_save_trajectory(trajectory_path: str, area_path: Optional[str])

        get_pose_with_heading() -> (x, y, z, heading)

    Notes:
        - heading is a yaw angle [rad] in the (x,z) plane, derived from the
          rotation matrix. You may need to tune the exact formula based on
          your coordinate convention, but this gives a reasonable starting point.
    """

    def __init__(self) -> None:
        self.cam: Optional[sl.Camera] = None
        self.runtime: Optional[sl.RuntimeParameters] = None
        self.pose: Optional[sl.Pose] = None
        self.tracking_enabled: bool = False

        # For logging during TRAINING
        self._logging: bool = False
        self._log_start_time: float = 0.0
        self._trajectory: List[Tuple[float, float, float, float]] = []    # (t, x, y, z)

    # ------------------------------------------------------------------ #
    # Camera / tracking setup
    # ------------------------------------------------------------------ #

    def open(self) -> None:
        """
        Open the ZED camera with default parameters.
        """
        if self.cam is not None:
            # Already open
            return

        self.cam = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        init_params.camera_fps = 30

        print("[ZED] Opening camera...")
        err = self.cam.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.cam = None
            raise RuntimeError(f"[ZED] Failed to open camera: {err}")

        self.runtime = sl.RuntimeParameters()
        self.pose = sl.Pose()

        print("[ZED] Camera opened.")

    def close(self) -> None:
        """
        Close the ZED camera and disable tracking if needed.
        """
        if self.cam is None:
            return

        if self.tracking_enabled:
            self.disable_tracking()

        print("[ZED] Closing camera...")
        self.cam.close()
        self.cam = None
        self.runtime = None
        self.pose = None
        print("[ZED] Camera closed.")

    def enable_tracking(self, load_area: bool, area_path: Optional[str]) -> None:
        """
        Enable positional tracking (SLAM).

        Args:
            load_area: if True, tries to load an existing .area file for relocalization.
            area_path: path to the .area file to load (if load_area is True).
        """
        if self.cam is None:
            raise RuntimeError("[ZED] Camera not opened before enable_tracking().")

        if self.tracking_enabled:
            # Already enabled
            return

        tracking_params = sl.PositionalTrackingParameters()

        if load_area:
            if area_path is None or not os.path.isfile(area_path):
                raise FileNotFoundError(f"[ZED] Area file not found: {area_path}")
            # Tell ZED to load existing area memory
            tracking_params.area_file_path = area_path
            print(f"[ZED] Enabling positional tracking with area file: {area_path}")
        else:
            print("[ZED] Enabling positional tracking without area file (fresh).")

        err = self.cam.enable_positional_tracking(tracking_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"[ZED] Failed to enable positional tracking: {err}")

        self.tracking_enabled = True
        print("[ZED] Positional tracking enabled.")

    def disable_tracking(self) -> None:
        """
        Disable positional tracking if it's enabled.
        """
        if self.cam is None or not self.tracking_enabled:
            return

        print("[ZED] Disabling positional tracking...")
        self.cam.disable_positional_tracking()
        self.tracking_enabled = False
        print("[ZED] Positional tracking disabled.")

    # ------------------------------------------------------------------ #
    # Logging trajectory during TRAINING
    # ------------------------------------------------------------------ #

    def start_logging_trajectory(self, start_time: Optional[float] = None) -> None:
        """
        Start recording trajectory for TRAINING.

        Args:
            start_time: reference time in seconds (e.g. time.time()).
                        If None, the current time is used.
        """
        if not self.tracking_enabled:
            raise RuntimeError("[ZED] Cannot start logging; tracking is not enabled.")

        if start_time is None:
            start_time = time.time()

        self._logging = True
        self._log_start_time = start_time
        self._trajectory = []
        print("[ZED] Trajectory logging started.")

    def log_pose_step(self) -> None:
        """
        Grab one frame, get pose, and append to trajectory list if logging is on.

        This is meant to be called in a loop during TRAINING.
        """
        if not self._logging or self.cam is None or self.runtime is None or self.pose is None:
            return

        if self.cam.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            # No new frame; skip this iteration
            return

        self.cam.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
        translation = self.pose.get_translation().get()    # [x, y, z]
        t_now = time.time()
        t_rel = t_now - self._log_start_time

        self._trajectory.append((t_rel, translation[0], translation[1], translation[2]))

    def stop_and_save_trajectory(self, trajectory_path: str, area_path: Optional[str]) -> None:
        """
        Stop logging and save trajectory and area file.

        Args:
            trajectory_path: path to save CSV file with columns [time_s, x_m, y_m, z_m].
            area_path: path to save the ZED area memory (.area). If None, not saved.
        """
        self._logging = False

        # Save trajectory
        if trajectory_path and self._trajectory:
            os.makedirs(os.path.dirname(trajectory_path), exist_ok=True)
            print(f"[ZED] Writing trajectory CSV to {trajectory_path}")
            with open(trajectory_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "x_m", "y_m", "z_m"])
                for (t, x, y, z) in self._trajectory:
                    writer.writerow([t, x, y, z])
        else:
            print("[ZED] No trajectory data to save or empty path.")

        # Save area map
        if area_path and self.cam is not None:
            os.makedirs(os.path.dirname(area_path), exist_ok=True)
            print(f"[ZED] Saving area map to {area_path}")
            err = self.cam.save_area_map(area_path)
            if err != sl.ERROR_CODE.SUCCESS:
                print(f"[ZED] Warning: failed to save area map: {err}")
            else:
                print("[ZED] Area map saved.")
        else:
            print("[ZED] No area file path provided; skipping area save.")

    # ------------------------------------------------------------------ #
    # NAVIGATE: get pose + heading
    # ------------------------------------------------------------------ #

    def get_pose_with_heading(self) -> Tuple[float, float, float, float]:
        """
        Get the current camera pose (x, y, z) and heading (yaw) in radians.

        Returns:
            (x_m, y_m, z_m, heading_rad)

        Notes:
            - heading is approximated from the rotation matrix's orientation
              in the (x,z) plane. You may need to adjust signs depending on
              your coordinate frame and what you consider "forward".
        """
        if self.cam is None or self.runtime is None or self.pose is None:
            raise RuntimeError("[ZED] Camera not initialized for get_pose_with_heading().")

        # Grab one frame
        if self.cam.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            # If grab fails, we can still return the last pose, or raise.
            # Here we raise to force caller to handle it.
            raise RuntimeError("[ZED] grab() failed in get_pose_with_heading().")

        self.cam.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
        translation = self.pose.get_translation().get()    # [x, y, z]
        x, y, z = translation[0], translation[1], translation[2]

        # Get rotation matrix and compute a yaw angle
        # pose.get_rotation_matrix() returns a sl.Rotation
        rot = self.pose.get_rotation_matrix()
        # 'r' is a flat array of 9 elements; reshape to 3x3
        R = np.array(rot.r).reshape((3, 3))

        # Approximate yaw (heading) around Y axis based on R.
        # Depending on the ZED coordinate frame, you may need to adjust this.
        # This formula assumes a standard convention where:
        # - x is right,
        # - y is up,
        # - z is forward,
        # and yaw is the rotation in the x-z plane.
        #
        # Here we use:
        # heading = atan2(R[2, 0], R[2, 2])
        #
        # You can experiment with different formulas if the direction
        # doesn't match what you expect.
        heading = math.atan2(R[2, 0], R[2, 2])

        return x, y, z, heading
