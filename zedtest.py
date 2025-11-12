import pyzed.sl as sl
import numpy as np
import matplotlib.pyplot as plt
import csv
import time

# ---------------------------
# 1. Initialize ZED camera
# ---------------------------
zed = sl.Camera()

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720          # 1280x720 @ 30 FPS
init_params.depth_mode = sl.DEPTH_MODE.NEURAL                 # Best accuracy mode
init_params.coordinate_units = sl.UNIT.METER                  # Use meters
init_params.camera_fps = 30

print("Opening ZED camera...")
if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("❌ Failed to open ZED camera.")
    exit(1)
print("✅ Camera initialized successfully.\n")

# ---------------------------
# 2. Enable positional tracking (SLAM)
# ---------------------------
tracking_params = sl.PositionalTrackingParameters()
tracking_params.set_floor_as_origin = True                    # Optional
err = zed.enable_positional_tracking(tracking_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("❌ Positional tracking failed:", err)
    zed.close()
    exit(1)
print("✅ Positional tracking enabled.\n")

# ---------------------------
# 3. Create runtime and pose objects
# ---------------------------
runtime = sl.RuntimeParameters()
pose = sl.Pose()

# For trajectory plotting
positions = []
timestamps = []

# ---------------------------
# 4. Start loop
# ---------------------------
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlabel("X (m)")
ax.set_ylabel("Z (m)")
ax.set_title("ZED SLAM Trajectory")

print("🛰️  Starting SLAM tracking... Move the camera around.\nPress Ctrl+C to stop.\n")

try:
    start_time = time.time()
    while True:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)
            translation = pose.get_translation().get()  # [x, y, z]
            rotation = np.array(pose.get_rotation_matrix().r)  # 3x3 rotation matrix

            # Store data
            positions.append(translation)
            timestamps.append(time.time() - start_time)

            # Update plot every 10 frames
            if len(positions) % 10 == 0:
                pts = np.array(positions)
                ax.clear()
                ax.plot(pts[:, 0], pts[:, 2], 'b-', linewidth=1.5)
                ax.set_xlabel("X (m)")
                ax.set_ylabel("Z (m)")
                ax.set_title("ZED SLAM Trajectory")
                plt.pause(0.001)

except KeyboardInterrupt:
    print("\n🛑 Stopping tracking...")

# ---------------------------
# 5. Save trajectory to CSV
# ---------------------------
output_file = "trajectory.csv"
with open(output_file, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time_s", "x_m", "y_m", "z_m"])
    for t, pos in zip(timestamps, positions):
        writer.writerow([t, pos[0], pos[1], pos[2]])

print(f"✅ Trajectory saved to {output_file}")

# ---------------------------
# 6. Cleanup
# ---------------------------
zed.disable_positional_tracking()
zed.close()
plt.ioff()
plt.show()
print("✅ Done. Camera closed cleanly.")
