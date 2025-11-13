import pyzed.sl as sl
import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os, time

# ============================================================
# CONFIGURATION
# ============================================================
AREA_FILE = "my_room.area"       # Saved map file
UPDATE_INTERVAL = 10             # Update 3D plot every N frames

# ============================================================
# SETUP ZED CAMERA
# ============================================================
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.depth_mode = sl.DEPTH_MODE.NEURAL
init_params.coordinate_units = sl.UNIT.METER
init_params.camera_fps = 30

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("❌ Failed to open ZED camera.")
    exit(1)

runtime = sl.RuntimeParameters()
pose = sl.Pose()
image = sl.Mat()

positions = []   # Store trajectory points

# ============================================================
# HELPER FUNCTIONS
# ============================================================
def start_mapping():
    """Enable positional tracking + mapping."""
    tracking_params = sl.PositionalTrackingParameters()
    tracking_params.set_floor_as_origin = True
    err = zed.enable_positional_tracking(tracking_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("❌ Positional tracking failed:", err)
        return False

    mapping_params = sl.SpatialMappingParameters()
    zed.enable_spatial_mapping(mapping_params)
    
    print("🗺️  Mapping started. Move camera to explore the room.")
    return True


def stop_and_save_mapping():
    """Stop mapping and save to .area file."""
    print("🛑 Stopping mapping and saving map...")
    zed.disable_spatial_mapping()
    zed.save_area_map(AREA_FILE)
    print(f"✅ Map saved as '{AREA_FILE}'")
    zed.disable_positional_tracking()


def start_localization():
    """Load existing map and start localization (SDK 5.1 Python)."""
    if not os.path.exists(AREA_FILE):
        print(f"❌ No '{AREA_FILE}' found. Run mapping first.")
        return False

    tracking_params = sl.PositionalTrackingParameters()

    # Correct member for area file load
    tracking_params.area_file_path = AREA_FILE

    # Enable area memory (loop-closure / area reuse)
    tracking_params.enable_area_memory = True

    # Note: enable_relocalization is not available in Python binding.
    # We'll rely on enable_area_memory and area_file_path to handle localization.

    # IMU fusion (if IMU exists)
    tracking_params.enable_imu_fusion = True

    err = zed.enable_positional_tracking(tracking_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("❌ Failed to start localization:", err)
        return False

    print(f"📍 Localization started using '{AREA_FILE}'.")
    return True




def stop_localization():
    zed.disable_positional_tracking()


def update_plot(ax, pts):
    """Update 3D trajectory plot."""
    ax.clear()
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], 'b-', linewidth=1.5)
    ax.scatter(pts[-1, 0], pts[-1, 1], pts[-1, 2], c='r', s=40, label="Current Pos")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("ZED SLAM 3D Trajectory")
    ax.legend(loc="upper right")
    ax.view_init(elev=30, azim=45)
    plt.pause(0.001)


# ============================================================
# MAIN LOOP
# ============================================================
print("============================================================")
print("🧭 ZED Mapping + Localization + 3D Trajectory System")
print("------------------------------------------------------------")
print("[M] Start Mapping   → build map and save it as .area")
print("[L] Start Localization → load map and localize")
print("[Q] Quit")
print("============================================================\n")

mode = None
frame_count = 0

plt.ion()
fig = plt.figure(figsize=(6, 5))
ax = fig.add_subplot(111, projection="3d")

while True:
    key = cv2.waitKey(1) & 0xFF

    if key == ord('m'):
        if mode != "mapping":
            stop_localization()
            if start_mapping():
                mode = "mapping"
                positions.clear()

    elif key == ord('l'):
        if mode != "localization":
            if mode == "mapping":
                stop_and_save_mapping()
            if start_localization():
                mode = "localization"
                positions.clear()

    elif key == ord('q'):
        print("👋 Exiting...")
        break

    # Grab frame
    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        cv2.imshow("ZED Camera", image.get_data())

        if mode in ["mapping", "localization"]:
            zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)
            translation = pose.get_translation().get()
            positions.append(translation)

            # Print position every 15 frames
            if frame_count % 15 == 0:
                print(f"[{mode.upper()}] x={translation[0]:.2f}, y={translation[1]:.2f}, z={translation[2]:.2f}")

            # Update 3D plot every N frames
            if len(positions) > 1 and frame_count % UPDATE_INTERVAL == 0:
                pts = np.array(positions)
                update_plot(ax, pts)

        frame_count += 1

# ============================================================
# CLEANUP
# ============================================================
if mode == "mapping":
    stop_and_save_mapping()
elif mode == "localization":
    stop_localization()

zed.close()
cv2.destroyAllWindows()
plt.ioff()
plt.show()
print("✅ ZED closed cleanly.")
