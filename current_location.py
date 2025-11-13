import pyzed.sl as sl
import os
import time

AREA_FILE = "my_room.area"

# -------------------------------
# 1. Init Camera
# -------------------------------
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 30
init_params.depth_mode = sl.DEPTH_MODE.NEURAL
init_params.coordinate_units = sl.UNIT.METER

err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("❌ Failed to open ZED:", err)
    exit(1)

print("📷 ZED camera opened.")

# ZED must grab at least 1 frame before tracking
zed.grab()

# -------------------------------
# 2. Enable positional tracking
# -------------------------------
tracking_params = sl.PositionalTrackingParameters()

if not os.path.exists(AREA_FILE):
    print(f"❌ Area file not found: {AREA_FILE}")
    zed.close()
    exit(1)

tracking_params.area_file_path = AREA_FILE
tracking_params.enable_area_memory = True
tracking_params.enable_imu_fusion = True

err = zed.enable_positional_tracking(tracking_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("❌ Failed to enable positional tracking:", err)
    zed.close()
    exit(1)

print(f"📌 Area file loaded: {AREA_FILE}")

# -------------------------------
# 3. Test pose retrieval
# -------------------------------
runtime = sl.RuntimeParameters()

print("📍 Moving camera to test localization...")
for i in range(100):
    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        pose = sl.Pose()
        zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)

        if pose.pose_confidence > 30:
            print("✔ Localized at:",
                  pose.get_translation().get(),
                  "Confidence:", pose.pose_confidence)
            break

        time.sleep(0.01)

# -------------------------------
# 4. Cleanup
# -------------------------------
zed.disable_positional_tracking()
zed.close()
print("✅ Test completed.")
