from flask import Flask, Response, request, jsonify
import cv2
import requests
import threading
import time

app = Flask(__name__)

# -----------------------------
# Raspberry Pi endpoints
# -----------------------------
PI_MOVE_URL = "http://192.168.1.12:5000/move"
PI_ENV_URL  = "http://192.168.1.12:5001/env"    # TEMP & HUM endpoint


# -----------------------------
# Webcam setup
# -----------------------------
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ ERROR: Could not open webcam")
else:
    print("📷 Webcam initialized")


# -----------------------------
# Global storage for TEMP & HUM
# -----------------------------
latest_env = {
    "module": None,
    "fw": None,
    "temperature": None,
    "humidity": None,
    "last_update": None,
    "error": None
}


# ======================================================
#         BACKGROUND THREAD: POLL /env FROM RPI
# ======================================================
def poll_env():
    global latest_env

    while True:
        try:
            res = requests.get(PI_ENV_URL, timeout=2)
            data = res.json()
            latest_env = data
            print(f"🌡️ Temp: {data['temperature']}°C | 💧 Hum: {data['humidity']}%")
        except Exception as e:
            print("⚠️ Env polling error:", e)

        time.sleep(1)  # update every second


# Start environment polling thread
threading.Thread(target=poll_env, daemon=True).start()


# ======================================================
#               VIDEO STREAM ENDPOINT
# ======================================================
def generate_frames():
    while True:
        success, frame = cap.read()
        if not success:
            print("⚠️ Frame grab failed")
            break

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            print("⚠️ JPEG encode failed")
            continue

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            buffer.tobytes() +
            b'\r\n'
        )


@app.route("/video")
def video():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# ======================================================
#       COMMAND RELAY TO RASPBERRY PI (/cmd)
# ======================================================
@app.route("/cmd", methods=["POST"])
def cmd():
    try:
        # Read JSON from Android
        data = request.get_json(force=True)
        print("📲 Received from Android:", data)

        # Forward command to Pi
        pi_response = requests.post(PI_MOVE_URL, json=data, timeout=3)

        print("🤖 Pi responded:", pi_response.text)

        return jsonify({
            "status": "ok",
            "android_received": data,
            "pi_response": pi_response.json(),
            "env": latest_env       # include temp + humidity here
        })

    except Exception as e:
        print("❌ Error:", e)
        return jsonify({"status": "error", "message": str(e)}), 400


# ======================================================
#          OPTIONAL ENDPOINT FOR PC TO SHOW ENV
# ======================================================
@app.route("/env")
def env():
    """Frontend/Android can query PC server directly."""
    return jsonify(latest_env)


# ======================================================
#                   MAIN
# ======================================================
if __name__ == "__main__":
    print("🚀 PC Streaming Server running on port 5000")
    print("📡 Polling temperature & humidity from Raspberry Pi...")
    app.run(host="0.0.0.0", port=5000, threaded=True)
