import cv2
import time
import subprocess
from flask import Flask, Response

app = Flask(__name__)
camera = cv2.VideoCapture(0)  # default webcam

# 2. Give the camera hardware a tiny pause to wake up before blasting it with commands
time.sleep(1)

def setup_camera(device="/dev/video0"):
    """
    Configures V4L2 hardware settings for the Logitech C920.
    Must be run before or immediately after initializing cv2.VideoCapture.
    """
    print(f"=== Configuring Camera on {device} ===")

    # ---------------------------------------------------------
    # STEP 1: KILL THE AUTO LOOPS
    # ---------------------------------------------------------
    auto_controls = {
        "auto_exposure": "1",                 # 1 = Manual Mode
        "white_balance_automatic": "0",       # 0 = Off
        "focus_automatic_continuous": "0",    # 0 = Off
        "exposure_dynamic_framerate": "0",     # 0 = Off
        "power_line_frequency": "0",
    }

    print("Disabling automatic controls...")
    for control, value in auto_controls.items():
        try:
            subprocess.run(
                ['v4l2-ctl', '-d', device, f'--set-ctrl={control}={value}'],
                check=True, stderr=subprocess.DEVNULL
            )
        except subprocess.CalledProcessError:
            print(f"  [Skip] {control} (might already be off or unsupported)")

    # Give the hardware a tiny fraction of a second to register the mode changes
    time.sleep(0.2)

    # ---------------------------------------------------------
    # STEP 2: SET ABSOLUTE VALUES
    # ---------------------------------------------------------
    manual_controls = {
        # "exposure_time_absolute": "312",
        "white_balance_temperature": "4000",
        "focus_absolute": "0",
        "sharpness": "128",
        "contrast": "32",
        "saturation": "128",
        "brightness": "0",
        "zoom_absolute": "100",
        "backlight_compensation": "0",
        "gain": "0"  # Placed last to overwrite any lingering Auto Gain Control
    }

    print("Applying manual hardware settings...")
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-p", "30"])
    for control, value in manual_controls.items():
        try:
            subprocess.run(
                ['v4l2-ctl', '-d', device, f'--set-ctrl={control}={value}'],
                check=True, stderr=subprocess.DEVNULL
            )
        except subprocess.CalledProcessError:
            print(f"  [Failed] Could not set {control}={value}")

    print("=== Setup Complete! ===\n")

time.sleep(2)
# setup_camera("/dev/video0")

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode(".jpg", frame)
            frame = buffer.tobytes()
            yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")


@app.route("/video")
def video_feed():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=4321)  # i think i have wifi AP setup on 8080
