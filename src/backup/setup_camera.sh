#!/bin/bash

# Change this if your camera switches USB ports
DEVICE="/dev/video0"

echo "=== Configuring Camera on $DEVICE ==="

# ---------------------------------------------------------
# STEP 1: KILL THE AUTO LOOPS
# These must be turned off FIRST, otherwise the camera 
# will reject the manual settings below.
# ---------------------------------------------------------
echo "Disabling automatic controls..."
v4l2-ctl -d $DEVICE --set-ctrl=auto_exposure=1                 # 1 = Manual Mode
v4l2-ctl -d $DEVICE --set-ctrl=white_balance_automatic=0       # 0 = Off
v4l2-ctl -d $DEVICE --set-ctrl=focus_automatic_continuous=0    # 0 = Off
v4l2-ctl -d $DEVICE --set-ctrl=exposure_dynamic_framerate=0    # 0 = Off

v4l2-ctl -p 30 

# Give the hardware a tiny fraction of a second to register the mode changes
sleep 0.2

# ---------------------------------------------------------
# STEP 2: SET ABSOLUTE VALUES
# ---------------------------------------------------------
echo "Applying manual hardware settings..."
v4l2-ctl -d $DEVICE --set-ctrl=exposure_time_absolute=20
v4l2-ctl -d $DEVICE --set-ctrl=white_balance_temperature=4000
v4l2-ctl -d $DEVICE --set-ctrl=focus_absolute=0
v4l2-ctl -d $DEVICE --set-ctrl=sharpness=128
v4l2-ctl -d $DEVICE --set-ctrl=brightness=255
v4l2-ctl -d $DEVICE --set-ctrl=power_line_frequency=0
v4l2-ctl -d $DEVICE --set-ctrl=contrast=128
v4l2-ctl -d $DEVICE --set-ctrl=saturation=128
v4l2-ctl -d $DEVICE --set-ctrl=zoom_absolute=100
v4l2-ctl -d $DEVICE --set-ctrl=backlight_compensation=0

# Set Gain last to ensure it overwrites any lingering Auto Gain Control
v4l2-ctl -d $DEVICE --set-ctrl=gain=0

echo "=== Setup Complete! ==="
echo ""

# ---------------------------------------------------------
# STEP 3: VERIFY
# Print out the critical settings so you can verify they stuck
# ---------------------------------------------------------
echo "Active Settings (Look for 'flags=inactive' to ensure auto is off):"
v4l2-ctl -d $DEVICE --list-ctrls | grep -E "exposure|white_balance|focus|gain|backlight"


