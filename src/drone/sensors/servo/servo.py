from gpiozero import Servo
import time

# Initialize servo on GPIO 17, with specific pulse widths if needed
# Common SG90 servos use default angles, but might need range adjustment
servo = AngularServo(18, min_angle=0, max_angle=180)

try:
    while True:
        s1.max()
        s3.max()

        s2.max()
        s4.max()

        time.sleep(5)
        s1.mid()
        s2.mid()
        s3.mid()
        s4.mid()
        time.sleep(5)

except KeyboardInterrupt:
    print("Program stopped")
