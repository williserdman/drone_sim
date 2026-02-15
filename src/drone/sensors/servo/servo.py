from gpiozero import AngularServo
from time import sleep

# Initialize servo on GPIO 17, with specific pulse widths if needed
# Common SG90 servos use default angles, but might need range adjustment
servo = AngularServo(17, min_angle=0, max_angle=180)

try:
    while True:
        servo.angle = 0
        sleep(1)
        servo.angle = 90
        sleep(1)
        servo.angle = 180
        sleep(1)
except KeyboardInterrupt:
    print("Program stopped")
