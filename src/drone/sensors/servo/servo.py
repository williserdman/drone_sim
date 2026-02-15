from gpiozero import Servo
from time import sleep

# Connect the signal wire to GPIO pin 17
servo = Servo(17)

while True:
    servo.min()  # Move to -1 (minimum)
    sleep(1)
    servo.mid()  # Move to 0 (middle)
    sleep(1)
    servo.max()  # Move to 1 (maximum)
    sleep(1)
