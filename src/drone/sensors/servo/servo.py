from gpiozero import Servo
import time

<<<<<<< HEAD
# Initialize servo on GPIO 17, with specific pulse widths if needed
# Common SG90 servos use default angles, but might need range adjustment
servo = AngularServo(18, min_angle=0, max_angle=180)
=======

class Dropper:
    def __init__(self, pins=[1, 7, 8, 25]):
        self.servos = [
            Servo(i, min_pulse_width=0.0005, max_pulse_width=0.0025) for i in pins
        ]

    def drop(self, delay_hold=3):
        for s in self.servos:
            s.max()
        time.sleep(delay_hold)
        for s in self.servos:
            s.mid()


""" 
# Connect the servo signal wire to GPIO 17
s1 = Servo(7, min_pulse_width=0.0005, max_pulse_width=0.0025)
s2 = Servo(1, min_pulse_width=0.0005, max_pulse_width=0.0025)
s3 = Servo(25, min_pulse_width=0.0005, max_pulse_width=0.0025)
s4 = Servo(8, min_pulse_width=0.0005, max_pulse_width=0.0025)
>>>>>>> 611a79d (dropper class)

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
    print("Program stopped") """

if __name__ == "__main__":
    dropper = Dropper()
    time.sleep(5)
    dropper.drop()
