from gpiozero import Servo
import time


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


if __name__ == "__main__":
    dropper = Dropper()
    time.sleep(5)
    dropper.drop()
