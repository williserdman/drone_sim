from gpiozero import Servo
import time

# Connect the servo signal wire to GPIO 17
s1 = Servo(7, min_pulse_width=0.0005, max_pulse_width=0.0025)
s2 = Servo(1, min_pulse_width=0.0005, max_pulse_width=0.0025)
s3 = Servo(25, min_pulse_width=0.0005, max_pulse_width=0.0025)
s4 = Servo(8, min_pulse_width=0.0005, max_pulse_width=0.0025)

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
