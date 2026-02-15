import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
pwm = GPIO.PWM(17, 50)  # 50Hz
pwm.start(0)

# Move to 90 degrees
while True:
    pwm.ChangeDutyCycle(7.5)
    time.sleep(2)
    pwm.ChangeDutyCycle(0)
    time.sleep(2)
