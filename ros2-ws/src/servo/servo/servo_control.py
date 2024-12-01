# Tutorial from: https://www.digikey.com/en/maker/tutorials/2021/how-to-control-servo-motors-with-a-raspberry-pi

from gpiozero import Servo
from time import sleep

SERVO_PIN = 14

servo = Servo(SERVO_PIN)

try:
    while True:
        print("Minimum angle")
        servo.min()
        sleep(0.5) # wait half a second

        print("Middle angle")
        servo.mid()
        sleep(0.5)

        print("Maximum angle")
        servo.max()
        sleep(0.5)
except KeyboardInterrupt:
    print("Program stopped")