# IMPORTANT: run `pigpiod` from the command line before running this script
import pigpio
from time import sleep

DIR = 2    # Direction GPIO Pin
STEP = 15  # Step GPIO Pin
CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
SPR = 800   # Steps per Revolution (360 / 0.45)

pi = pigpio.pi()

pi.set_mode(DIR, pigpio.OUTPUT)
pi.set_mode(STEP, pigpio.OUTPUT)
pi.write(DIR, CW)

step_count = SPR/4 # quarter turn
delay = .01

for x in range(step_count):
    pi.write(STEP, 1)
    sleep(delay)
    pi.write(STEP, 0)
    sleep(delay)

# sleep(.5)
# GPIO.output(DIR, CCW)
# for x in range(step_count):
#     GPIO.output(STEP, GPIO.HIGH)
#     sleep(delay)
#     GPIO.output(STEP, GPIO.LOW)
#     sleep(delay)

# GPIO.cleanup()
