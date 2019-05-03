# IMPORTANT: run `pigpiod` from the command line before running this script
import pigpio
from time import sleep
import signal

SPR = 800  # Steps per Revolution (360 / 0.45)

DIR = 2    # Direction GPIO Pin
STEP = 18  # Step GPIO Pin
# PWM 0: GPIO 12 or 18
# PWM 1: GPIO 13 or 19

CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
DUTY_CYCLE = 500000 # 50%

pi = pigpio.pi()

def signal_handler(sig, frame):
    pi.hardware_PWM(STEP, 0, 0)
signal.signal(signal.SIGINT, signal_handler)

pi.set_mode(DIR, pigpio.OUTPUT)
pi.set_mode(STEP, pigpio.OUTPUT)
pi.write(DIR, CW)

Hz = SPR/10 # 1/10 of a revolution per second
pi.hardware_PWM(STEP, Hz, DUTY_CYCLE)

sleep(100000000000)

signal_handler(None, None)