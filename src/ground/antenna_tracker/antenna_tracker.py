# IMPORTANT: run `pigpiod` from the command line before running this script
pigpio = None
try:
    import pigpio
except:
    print("Dry run")
import time
import signal
import math

import stepper_utils

# GPIO Pins
SERVO = 23
STEPPER_DIR = 2
STEPPER_STEP = 18
STEPPER_SLEEP = 14
# PWM 0: GPIO 12 or 18
# PWM 1: GPIO 13 or 19

STEPPER_SPR = 800  # Steps per Revolution (360 / 0.45)

SERVO_OFF = 0

def servo_pulsewidth(fraction): # 0 = retracted, 1 = extended
    return 1000 + 1000*fraction

pi = None
if pigpio:
    pi = pigpio.pi()

if pigpio:
    pi.set_mode(SERVO, pigpio.OUTPUT)
    pi.set_mode(STEPPER_DIR, pigpio.OUTPUT)
    pi.set_mode(STEPPER_STEP, pigpio.OUTPUT)
    pi.set_mode(STEPPER_SLEEP, pigpio.OUTPUT)

def on_shutdown(sig, frame):
    if pigpio:
        pi.wave_tx_stop()
        pi.wave_clear()
        pi.write(STEPPER_SLEEP, 0)
        pi.set_servo_pulsewidth(SERVO, SERVO_OFF)
        pi.stop()
signal.signal(signal.SIGINT, on_shutdown)


FEET_PER_METER = 3.28084
FEET_PER_DEGREE_LAT = FEET_PER_METER * 10**7 / 90 # The French originally defined the meter so that 10^7 meters would be the distance along the Paris meridian from the equator to the north pole.
FEET_PER_DEGREE_LNG = None
antenna_lat = None
antenna_lng = None
position_configured = False
def on_configure_pos(antenna_pos):
    antenna_lat = antenna_pos['lat']
    antenna_lng = antenna_pos['lng']
    FEET_PER_DEGREE_LNG = FEET_PER_DEGREE_LAT * math.cos(math.radians(antenna_lat))
    if pigpio:
        pi.write(STEPPER_SLEEP, 1) # wake up
    position_configured = True

stepper_pos = 0 # East (must be calibrated on startup)
def track(drone_pos):
    if not position_configured:
        return

    drone_lat = drone_pos['lat']
    drone_lng = drone_pos['lng']
    drone_alt = drone_pos['alt'] * FEET_PER_METER

    x_dist = (drone_lng - antenna_lng) * FEET_PER_DEGREE_LNG
    y_dist = (drone_lat - antenna_lat) * FEET_PER_DEGREE_LAT
    if (not pigpio) or (not pi.wave_tx_busy()):
        new_stepper_pos = (math.degrees(math.atan2(y_dist, x_dist)) * STEPPER_SPR) // 360
        print("stepper: {}", new_stepper_pos)
        while (new_stepper_pos - stepper_pos > SPR/2):
            new_stepper_pos -= SPR
        while (new_stepper_pos - stepper_pos < -SPR/2):
            new_stepper_pos += SPR
        if pigpio:
            stepper_utils.move_stepper(pi, STEPPER_SPR, new_stepper_pos - stepper_pos, STEPPER_STEP, STEPPER_DIR)
        stepper_pos = new_stepper_pos
    
    ground_dist = math.sqrt(x_dist**2 + y_dist**2)
    pitch = math.degrees(atan2(drone_alt, ground_dist))
    servo_extension = 0 # 0 to 1
    servo_extension = pitch/90 # temporary. TODO actual calculation
    print("servo: {}", servo_extension)
    if pigpio:
        pi.set_servo_pulsewidth(SERVO, servo_pulsewidth(servo_extension))

on_shutdown(None, None)
