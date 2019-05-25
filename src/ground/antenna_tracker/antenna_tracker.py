# IMPORTANT: run `pigpiod` from the command line before running this script
pigpio = None
try:
    import pigpio
except:
    print("Dry run")
import time
import signal
import math

from stepper_utils import StepperUtils

# GPIO Pins
LED = 15
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

stepper_utils = StepperUtils(pigpio, pi, STEPPER_SPR, STEPPER_STEP, STEPPER_DIR)

if pigpio:
    pi.set_mode(LED, pigpio.OUTPUT)
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
        pi.write(LED, 0)
        pi.stop()
    exit()
signal.signal(signal.SIGINT, on_shutdown)


FEET_PER_METER = 3.28084
FEET_PER_DEGREE_LAT = FEET_PER_METER * 10**7 / 90 # The French originally defined the meter so that 10^7 meters would be the distance along the Paris meridian from the equator to the north pole.
FEET_PER_DEGREE_LNG = None
antenna_lat = None
antenna_lng = None
position_configured = False
def configure_pos(antenna_pos):
    global FEET_PER_DEGREE_LNG
    global antenna_lat
    global antenna_lng
    global position_configured
    antenna_lat = antenna_pos['lat']
    antenna_lng = antenna_pos['lng']
    FEET_PER_DEGREE_LNG = FEET_PER_DEGREE_LAT * math.cos(math.radians(antenna_lat))
    if pigpio:
        pi.write(STEPPER_SLEEP, 1) # wake up
    position_configured = True

MAX_REVOLUTIONS = 2 # Don't tangle the wires!
stepper_pos = 0 # East (must be calibrated on startup)
led_state = False
def track(drone_pos):
    global stepper_pos
    global led_state
    if not position_configured:
        return

    if pigpio: # flash LED so we know data is received
        pi.write(LED, led_state)
        led_state = not led_state

    drone_lat = drone_pos['latitude']
    drone_lng = drone_pos['longitude']
    drone_alt = drone_pos['relative_altitude'] * FEET_PER_METER

    x_dist = (drone_lng - antenna_lng) * FEET_PER_DEGREE_LNG
    y_dist = (drone_lat - antenna_lat) * FEET_PER_DEGREE_LAT
    if (not pigpio) or (not pi.wave_tx_busy()):
        new_stepper_pos = int((math.degrees(math.atan2(y_dist, x_dist)) * STEPPER_SPR) / 360)
        while (new_stepper_pos - stepper_pos > STEPPER_SPR/2):
            new_stepper_pos -= STEPPER_SPR
        while (new_stepper_pos - stepper_pos < -STEPPER_SPR/2):
            new_stepper_pos += STEPPER_SPR
        while (new_stepper_pos > STEPPER_SPR * MAX_REVOLUTIONS):
            new_stepper_pos -= STEPPER_SPR
        while (new_stepper_pos < -STEPPER_SPR * MAX_REVOLUTIONS):
            new_stepper_pos += STEPPER_SPR
        print("stepper:", new_stepper_pos)
        if pigpio:
            stepper_utils.move_stepper(new_stepper_pos - stepper_pos)
        stepper_pos = new_stepper_pos
    
    ground_dist = math.sqrt(x_dist**2 + y_dist**2)
    pitch = math.degrees(math.atan2(drone_alt, ground_dist))
    servo_extension = 0 # 0 to 1
    servo_extension = pitch/90 # temporary. TODO actual calculation
    print("pitch:", pitch, "   servo:", servo_extension)
    if pigpio:
        pi.set_servo_pulsewidth(SERVO, servo_pulsewidth(servo_extension))


if __name__ == "__main__":
    import socketio
    import os

    server_ip = 'localhost'
    if os.uname().machine.startswith("arm"): # if on raspberry pi
        server_ip = '192.168.1.10' # static ip of ground station

    sio = socketio.Client()

    @sio.on('connect', namespace='/tracky')
    def on_connect():
        print("connected to ground server")
        if pigpio:
            pi.write(LED, 1)

    @sio.on('disconnect', namespace='/tracky')
    def on_disconnect():
        print("disconnected from ground server!")
        if pigpio:
            pi.write(LED, 0)

    @sio.on('CONFIGURE_POS', namespace='/tracky')
    def on_configure_pos(antenna_pos):
        configure_pos(antenna_pos)

    @sio.on('DRONE_POS', namespace='/tracky')
    def on_drone_pos(drone_pos):
        track(drone_pos)

    while True:
        try:
            sio.connect('http://'+server_ip+':8081', namespaces=['/tracky'], transports='websocket')
            break
        except:
            print("Can't connect to ground server. Retrying in 2 seconds...")
            sio.sleep(2)

    sio.wait()

    on_shutdown(None, None)
