#!/usr/bin/env python3

# Run on RPi boot: https://www.raspberrypi.org/documentation/linux/usage/rc-local.md

# IMPORTANT: Enable I2C on the RPi using `sudo raspi-config` -> Interfacing Options
# See for LCD (Arduino) examples https://github.com/sparkfun/OpenLCD/tree/2829d059a7a7dc137ab48a64771079831a65c3bd/firmware/Examples
#     Note that that is an old version, because the firmware in our LCD is outdated. It is possible to update with an FTDI programmer:
#     https://learn.sparkfun.com/tutorials/avr-based-serial-enabled-lcds-hookup-guide/all#troubleshooting

import pigpio
import time
import subprocess
import colorsys
import sys
import socketio

process = subprocess.Popen(["killall", "pigpiod"])
process.wait()
process = subprocess.Popen(["pigpiod"])
process.wait()
time.sleep(2) # give some time for pigpiod to be completely ready
print("Finished Setup")

leftLED = 17
middleLED = 25
rightLED = 19

pi = pigpio.pi()

pi.set_mode(leftLED, pigpio.OUTPUT)
pi.set_mode(middleLED, pigpio.OUTPUT)
pi.set_mode(rightLED, pigpio.OUTPUT)

flashTime = 0.01

def leftON():
    pi.write(leftLED, pigpio.HIGH)

def leftOFF():
    pi.write(leftLED, pigpio.LOW)

def leftFLASH():
    pi.write(leftLED, pigpio.LOW)
    time.sleep(flashTime)
    pi.write(leftLED, pigpio.HIGH)
    time.sleep(flashTime)

def rightON():
    pi.write(rightLED, pigpio.HIGH)

def rightOFF():
    pi.write(rightLED, pigpio.LOW)

def rightFLASH():
    pi.write(rightLED, pigpio.LOW)
    time.sleep(flashTime)
    pi.write(rightLED, pigpio.HIGH)
    time.sleep(flashTime)

def middleON():
    pi.write(middleLED, pigpio.HIGH)

def middleOFF():
    pi.write(middleLED, pigpio.LOW)

green = 27 # AUTO LOWER
bottomRed = 16
topRed = 5 # STOP
bottomYellow = 22 # DOWN
topYellow = 18 # UP
rightWhite = 13
leftWhite = 6
topRightBlue = 4 # CUT
blueTwo = 24
blueThree = 20
blueFour = 12
blueFive = 23

autoLowerBtn = green
stopBtn = topRed
downBtn = bottomYellow
upBtn = topYellow
cutBtn = topRightBlue
cancelBtn = bottomRed
resetBtn = leftWhite
stopCutBtn = rightWhite

pi.set_mode(green, pigpio.INPUT)
pi.set_pull_up_down(green, pigpio.PUD_UP)
pi.set_mode(bottomRed, pigpio.INPUT)
pi.set_pull_up_down(bottomRed, pigpio.PUD_UP)
pi.set_mode(topRed, pigpio.INPUT)
pi.set_pull_up_down(topRed, pigpio.PUD_UP)
pi.set_mode(bottomYellow, pigpio.INPUT)
pi.set_pull_up_down(bottomYellow, pigpio.PUD_UP)
pi.set_mode(topYellow, pigpio.INPUT)
pi.set_pull_up_down(topYellow, pigpio.PUD_UP)
pi.set_mode(rightWhite, pigpio.INPUT)
pi.set_pull_up_down(rightWhite, pigpio.PUD_UP)
pi.set_mode(leftWhite, pigpio.INPUT)
pi.set_pull_up_down(leftWhite, pigpio.PUD_UP)
pi.set_mode(topRightBlue, pigpio.INPUT)
pi.set_pull_up_down(topRightBlue, pigpio.PUD_UP)
pi.set_mode(blueTwo, pigpio.INPUT)
pi.set_pull_up_down(blueTwo, pigpio.PUD_UP)
pi.set_mode(blueThree, pigpio.INPUT)
pi.set_pull_up_down(blueThree, pigpio.PUD_UP)
pi.set_mode(blueFour, pigpio.INPUT)
pi.set_pull_up_down(blueFour, pigpio.PUD_UP)
pi.set_mode(blueFive, pigpio.INPUT)
pi.set_pull_up_down(blueFive, pigpio.PUD_UP)

LCD_I2C_ADDRESS = 0x72
I2C_BUS = 0
h = pi.i2c_open(1, LCD_I2C_ADDRESS) # open device at address 0x72 on bus 0

def printLCD(msg):
    pi.i2c_write_byte(I2C_BUS, 0x7C) # Put LCD into setting mode
    pi.i2c_write_byte(I2C_BUS, 0x2D) # Send clear display command
    pi.i2c_write_device(h, msg.encode())

pi.i2c_write_byte(I2C_BUS, 0x7C) # Put LCD into setting mode
pi.i2c_write_byte(I2C_BUS, 0x18) # contrast command
pi.i2c_write_byte(I2C_BUS, 0) # max contrast
printLCD("UAS at UCLA")

def receiveDroneState(message):
    print(message)
    printLCD(message)

    H = 0.0
    S = 1
    V = 1

    if message == "DISARMED":
        H = 108.0 / 360.0
    elif message == "AUTO.LOITER":
        H = 60.0 / 360.0
    elif message == "AUTO.TAKEOFF":
        H = 190.0 / 360.0
    elif message == "AUTO.LAND":
        H = 255.0 / 360.0
    elif message == "OFFBOARD":
        H = 26.0 / 360.0
    elif message == "MANUAL" or message == "STABILIZED":
        H = 276.0 / 360.0
    elif message == "POSCTL":
        H = 224.0 / 360.0
    else:
        H = 0.0 / 360.0

    R,G,B = colorsys.hsv_to_rgb(H,S,V)

    pi.i2c_write_byte(I2C_BUS, 0x7C) # Put LCD into setting mode
    pi.i2c_write_byte(I2C_BUS, 128 + round(R*29)) #  white/red

    pi.i2c_write_byte(I2C_BUS, 0x7C) # Put LCD into setting mode
    pi.i2c_write_byte(I2C_BUS, 158 + round(G*29)) # green

    pi.i2c_write_byte(I2C_BUS, 0x7C) # Put LCD into setting mode
    pi.i2c_write_byte(I2C_BUS, 188 + round(B*29)) # blue
    

delay = 0.05

btnProtection = False

readyToDrop = False
dropping = False
cut = False

def setReadyToDrop(ready):
    global readyToDrop
    readyToDrop = ready

def setDropping():
    global dropping
    dropping = True

def setCut():
    global cut
    cut = True

def setNotDropping():
    global dropping
    dropping = False

server_ip = '192.168.1.10' # static ip of ground station
if (len(sys.argv) >= 2):
    server_ip = sys.argv[1]
sio = socketio.Client(logger=True, engineio_logger=True)

@sio.on('connect', namespace='/button-panel')
def on_connect():
    printLCD("Connected")

@sio.on('disconnect', namespace='/button-panel')
def on_disconnect():
    printLCD("Disconnected")

@sio.on('DRONE_STATE', namespace='/button-panel')
def on_drone_state(state):
    receiveDroneState(state)

@sio.on('DROPPY_READY', namespace='/button-panel')
def on_droppy_ready(ready):
    setReadyToDrop(ready)

@sio.on('DROPPY_COMMAND_RECEIVED', namespace='/button-panel')
def on_droppy_command_received(cmd):
    if cmd == "START_DROP":
        setDropping()
    elif cmd == "CUT_LINE":
        setCut()

print("Attempting to connect to " + 'http://'+server_ip+':8081')
while True:
    try:
        sio.connect('http://'+server_ip+':8081', namespaces=['/button-panel'], transports='websocket')
        break
    except:
        print("Can't connect to ground server. Retrying in 2 seconds...")
        sio.sleep(2)

def run():
    while True:
        if readyToDrop and dropping:
            rightFLASH()
        elif readyToDrop:
            rightON()
        else:
            rightOFF()

        t = time.time()
        while (not pi.read(autoLowerBtn)):  # 1 = button not pressed, 0 = button pressed
            if btnProtection and ((not readyToDrop) or (dropping)):
                break
            time.sleep(flashTime)
            if time.time() > (t + delay):
                print("BEGIN LOWERING THE UGV")
                sio.emit('CHANGE_DROPPY_STATE', 'START_DROP', namespace='/button-panel')
                middleON()
    
        while (not pi.read(stopBtn)):
            if btnProtection and not dropping:
                break
            time.sleep(flashTime)
            if time.time() > (t + delay):
                print("STOP LOWERING THE UGV")
                sio.emit('CHANGE_DROPPY_STATE', 'MOTOR_STOP', namespace='/button-panel')
                leftON()
    
        while (not pi.read(upBtn)):
            if btnProtection and not dropping:
                break
            time.sleep(flashTime)
            if time.time() > (t + delay):
                print("RAISE THE UGV")
                sio.emit('CHANGE_DROPPY_STATE', 'MOTOR_UP', namespace='/button-panel')
                leftON()

        while (not pi.read(downBtn)):
            if btnProtection and not dropping:
                break
            time.sleep(flashTime)
            if time.time() > (t + delay):
                print("LOWER THE UGV")
                sio.emit('CHANGE_DROPPY_STATE', 'MOTOR_DOWN', namespace='/button-panel')
                leftON()

        while (not pi.read(cutBtn)):
            time.sleep(flashTime)
            if btnProtection and ((not dropping) or (cut)):
                break
            if time.time() > (t + delay):
                print("CUT THE LINE")
                sio.emit('CHANGE_DROPPY_STATE', 'CUT_LINE', namespace='/button-panel')
                middleON()

        while (not pi.read(cancelBtn)):
            time.sleep(flashTime)
            if time.time() > (t + delay):
                print("CANCEL DROP")
                sio.emit('CHANGE_DROPPY_STATE', 'CANCEL_DROP', namespace='/button-panel')
                leftON()

        while (not pi.read(resetBtn)):
            time.sleep(flashTime)
            if time.time() > (t + delay):
                print("RESET LATCH")
                sio.emit('CHANGE_DROPPY_STATE', 'RESET_LATCH', namespace='/button-panel')
                    middleON()

        while (not pi.read(stopCutBtn)):
            time.sleep(flashTime)
            if time.time() > (t + delay):
                print("STOP CUT")
                sio.emit('CHANGE_DROPPY_STATE', 'STOP_CUT', namespace='/button-panel')
                    middleON()
                
        leftOFF()
        middleOFF()


sio.start_background_task(run)
sio.wait()
