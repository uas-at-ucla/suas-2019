# IMPORTANT: run `pigpiod` from the command line before running this script
import pigpio
import time

pi = pigpio.pi()
pi.set_mode(4, pigpio.INPUT)  # GPIO 4 as input
pi.set_pull_up_down(4, pigpio.PUD_UP)
pi.set_mode(17, pigpio.OUTPUT) # GPIO 17 as output
led = 0

while True:
    print(pi.read(4)) # 1 = button not pressed, 0 = button pressed
    led = 1 - led
    pi.write(17, led) # set tom's GPIO 4 to high
    time.sleep(0.2)