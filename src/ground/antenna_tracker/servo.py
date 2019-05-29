# IMPORTANT: run `pigpiod` from the command line before running this script
# Servo power: 4.5 - 7.5 V
import pigpio
import time

pi = pigpio.pi()

pi.set_mode(18, pigpio.OUTPUT) # GPIO 18 as output

# 0 = stop sending signal
# MIN = 1000
# MAX = 2000

pi.set_servo_pulsewidth(18, 0)    # off
time.sleep(1)
pi.set_servo_pulsewidth(18, 1000) # retracted
time.sleep(10)
pi.set_servo_pulsewidth(18, 1100)
time.sleep(5)
pi.set_servo_pulsewidth(18, 1200)
time.sleep(5)
pi.set_servo_pulsewidth(18, 1300)
time.sleep(5)
pi.set_servo_pulsewidth(18, 1400)
time.sleep(5)
pi.set_servo_pulsewidth(18, 1500)
time.sleep(5)
pi.set_servo_pulsewidth(18, 1600)
time.sleep(5)
pi.set_servo_pulsewidth(18, 1700)
time.sleep(5)
pi.set_servo_pulsewidth(18, 1800)
time.sleep(5)
pi.set_servo_pulsewidth(18, 1900)
time.sleep(5)
pi.set_servo_pulsewidth(18, 2000) # extended
time.sleep(5)
pi.set_servo_pulsewidth(18, 0)    # off
