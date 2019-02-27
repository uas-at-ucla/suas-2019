# IMPORTANT: Enable I2C on the RPi using `sudo raspi-config` -> Interfacing Options
# IMPORTANT: run `pigpiod` from the command line before running this script
# See for (Arduino) examples https://github.com/sparkfun/OpenLCD/tree/2829d059a7a7dc137ab48a64771079831a65c3bd/firmware/Examples
#     Note that that is an old version, because the firmware in our LCD is outdated. It is possible to update with an FTDI programmer:
#     https://learn.sparkfun.com/tutorials/avr-based-serial-enabled-lcds-hookup-guide/all#troubleshooting
import pigpio
import time
import colorsys

LCD_I2C_ADDRESS = 0x72

pi = pigpio.pi()
h = pi.i2c_open(1, LCD_I2C_ADDRESS) # open device at address 0x72 on bus 1

pi.i2c_write_byte(1, 0x7C) # Put LCD into setting mode
pi.i2c_write_byte(1, 0x2D) # Send clear display command

pi.i2c_write_byte(1, 0x7C) # Put LCD into setting mode
pi.i2c_write_byte(1, 0x18) # contrast command
pi.i2c_write_byte(1, 0) # max contrast

pi.i2c_write_device(h, b"UAS at UCLA!!!  UAS at UCLA!!!")

H = 0
S = 1
V = 1

while True:
    R,G,B = colorsys.hsv_to_rgb(H,S,V)

    # Unfortunately, the LCD prints an annoying message every time you change the backlight color/brightness.
    # This would be easy to remove if we could reprogram it (see top of file).

    pi.i2c_write_byte(1, 0x7C) # Put LCD into setting mode
    pi.i2c_write_byte(1, 128 + round(R*29)) #  white/red

    pi.i2c_write_byte(1, 0x7C) # Put LCD into setting mode
    pi.i2c_write_byte(1, 158 + round(G*29)) # green

    pi.i2c_write_byte(1, 0x7C) # Put LCD into setting mode
    pi.i2c_write_byte(1, 188 + round(B*29)) # blue

    H = H + 0.1
    if (H >= 1):
        H = 0

    time.sleep(3)

pi.i2c_close(h)