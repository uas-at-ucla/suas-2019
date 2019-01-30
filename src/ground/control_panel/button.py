import pigpio
pi = pigpio.pi()
pi.set_mode(4, pigpio.INPUT)  # GPIO 4 as input
pi.set_pull_up_down(4, pigpio.PUD_UP)
while True:
    print(pi.read(4)) # 1 = button not pressed, 0 = button pressed