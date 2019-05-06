CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation

class StepperUtils:
    def __init__(self, pigpio, pi, SPR, STEP, DIR):
        self.pigpio = pigpio
        self.pi = pi
        self.SPR = SPR
        self.STEP = STEP
        self.DIR = DIR

        self.STEPPER_MIN_HZ = self.SPR / 20
        self.STEPPER_MAX_HZ = self.SPR / 5
        self.STEPPER_MIN_TO_MAX_SPEED_STEPS = self.SPR // 4 # reach max speed in a quarter revolution
        self.STEPPER_ACCEL_INTERVAL = self.SPR // 16 # how often to increase speed (in steps)
        self.STEPPER_ACCEL_AMOUNT = (self.STEPPER_MAX_HZ - self.STEPPER_MIN_HZ) * self.STEPPER_ACCEL_INTERVAL / self.STEPPER_MIN_TO_MAX_SPEED_STEPS

    def repeat_step_pulse(self, Hz, pulses):
        delay = 1000000//Hz//2
        self.pi.wave_add_generic([self.pigpio.pulse(0, 1<<self.STEP, delay), self.pigpio.pulse(1<<self.STEP, 0, delay)])
        return [
            255, 0, # loop start
                self.pi.wave_create(),
            255, 1, pulses%256, pulses//256, # loop end (repeat `pulses` times)
        ]

    def move_stepper(self, steps):
        total_steps = 0
        self.pi.wave_clear() # clear any existing waveforms
        if (steps > 0):
            self.pi.write(self.DIR, CCW)
        else:
            steps = -steps
            self.pi.write(self.DIR, CW)
        num_intervals = steps//2//self.STEPPER_ACCEL_INTERVAL
        wave_chain = []
        Hz = self.STEPPER_MIN_HZ
        for i in range(num_intervals):
            wave_chain.extend(self.repeat_step_pulse(Hz, self.STEPPER_ACCEL_INTERVAL))
            total_steps += self.STEPPER_ACCEL_INTERVAL
            Hz += self.STEPPER_ACCEL_AMOUNT
            if (Hz >= self.STEPPER_MAX_HZ):
                break
        i += 1
        if i < num_intervals:
            max_speed_steps = self.STEPPER_ACCEL_INTERVAL * (num_intervals-i)*2
            wave_chain.extend(self.repeat_step_pulse(self.STEPPER_MAX_HZ, max_speed_steps))
            total_steps += max_speed_steps
        for j in range(i):
            Hz -= self.STEPPER_ACCEL_AMOUNT
            wave_chain.extend(self.repeat_step_pulse(Hz, self.STEPPER_ACCEL_INTERVAL))
            total_steps += self.STEPPER_ACCEL_INTERVAL
        leftover_steps = steps%self.STEPPER_ACCEL_INTERVAL
        wave_chain.extend(self.repeat_step_pulse(self.STEPPER_MIN_HZ, leftover_steps))
        total_steps += leftover_steps
        assert total_steps == steps
        self.pi.wave_chain(wave_chain)
