CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation

def repeat_step_pulse(pigpio, pi, Hz, pulses, STEP):
    delay = 1000000//Hz//2
    pi.wave_add_generic([pigpio.pulse(0, 1<<STEP, delay), pigpio.pulse(1<<STEP, 0, delay)])
    return [
        255, 0, # loop start
            pi.wave_create(),
        255, 1, pulses%256, pulses//256, # loop end (repeat `pulses` times)
    ]

def move_stepper(pigpio, pi, SPR, steps, STEP, DIR):
    STEPPER_MIN_HZ = SPR / 20
    STEPPER_MAX_HZ = SPR / 5
    STEPPER_MIN_TO_MAX_SPEED_STEPS = SPR // 4 # reach max speed in a quarter revolution
    STEPPER_ACCEL_INTERVAL = SPR // 16 # how often to increase speed (in steps)
    STEPPER_ACCEL_AMOUNT = (STEPPER_MAX_HZ - STEPPER_MIN_HZ) * STEPPER_ACCEL_INTERVAL / STEPPER_MIN_TO_MAX_SPEED_STEPS

    total_steps = 0
    pi.wave_clear() # clear any existing waveforms
    if (steps > 0):
        pi.write(DIR, CCW)
    else:
        steps = -steps
        pi.write(DIR, CW)
    num_intervals = steps//2//STEPPER_ACCEL_INTERVAL
    wave_chain = []
    Hz = STEPPER_MIN_HZ
    for i in range(num_intervals):
        wave_chain.extend(repeat_step_pulse(pi, Hz, STEPPER_ACCEL_INTERVAL, STEP))
        total_steps += STEPPER_ACCEL_INTERVAL
        Hz += STEPPER_ACCEL_AMOUNT
        if (Hz >= STEPPER_MAX_HZ):
            break
    i += 1
    if i < num_intervals:
        max_speed_steps = STEPPER_ACCEL_INTERVAL * (num_intervals-i)*2
        wave_chain.extend(repeat_step_pulse(pi, STEPPER_MAX_HZ, max_speed_steps, STEP))
        total_steps += max_speed_steps
    for j in range(i):
        Hz -= STEPPER_ACCEL_AMOUNT
        wave_chain.extend(repeat_step_pulse(pi, Hz, STEPPER_ACCEL_INTERVAL, STEP))
        total_steps += STEPPER_ACCEL_INTERVAL
    leftover_steps = steps%STEPPER_ACCEL_INTERVAL
    wave_chain.extend(repeat_step_pulse(pi, STEPPER_MIN_HZ, leftover_steps, STEP))
    total_steps += leftover_steps
    assert total_steps == steps
    pi.wave_chain(wave_chain)
