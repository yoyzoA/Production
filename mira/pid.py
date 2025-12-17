# pid.py
#
# Simple 2-wheel velocity PID controller for Rover 5 using:
# - 2 quadrature encoders (left & right)
# - L298N motor driver
# - RPi.GPIO only
#
# In NAVIGATE / TRAINING you can map keys/commands to:
# W -> set_target(+v, +v)
# S -> set_target(-v, -v)
# A -> set_target(-v, +v)
# D -> set_target(+v, -v)

import RPi.GPIO as GPIO
import time
import math

# From Rover 5 datasheet:
TICKS_PER_REV = 1000.0 / 3.0    # 333.33 ticks per wheel revolution
WHEEL_RADIUS_M = 0.03
DIST_PER_TICK = 2.0 * math.pi * WHEEL_RADIUS_M / TICKS_PER_REV

# Motor driver (L298N) pins
LEFT_EN  = 12    # ENA (PWM)
LEFT_IN1 = 17    # IN1
LEFT_IN2 = 27    # IN2

RIGHT_EN  = 13    # ENB (PWM)
RIGHT_IN1 = 22    # IN3
RIGHT_IN2 = 23    # IN4

ENC_L_A = 5
ENC_L_B = 6

ENC_R_A = 20
ENC_R_B = 21

# PWM frequency [Hz] for motor speed control
PWM_FREQ = 100.0

# PID gains
KP = 8.0
KI = 2.0
KD = 0.0

# Duty cycle limits
MAX_DUTY = 100.0
MIN_DUTY_TO_MOVE = 20.0    # minimum duty to overcome friction (tune)

# Quadrature Encoder Matrix (from Rover 5 doc)
# Out = QEM[old_state * 4 + new_state] -> -1, 0, +1 or 2 (error)
QEM = [0, -1,  1,  2,
       1,  0,  2, -1,
      -1,  2,  0,  1,
       2,  1, -1,  0]


class WheelEncoder:
    """Quadrature encoder using two GPIO pins (A,B)."""

    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self._ticks = 0
        self._prev_state = 0

        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Initial state
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        self._prev_state = (a << 1) | b

        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._callback)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self._callback)

    def _callback(self, channel):
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        new_state = (a << 1) | b
        index = (self._prev_state << 2) | new_state
        delta = QEM[index]

        # Ignore "2" (invalid transition) but count -1 / +1
        if delta != 2:
            self._ticks += delta

        self._prev_state = new_state

    def read_and_reset(self):
        ticks = self._ticks
        self._ticks = 0
        return ticks

class WheelPID:
    """One wheel: encoder -> speed -> PID -> motor PWM."""

    def __init__(self, name, enc, en_pin, in1_pin, in2_pin):
        self.name = name
        self.encoder = enc
        self.en_pin = en_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin

        GPIO.setup(self.en_pin, GPIO.OUT)
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self.en_pin, PWM_FREQ)
        self.pwm.start(0.0)

        # PID state
        self.v_target = 0.0    # [m/s]
        self.v_meas = 0.0      # [m/s]
        self._i_term = 0.0
        self._prev_err = 0.0

    def set_target(self, v_mps):
        self.v_target = float(v_mps)

    def _set_motor(self, duty):
        """Set motor direction + duty (signed duty in [-100,100])."""
        if abs(duty) < 1e-3:
            # stop
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.LOW)
            self.pwm.ChangeDutyCycle(0.0)
            return

        forward = duty > 0
        d = min(MAX_DUTY, max(0.0, abs(duty)))

        # Enforce minimum duty when we want to move
        if d < MIN_DUTY_TO_MOVE:
            d = MIN_DUTY_TO_MOVE

        if forward:
            GPIO.output(self.in1_pin, GPIO.HIGH)
            GPIO.output(self.in2_pin, GPIO.LOW)
        else:
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.HIGH)

        self.pwm.ChangeDutyCycle(d)

    def update(self, dt):
        """Run one PID step. dt in seconds."""
        if dt <= 0:
            return

        # 1) Read ticks since last update
        dcounts = self.encoder.read_and_reset()

        # 2) Convert to speed [m/s]
        distance = dcounts * DIST_PER_TICK
        self.v_meas = distance / dt

        # 3) PID
        err = self.v_target - self.v_meas

        # Integral
        self._i_term += KI * err * dt

        # Anti-windup clamp
        self._i_term = max(-MAX_DUTY, min(MAX_DUTY, self._i_term))

        # Derivative
        derr = (err - self._prev_err) / dt
        self._prev_err = err

        # PID output as duty
        duty = KP * err + self._i_term + KD * derr

        # Saturate duty
        if duty > MAX_DUTY:
            duty = MAX_DUTY
        elif duty < -MAX_DUTY:
            duty = -MAX_DUTY

        self._set_motor(duty)

    def stop(self):
        self.v_target = 0.0
        self._i_term = 0.0
        self._prev_err = 0.0
        self._set_motor(0.0)

    def cleanup(self):
        self.stop()
        self.pwm.stop()


# -------------------- HIGH-LEVEL ROVER PID --------------------

class RoverPID:
    """
    High-level PID controller for the Rover:
    - 2 wheels (left/right)
    - set target speeds in m/s
    - call update() regularly
    """

    def __init__(self):
        GPIO.setmode(GPIO.BCM)

        # Encoders
        enc_left = WheelEncoder(ENC_L_A, ENC_L_B)
        enc_right = WheelEncoder(ENC_R_A, ENC_R_B)

        # Wheel controllers
        self.left = WheelPID("L", enc_left, LEFT_EN, LEFT_IN1, LEFT_IN2)
        self.right = WheelPID("R", enc_right, RIGHT_EN, RIGHT_IN1, RIGHT_IN2)

        self._last_time = None

    def set_target(self, v_left_mps, v_right_mps):
        """Set desired wheel speeds [m/s]."""
        self.left.set_target(v_left_mps)
        self.right.set_target(v_right_mps)

    def update(self):
        """
        Run one PID cycle.
        Call this in your main loop as often as possible.
        """
        now = time.time()
        if self._last_time is None:
            self._last_time = now
            return

        dt = now - self._last_time
        self._last_time = now

        if dt <= 0:
            return

        self.left.update(dt)
        self.right.update(dt)

    def stop(self):
        """Stop both wheels immediately and clear integrators."""
        self.left.stop()
        self.right.stop()

    def cleanup(self):
        """Stop motors and release GPIO resources."""
        self.stop()
        self.left.cleanup()
        self.right.cleanup()
        GPIO.cleanup()
