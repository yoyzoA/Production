# test/test_pid.py
#
# Tests for mira.pid:
# - WheelPID basic PID behaviour (sign, integral, stop)
# - _set_motor duty clamping / MIN_DUTY_TO_MOVE
# - RoverPID.update uses elapsed time for both wheels
#
# These tests DO NOT touch real hardware: they fake RPi.GPIO completely.

import os
import sys
import math
import types
import importlib

import pytest

# Ensure project root ("fyp") is on sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


def import_pid_with_fake_gpio():
    """
    Create a fake RPi.GPIO module, register it in sys.modules, then import
    mira.pid using that fake GPIO. This keeps tests hardware-independent.
    """
    # If already imported once, just reuse it
    if "mira.pid" in sys.modules:
        pid_mod = sys.modules["mira.pid"]
        gpio_mod = sys.modules.get("RPi.GPIO")
        return pid_mod, gpio_mod

    # --- Fake GPIO module ---
    fake_gpio = types.ModuleType("RPi.GPIO")

    # Constants used in pid.py (we include extra ones just in case)
    fake_gpio.BCM = "BCM"
    fake_gpio.IN = "IN"
    fake_gpio.OUT = "OUT"
    fake_gpio.PUD_UP = "PUD_UP"
    fake_gpio.PUD_DOWN = "PUD_DOWN"
    fake_gpio.HIGH = 1
    fake_gpio.LOW = 0

    # *** NEW: edge-detection constants used by Encoder ***
    fake_gpio.BOTH = "BOTH"
    fake_gpio.RISING = "RISING"
    fake_gpio.FALLING = "FALLING"

    # Dummy functions
    def noop(*args, **kwargs):
        return None

    fake_gpio.setmode = noop
    fake_gpio.setup = noop
    fake_gpio.add_event_detect = noop
    fake_gpio.output = noop
    fake_gpio.cleanup = noop
    # For encoder initial state; always 0
    fake_gpio.input = lambda pin: 0

    # Fake PWM class
    class FakePWM:
        def __init__(self, pin, freq):
            self.pin = pin
            self.freq = freq
            self.last_duty = 0.0

        def start(self, duty):
            self.last_duty = duty

        def ChangeDutyCycle(self, duty):
            self.last_duty = duty

        def stop(self):
            pass

    fake_gpio.PWM = FakePWM

    # Fake RPi package that exposes GPIO submodule
    rpi_pkg = types.ModuleType("RPi")
    rpi_pkg.GPIO = fake_gpio
    rpi_pkg.__path__ = []    # make it behave like a package

    sys.modules["RPi"] = rpi_pkg
    sys.modules["RPi.GPIO"] = fake_gpio

    # Now import the module under test
    pid_mod = importlib.import_module("mira.pid")
    return pid_mod, fake_gpio


# ----------------------------------------------------------------------
# WheelPID: basic PID behaviour
# ----------------------------------------------------------------------

def _make_wheel(pid_mod, enc_stub=None):
    """Helper: create a WheelPID with an encoder stub."""
    if enc_stub is None:
        class EncoderStub:
            def read_and_reset(self):
                return 0
        enc_stub = EncoderStub()
    # Pin numbers are irrelevant because GPIO is fake
    return pid_mod.WheelPID("L", enc_stub, en_pin=12, in1_pin=17, in2_pin=27)


def test_wheelpid_zero_error_produces_zero_duty():
    pid_mod, _ = import_pid_with_fake_gpio()

    wheel = _make_wheel(pid_mod)

    # Capture duty values by patching _set_motor on this instance
    duties = []

    def fake_set_motor(self, duty):
        duties.append(duty)

    wheel._set_motor = fake_set_motor.__get__(wheel, pid_mod.WheelPID)

    # Zero target, zero ticks => zero error
    wheel.set_target(0.0)
    wheel.update(dt=0.1)

    assert duties, "update() should call _set_motor at least once"
    assert abs(duties[-1]) < 1e-6
    assert wheel.v_meas == 0.0


def test_wheelpid_positive_and_negative_error_control_duty_sign():
    pid_mod, _ = import_pid_with_fake_gpio()

    class EncoderZero:
        def read_and_reset(self):
            return 0    # always zero speed

    # Positive target => positive duty
    wheel_pos = pid_mod.WheelPID("L", EncoderZero(), 12, 17, 27)
    duties_pos = []

    def fake_set_motor_pos(self, duty):
        duties_pos.append(duty)

    wheel_pos._set_motor = fake_set_motor_pos.__get__(wheel_pos, pid_mod.WheelPID)
    wheel_pos.set_target(0.5)    # m/s
    wheel_pos.update(dt=0.1)
    assert duties_pos[-1] > 0

    # Negative target => negative duty
    wheel_neg = pid_mod.WheelPID("L", EncoderZero(), 12, 17, 27)
    duties_neg = []

    def fake_set_motor_neg(self, duty):
        duties_neg.append(duty)

    wheel_neg._set_motor = fake_set_motor_neg.__get__(wheel_neg, pid_mod.WheelPID)
    wheel_neg.set_target(-0.5)    # m/s
    wheel_neg.update(dt=0.1)
    assert duties_neg[-1] < 0


def test_wheelpid_integral_makes_duty_grow_over_time():
    pid_mod, _ = import_pid_with_fake_gpio()

    class EncoderZero:
        def read_and_reset(self):
            return 0

    wheel = pid_mod.WheelPID("L", EncoderZero(), 12, 17, 27)
    duties = []

    def fake_set_motor(self, duty):
        duties.append(duty)

    wheel._set_motor = fake_set_motor.__get__(wheel, pid_mod.WheelPID)

    wheel.set_target(0.5)    # constant positive target
    # Call update multiple times with same dt
    for _ in range(5):
        wheel.update(dt=0.1)

    # Duty should be positive and generally increasing (integral builds up)
    assert all(d > 0 for d in duties)
    assert duties[-1] > duties[0]


def test_wheelpid_stop_resets_state_and_sends_zero_duty():
    pid_mod, _ = import_pid_with_fake_gpio()

    class EncoderZero:
        def read_and_reset(self):
            return 0

    wheel = pid_mod.WheelPID("L", EncoderZero(), 12, 17, 27)

    # Let it accumulate some integral
    wheel.set_target(0.5)
    wheel.update(dt=0.1)
    wheel.update(dt=0.1)

    last_duty = []

    def fake_set_motor(self, duty):
        last_duty.append(duty)

    wheel._set_motor = fake_set_motor.__get__(wheel, pid_mod.WheelPID)

    wheel.stop()

    # Internals reset
    assert wheel.v_target == 0.0
    assert wheel._i_term == 0.0
    assert wheel._prev_err == 0.0
    # Motor commanded to 0
    assert last_duty[-1] == 0.0


def test_set_motor_enforces_min_duty_to_move():
    pid_mod, fake_gpio = import_pid_with_fake_gpio()

    # Use a dummy encoder; we won't call update here.
    class EncoderZero:
        def read_and_reset(self):
            return 0

    wheel = pid_mod.WheelPID("L", EncoderZero(), 12, 17, 27)

    # Call the real _set_motor, but inspect fake PWM's last_duty.
    # Use a small non-zero duty; should be raised to MIN_DUTY_TO_MOVE.
    small_duty = 5.0
    wheel._set_motor(small_duty)

    pwm_obj = wheel.pwm    # FakePWM instance
    assert pwm_obj.last_duty >= pid_mod.MIN_DUTY_TO_MOVE
    assert pwm_obj.last_duty <= pid_mod.MAX_DUTY


# ----------------------------------------------------------------------
# RoverPID: high-level wrapper behaviour
# ----------------------------------------------------------------------

def test_roverpid_update_calls_both_wheels_with_dt(monkeypatch):
    pid_mod, _ = import_pid_with_fake_gpio()

    # Replace WheelPID.update to record calls (name, dt)
    calls = []

    def fake_update(self, dt):
        calls.append((self.name, dt))

    monkeypatch.setattr(pid_mod.WheelPID, "update", fake_update)

    # Control time.time() so we get a known dt
    times = [0.0, 0.1]    # first call: 0.0, second call: 0.1 => dt=0.1

    def fake_time():
        return times.pop(0)

    monkeypatch.setattr(pid_mod.time, "time", fake_time)

    rover = pid_mod.RoverPID()

    # First update: just initializes _last_time
    rover.update()
    # Second update: should call fake_update on both wheels with dt=0.1
    rover.update()

    assert calls == [
        ("L", pytest.approx(0.1, rel=1e-6)),
        ("R", pytest.approx(0.1, rel=1e-6)),
    ]
    