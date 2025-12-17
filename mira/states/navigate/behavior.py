# states/navigate/behavior.py

from pid import RoverPID

"""
NAVIGATE state behavior.

This state now uses the RoverPID controller instead of direct GPIO.
Commands:
    'w' -> forward
    's' -> backward
    'a' -> turn left
    'd' -> turn right
    anything else -> stop
"""

# Speed to use during navigate (m/s)
NAV_SPEED = 0.2    # tune as needed


def init_motor_pins():
    """
    Kept only for compatibility with old code.

    Motor pins and GPIO initialization are handled inside pid.RoverPID.
    This function does nothing and can be removed once old calls are gone.
    """
    pass


def start_navigate(rover_pid: RoverPID):
    """
    Called once when the FSM enters the NAVIGATE state.
    """
    print("Current state: NAVIGATE")
    stop_motion(rover_pid)


def navigate_step(rover_pid: RoverPID, command):
    """
    Called repeatedly while the FSM is in the NAVIGATE state.

    command:
        'f' / 'F' -> move forward
        'b' / 'B' -> move backward
        'l' / 'L' -> turn left
        'r' / 'R' -> turn right
        anything else or None -> stop
    """
    if command is None:
        return

    c = command.lower()

    if c == 'w':
        move_forward(rover_pid)
    elif c == 's':
        move_backward(rover_pid)
    elif c == 'a':
        turn_left(rover_pid)
    elif c == 'd':
        turn_right(rover_pid)
    else:
        stop_motion(rover_pid)


# ---------------- BASIC MOTION FUNCTIONS (via PID) ----------------

def move_forward(rover_pid: RoverPID):
    """
    Drive both sides forward.
    """
    rover_pid.set_target(+NAV_SPEED, +NAV_SPEED)
    print("NAVIGATE: moving forward")


def move_backward(rover_pid: RoverPID):
    """
    Drive both sides backward.
    """
    rover_pid.set_target(-NAV_SPEED, -NAV_SPEED)
    print("NAVIGATE: moving backward")


def turn_left(rover_pid: RoverPID):
    """
    Turn in place to the left:
    left wheel backward, right wheel forward.
    """
    rover_pid.set_target(-NAV_SPEED, +NAV_SPEED)
    print("NAVIGATE: turning left")


def turn_right(rover_pid: RoverPID):
    """
    Turn in place to the right:
    left wheel forward, right wheel backward.
    """
    rover_pid.set_target(+NAV_SPEED, -NAV_SPEED)
    print("NAVIGATE: turning right")


def stop_motion(rover_pid: RoverPID):
    """
    Stop all motion.
    """
    rover_pid.set_target(0.0, 0.0)
    print("NAVIGATE: stop")


def cleanup_gpio(rover_pid: RoverPID):
    """
    Release GPIO resources on program exit via RoverPID.
    """
    rover_pid.cleanup()
    print("NAVIGATE: GPIO cleanup done")
