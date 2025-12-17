# states/training/behavior.py

from pid import RoverPID

# Internal flag to track if training is finished
_training_done = False

# Speed to use during training (m/s)
TRAINING_SPEED = 0.2    # tune this value as you like


def start_training(rover_pid: RoverPID):
    """
    Called once when the FSM enters the TRAINING state.
    """
    global _training_done
    _training_done = False

    print("Current state: Start TRAINING")
    stop_training_motion(rover_pid)


def training_step(rover_pid: RoverPID, key=None):
    """
    Called repeatedly while in TRAINING.

    key:
        'w' / 'W' -> forward
        's' / 'S' -> backward
        'a' / 'A' -> turn left
        'd' / 'D' -> turn right
        'f' / 'F' -> finish training
    """
    if key is None:
        return

    k = key.lower()

    if k == 'w':
        move_forward(rover_pid)
    elif k == 's':
        move_backward(rover_pid)
    elif k == 'a':
        turn_left(rover_pid)
    elif k == 'd':
        turn_right(rover_pid)
    elif k == 'f':
        set_training_done()
        stop_training_motion(rover_pid)
        print("Training finished.")
    else:
        print("use w a s d to move or f to finish")


def is_training_done():
    return _training_done


def set_training_done():
    global _training_done
    _training_done = True
    print("TRAINING Done")


def stop_training_motion(rover_pid: RoverPID):
    """
    Stop any motion during training.
    """
    rover_pid.set_target(0.0, 0.0)
    print("TRAINING: stop")


def move_forward(rover_pid: RoverPID):
    """
    Move the rover forward during training (W).
    """
    rover_pid.set_target(+TRAINING_SPEED, +TRAINING_SPEED)
    print("TRAINING: forward")


def move_backward(rover_pid: RoverPID):
    """
    Move the rover backward during training (S).
    """
    rover_pid.set_target(-TRAINING_SPEED, -TRAINING_SPEED)
    print("TRAINING: backward")


def turn_left(rover_pid: RoverPID):
    """
    Turn the rover left in place during training (A).
    """
    rover_pid.set_target(-TRAINING_SPEED, +TRAINING_SPEED)
    print("TRAINING: turn left")


def turn_right(rover_pid: RoverPID):
    """
    Turn the rover right in place during training (D).
    """
    rover_pid.set_target(+TRAINING_SPEED, -TRAINING_SPEED)
    print("TRAINING: turn right")
