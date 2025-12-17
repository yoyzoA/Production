# states/idle/behavior.py

from pid import RoverPID

def idle_step(rover_pid: RoverPID):
 
    print("Current state: IDLE")
    stop_all_motion(rover_pid)


def stop_all_motion(rover_pid: RoverPID):
    """
    This sets both wheel target speeds to 0 m/s.
    """
    rover_pid.set_target(0.0, 0.0)
