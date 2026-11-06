"""Release the plate and retreat vertically after successful placement."""
import math
import threading

from .base_placement import _move_goals
from .robot import XArm6Robot


def release_and_lift(config, execute=False, stop=None, *, lift_mm=100.0):
    """Open once, allow 4 s for jaw travel, then lift along base-frame +Z.

    Hold XY/orientation and use the configured ACT translation limit. A stopped or failed
    open command prevents the lift. Prediction only reads the TCP and logs this sequence.
    """
    if not math.isfinite(lift_mm) or lift_mm <= 0:
        raise ValueError('Post-placement lift_mm must be finite and positive')
    stop = stop if stop is not None else threading.Event()
    if stop.is_set():
        return 'stopped'
    robot = XArm6Robot(config.robot_ip, config.motion, read_only=not execute)
    try:
        robot.connect()
        robot.initialize_target_pose()
        if stop.is_set():
            return 'stopped'
        print(f'Post-placement {"EXECUTE" if execute else "PREDICTION ONLY"}: '
              f'open gripper, wait 4 s, lift {lift_mm:g} mm along base +Z', flush=True)
        if not execute:
            return 'done'
        robot.open_gripper()
        if stop.wait(4.0):
            return 'stopped'
        initial = robot.initialize_target_pose()
        goal = initial.copy()
        goal[2] += lift_mm / 1000
        if stop.is_set():
            return 'stopped'
        robot.enable()
        return _move_goals(robot, initial, (goal,), config.motion, stop, 180.0, 'Post-placement lift')
    finally:
        robot.close()
