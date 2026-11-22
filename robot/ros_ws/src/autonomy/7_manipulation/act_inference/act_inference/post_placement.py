"""Release the plate and retreat vertically after successful placement."""
from dataclasses import replace
import math
import threading

from .base_placement import _move_goals
from .robot import XArm6Robot


def release_and_lift(config, execute=False, stop=None, *, lift_mm=100.0, speed_mm_s=None, accel_mm_s2=None,
                     open_wait_sec=4.0, report_type='rich', arrival_sec=.3, shift_x_mm=0.0):
    """Optionally shift along base-frame X, open once, allow `open_wait_sec` for jaw travel, then lift along
    base-frame +Z.

    `shift_x_mm` (non-zero): before opening, move the closed gripper that far along base-frame X, holding Y, Z and
    orientation, at the configured ACT translation speed (or the lift speed if lower), until measured arrival.

    Hold XY/orientation and lift at `speed_mm_s` (None: the configured ACT translation limit), speeding up and
    slowing down at `accel_mm_s2` (None: full speed at once), until the measured TCP has held the top for
    `arrival_sec`. `report_type` is the xArm report socket (XArm6Robot). A stopped or failed open command prevents
    the lift. Prediction only reads the TCP and logs this sequence.
    """
    if not math.isfinite(lift_mm) or lift_mm <= 0:
        raise ValueError('Post-placement lift_mm must be finite and positive')
    if speed_mm_s is not None and (not math.isfinite(speed_mm_s) or speed_mm_s <= 0):
        raise ValueError('Post-placement speed_mm_s must be finite and positive')
    if accel_mm_s2 is not None and (not math.isfinite(accel_mm_s2) or accel_mm_s2 <= 0):
        raise ValueError('Post-placement accel_mm_s2 must be finite and positive')
    if not math.isfinite(open_wait_sec) or open_wait_sec < 0:
        raise ValueError('Post-placement open_wait_sec must be finite and non-negative')
    if not math.isfinite(shift_x_mm):
        raise ValueError('Post-placement shift_x_mm must be finite')
    stop = stop if stop is not None else threading.Event()
    if stop.is_set():
        return 'stopped'
    motion = config.motion if speed_mm_s is None else replace(config.motion, translation_speed=speed_mm_s / 1000)
    robot = XArm6Robot(config.robot_ip, motion, read_only=not execute, report_type=report_type)
    try:
        robot.connect()
        robot.initialize_target_pose()
        if stop.is_set():
            return 'stopped'
        shift_motion = replace(motion, translation_speed=min(motion.translation_speed,
                                                             config.motion.translation_speed))
        print(f'Post-placement {"EXECUTE" if execute else "PREDICTION ONLY"}: '
              + (f'move {shift_x_mm:+g} mm along base X at {shift_motion.translation_speed * 1000:g} mm/s, '
                 if shift_x_mm else '')
              + f'open gripper, wait {open_wait_sec:g} s, lift {lift_mm:g} mm along base +Z at '
              f'{motion.translation_speed * 1000:g} mm/s'
              + (f' ({accel_mm_s2:g} mm/s^2 ramps)' if accel_mm_s2 is not None else ''), flush=True)
        if not execute:
            return 'done'
        if shift_x_mm:
            start = robot.initialize_target_pose()
            shifted = start.copy()
            shifted[0] += shift_x_mm / 1000
            robot.enable()
            result = _move_goals(robot, start, (shifted,), shift_motion, stop, 10.0, 'Pre-release X shift',
                                 arrival_sec=arrival_sec)
            if result != 'done' or stop.is_set():
                return 'stopped' if stop.is_set() else result
        robot.open_gripper()
        if stop.wait(open_wait_sec):
            return 'stopped'
        initial = robot.initialize_target_pose()
        goal = initial.copy()
        goal[2] += lift_mm / 1000
        if stop.is_set():
            return 'stopped'
        robot.enable()
        return _move_goals(robot, initial, (goal,), motion, stop, 180.0, 'Post-placement lift',
                           accel=None if accel_mm_s2 is None else accel_mm_s2 / 1000, arrival_sec=arrival_sec)
    finally:
        robot.close()
