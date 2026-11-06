"""Slow Cartesian base placement using an xArm get_position() pose (mm, RPY degrees)."""
from dataclasses import dataclass, replace
import math
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .robot import XArm6Robot, continuous_rotvec
from .rollout import limit_action
from .control_rate import ControlRate


@dataclass(frozen=True)
class BasePlacementConfig:
    pose_mm_deg: tuple = (32.074207, 257.162689, 40.484722, -179.67395, -1.5738, -0.3)
    speed_mm_s: float = 15.0
    rotation_speed_deg_s: float = 6.0
    timeout_sec: float = 180.0

    def __post_init__(self):
        pose = np.asarray(self.pose_mm_deg, dtype=float)
        if pose.shape != (6,) or not np.isfinite(pose).all():
            raise ValueError('base_pose_mm_deg must contain six finite XYZ/RPY values')
        for name in ('speed_mm_s', 'rotation_speed_deg_s', 'timeout_sec'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'base {name} must be finite and positive')

    def target_pose(self):
        pose = np.asarray(self.pose_mm_deg, dtype=float)
        return np.concatenate((pose[:3] / 1000,
                               Rotation.from_euler('xyz', pose[3:], degrees=True).as_rotvec()))


def run_base_placement(config, placement, execute=False, stop=None):
    """Approach 10 mm above the fixed pose, then descend, checking feedback every tick.

    Prediction only reads the initial TCP and logs the destination. Execution finishes only
    after the measured pose stays within 1 mm / 0.5 degrees of the goal for 0.3 seconds.
    No camera, policy inference, sensor configuration, or gripper commands are used.
    """
    stop = stop if stop is not None else threading.Event()
    if stop.is_set():
        return 'stopped'
    motion = replace(config.motion, translation_speed=placement.speed_mm_s / 1000,
                     rotation_speed=math.radians(placement.rotation_speed_deg_s))
    robot = XArm6Robot(config.robot_ip, motion, read_only=not execute)
    try:
        robot.connect()
        initial = robot.initialize_target_pose()
        goal = placement.target_pose()
        goal[3:] = continuous_rotvec(goal[3:], initial[3:])
        if stop.is_set():
            return 'stopped'
        print(f'Base placement {"EXECUTE" if execute else "PREDICTION ONLY"}: '
              f'goal_mm_rpy_deg={list(placement.pose_mm_deg)}, '
              'via target Z + 10 mm, '
              f'speed={placement.speed_mm_s} mm/s, rotation={placement.rotation_speed_deg_s} deg/s', flush=True)
        if not execute:
            return 'done'
        robot.enable()
        hover = goal.copy()
        hover[2] += .010  # 10 mm above the destination in the robot base frame.
        return _move_goals(robot, initial, (hover, goal), motion, stop, placement.timeout_sec,
                           'Base placement')
    finally:
        robot.close()


def _move_goals(robot, initial, goals, motion, stop, timeout_sec, label, hold_xyz_tolerance=None):
    """Follow ordered waypoints, requiring measured arrival before advancing."""
    target = initial.copy()
    period = 1 / motion.loop_hz
    radius = max(float(np.linalg.norm(goal[:3] - initial[:3])) for goal in goals) + 1e-6
    deadline = time.monotonic() + timeout_sec
    rate = ControlRate(period, time.monotonic, stop.wait)
    for goal in goals:
        arrived_since = None
        while True:
            tick = time.monotonic()
            if stop.is_set():
                return 'stopped'
            if tick >= deadline:
                raise RuntimeError(f'{label} timed out before reaching the measured goal')
            measured, _ = robot.observation()
            if hold_xyz_tolerance is not None:
                drift = float(np.linalg.norm(measured[:3] - initial[:3]))
                if drift > hold_xyz_tolerance:
                    raise RuntimeError(f'{label}: unexpected XYZ drift {drift * 1000:.2f} mm '
                                       '(limit 2 mm); possible competing arm commands')
            position_error = np.linalg.norm(goal[:3] - measured[:3])
            rotation_error = (Rotation.from_rotvec(goal[3:]) *
                              Rotation.from_rotvec(measured[3:]).inv()).magnitude()
            target_at_goal = (np.linalg.norm(goal[:3] - target[:3]) < 1e-8 and
                              (Rotation.from_rotvec(goal[3:]) *
                               Rotation.from_rotvec(target[3:]).inv()).magnitude() < 1e-8)
            if target_at_goal and position_error <= .001 and rotation_error <= math.radians(.5):
                arrived_since = tick if arrived_since is None else arrived_since
                if tick - arrived_since >= .3:
                    break
            else:
                arrived_since = None
            target = limit_action(target, goal, motion, period, initial, True, radius)
            if stop.is_set():
                return 'stopped'
            robot.servo(target, period, measured=measured)
            rate.wait()
    return 'done'


def align_policy_yaw(config, execute=False, stop=None):
    """Set base-frame Euler yaw to 90 degrees while holding XYZ, roll, and pitch."""
    stop = stop if stop is not None else threading.Event()
    if stop.is_set():
        return 'stopped'
    motion = replace(config.motion, rotation_speed=math.radians(9.0))
    robot = XArm6Robot(config.robot_ip, motion, read_only=not execute)
    try:
        robot.connect()
        initial = robot.initialize_target_pose()
        rpy = Rotation.from_rotvec(initial[3:]).as_euler('xyz', degrees=True)
        yaw_error = (90.0 - rpy[2] + 180.0) % 360.0 - 180.0
        if stop.is_set():
            return 'stopped'
        if abs(yaw_error) <= .5:
            return 'done'
        print(f'Policy yaw alignment {"EXECUTE" if execute else "PREDICTION ONLY"}: '
              f'{rpy[2]:.3f} -> 90 degrees at 9 deg/s; '
              f'holding XYZ mm={(initial[:3] * 1000).round(3).tolist()}, '
              f'roll/pitch deg={rpy[:2].round(3).tolist()}', flush=True)
        if not execute:
            return 'done'
        rpy[2] = 90.0
        goal = initial.copy()
        goal[3:] = continuous_rotvec(Rotation.from_euler('xyz', rpy, degrees=True).as_rotvec(), initial[3:])
        robot.enable()
        return _move_goals(robot, initial, (goal,), motion, stop, 180.0, 'Policy yaw alignment',
                           hold_xyz_tolerance=.002)
    finally:
        robot.close()
