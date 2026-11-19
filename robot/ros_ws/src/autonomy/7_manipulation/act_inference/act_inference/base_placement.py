"""Base placement using an xArm get_position() pose (mm, RPY degrees), seated by the wrist F/T sensor."""
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
import math
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .robot import XArm6Robot, continuous_rotvec
from .rollout import limit_action
from .control_rate import ControlRate

DEFAULT_DEBUG_DIR = '~/.ros/log/act_inference_base_placement'
BASE_ARRIVAL_SEC = .1   # place_base and its lift: measured arrival held this long (100 Hz 'real' reports)


@dataclass(frozen=True)
class BasePlacementConfig:
    pose_mm_deg: tuple = (51.565662, 257.200928, 47.687004, 179.332982, -2.176667, 1.803213)   # probing start
    speed_mm_s: float = 40.0
    rotation_speed_deg_s: float = 6.0
    # From above (place_base starts at home_offset, ~180 mm away) the move to the probing start goes at
    # approach_speed_mm_s to approach_clearance_mm straight above it, then down the rest at speed_mm_s.
    approach_speed_mm_s: float = 150.0
    approach_clearance_mm: float = 20.0
    lift_speed_mm_s: float = 150.0        # the lift after the release (node: release_and_lift)
    # Both approach moves and the lift speed up and slow down at this rate instead of jumping to full speed.
    accel_mm_s2: float = 500.0
    timeout_sec: float = 180.0
    # F/T seating (run_base_placement): from pose_mm_deg, descend until |Fz - in-air baseline| > touch_force_n,
    # then slide along base -X until the sideways force |(Fx, Fy) - baseline after the touch| > ridge_force_n
    # (the plate edge meets the ridge). The first probe_fast_mm of the descent and slide_fast_mm of the slide
    # (before the deck, ~7 mm down, and the ridge, ~19 mm along) go at fast_speed_mm_s, still force-guarded.
    probe_speed_mm_s: float = 2.0
    probe_depth_mm: float = 11.0          # no touch this far below the start fails the run
    touch_force_n: float = 1.0
    slide_speed_mm_s: float = 4.0
    slide_max_mm: float = 30.0            # no ridge after this much -X travel fails the run
    ridge_force_n: float = 1.0
    fast_speed_mm_s: float = 10.0
    probe_fast_mm: float = 3.5
    slide_fast_mm: float = 12.0
    force_count: int = 2                  # consecutive 50 Hz reads above a threshold that count as contact
    settle_sec: float = 0.2               # still before each F/T baseline
    baseline_sec: float = 0.2             # F/T baseline: median over this window
    open_wait_sec: float = 1.5            # jaw travel after the open command, before the lift (node: release_and_lift)
    debug: bool = False                   # per-tick CSV + 10 Hz console trace of pose and force
    debug_dir: str = DEFAULT_DEBUG_DIR

    def __post_init__(self):
        pose = np.asarray(self.pose_mm_deg, dtype=float)
        if pose.shape != (6,) or not np.isfinite(pose).all():
            raise ValueError('base_pose_mm_deg must contain six finite XYZ/RPY values')
        for name in ('speed_mm_s', 'rotation_speed_deg_s', 'timeout_sec', 'probe_speed_mm_s', 'probe_depth_mm',
                     'touch_force_n', 'slide_speed_mm_s', 'slide_max_mm', 'ridge_force_n',
                     'settle_sec', 'baseline_sec', 'approach_speed_mm_s', 'lift_speed_mm_s', 'accel_mm_s2',
                     'fast_speed_mm_s', 'open_wait_sec'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'base {name} must be finite and positive')
        for name in ('approach_clearance_mm', 'probe_fast_mm', 'slide_fast_mm'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f'base {name} must be finite and non-negative')
        if (self.probe_speed_mm_s > self.speed_mm_s or self.slide_speed_mm_s > self.speed_mm_s
                or self.fast_speed_mm_s > self.speed_mm_s):
            raise ValueError('base probe/slide/fast speeds must not exceed speed_mm_s')
        if self.probe_fast_mm >= self.probe_depth_mm or self.slide_fast_mm >= self.slide_max_mm:
            raise ValueError('base probe_fast_mm/slide_fast_mm must be below probe_depth_mm/slide_max_mm')
        if int(self.force_count) != self.force_count or self.force_count < 1:
            raise ValueError('base force_count must be a positive integer')

    def target_pose(self):
        pose = np.asarray(self.pose_mm_deg, dtype=float)
        return np.concatenate((pose[:3] / 1000,
                               Rotation.from_euler('xyz', pose[3:], degrees=True).as_rotvec()))


class ForceReader:
    """Wrist F/T (ft_ext_force: Fx, Fy, Fz N, Tx, Ty, Tz Nm, SDK sensor frame) from a second, read-only SDK
    connection on the fast 'real' report socket, as visual_servoing reads it: the main connection's rich report
    updates too slowly to stop a touch. Every report is timestamped; a read older than `max_age` s fails."""

    def __init__(self, robot_ip, factory=None, max_age=.1):
        self.robot_ip, self._factory, self.max_age = robot_ip, factory, max_age
        self._api, self._stamp = None, None

    def connect(self):
        if self._factory is None:
            from xarm.wrapper import XArmAPI
            self._factory = lambda ip: XArmAPI(ip, is_radian=True, report_type='real')
        try:
            self._api = self._factory(self.robot_ip)
        except Exception as exc:   # the SDK raises a bare Exception when the socket does not connect
            raise RuntimeError(f'xArm F/T report connection to {self.robot_ip} failed: {exc}') from exc
        if not self._api.connected:
            raise RuntimeError(f'xArm F/T report connection to {self.robot_ip} failed')
        if not self._api.register_report_callback(self._on_report, report_cartesian=False, report_joints=False,
                                                  report_mtable=False, report_mtbrake=False, report_cmd_num=False):
            raise RuntimeError('Could not register xArm F/T reports')
        deadline = time.monotonic() + 2.
        while self._stamp is None:
            if time.monotonic() >= deadline:
                raise RuntimeError('No xArm F/T report received')
            time.sleep(.01)

    def _on_report(self, _data):
        self._stamp = time.monotonic()

    def read(self):
        if self._stamp is None or time.monotonic() - self._stamp > self.max_age:
            raise RuntimeError(f'xArm F/T reports stale (>{self.max_age * 1000:.0f} ms); stopping')
        wrench = np.asarray(self._api.ft_ext_force, dtype=float)
        if wrench.shape != (6,) or not np.isfinite(wrench).all():
            raise RuntimeError('Invalid F/T sensor reading')
        return wrench

    def close(self):
        if self._api is not None:
            try:
                self._api.release_report_callback(self._on_report)
            finally:
                try:
                    self._api.disconnect()
                finally:
                    self._api = None


class _Trace:
    """Debug mode: every control tick to a CSV, a console line every 0.1 s, and the baselines. Off: no-ops."""
    FIELDS = ('t_s', 'phase', 'x_mm', 'y_mm', 'z_mm', 'target_x_mm', 'target_y_mm', 'target_z_mm',
              'fx_n', 'fy_n', 'fz_n', 'tx_nm', 'ty_nm', 'tz_nm', 'metric_n', 'hits')

    def __init__(self, enabled, directory):
        self.enabled, self._file, self._t0, self._last_print = enabled, None, None, -math.inf
        if enabled:
            folder = Path(directory or DEFAULT_DEBUG_DIR).expanduser()
            folder.mkdir(parents=True, exist_ok=True)
            self.path = folder / f'base_placement_{datetime.now():%Y%m%d_%H%M%S}.csv'
            self._file = self.path.open('w')
            self._file.write(','.join(self.FIELDS) + '\n')
            self.note(f'logging every tick to {self.path}')

    def note(self, text):
        if self.enabled:
            print(f'[base debug] {text}', flush=True)

    def row(self, now, phase, measured, target, wrench, metric=math.nan, hits=0):
        if not self.enabled:
            return
        self._t0 = now if self._t0 is None else self._t0
        values = [f'{now - self._t0:.3f}', phase, *(f'{v * 1000:.3f}' for v in measured[:3]),
                  *(f'{v * 1000:.3f}' for v in target[:3]), *(f'{v:.4f}' for v in wrench),
                  f'{metric:.4f}', str(hits)]
        self._file.write(','.join(values) + '\n')
        if now - self._last_print >= .1:
            self._last_print = now
            self.note(f'{phase:<8} xyz_mm={np.round(measured[:3] * 1000, 2).tolist()} '
                      f'F_N={np.round(wrench[:3], 3).tolist()} metric={metric:.3f} N hits={hits}')

    def close(self):
        if self._file is not None:
            self._file.close()
            self._file = None


def run_base_placement(config, placement, execute=False, stop=None, ft_factory=None):
    """Seat the plate on the base deck by touch, starting from the fixed pose `placement.pose_mm_deg`.

    1. Move to the probing start `placement.pose_mm_deg` (measured arrival). From higher than approach_clearance_mm
       above it: at approach_speed_mm_s to that clearance straight above it first, then down at speed_mm_s. Both
       moves speed up and slow down at accel_mm_s2.
    2. Hold still, take the in-air F/T baseline, descend (probe_fast_mm at fast_speed_mm_s, then probe_speed_mm_s)
       until |dFz| > touch_force_n.
    3. Hold at the touch height, re-baseline, slide along base -X (slide_fast_mm at fast_speed_mm_s, then
       slide_speed_mm_s) until the sideways force |(dFx, dFy)| > ridge_force_n: the plate edge is against the
       ridge. Then step back to the measured X (no push left on the ridge) and finish; the node then releases and
       lifts.

    No touch within probe_depth_mm below the start, or no ridge within slide_max_mm, fails the run.
    Prediction only reads the initial TCP and logs the plan. No camera, policy inference, or gripper commands.
    `ft_factory` replaces the F/T report connection (tests).
    """
    stop = stop if stop is not None else threading.Event()
    if stop.is_set():
        return 'stopped'
    motion = replace(config.motion, translation_speed=placement.speed_mm_s / 1000,
                     rotation_speed=math.radians(placement.rotation_speed_deg_s))
    approach = replace(motion, translation_speed=max(placement.approach_speed_mm_s, placement.speed_mm_s) / 1000)
    accel = placement.accel_mm_s2 / 1000
    # The servo cap is the fastest move. 'real' reports: the measured pose is <=10 ms old, not up to 200 ms.
    robot = XArm6Robot(config.robot_ip, approach, read_only=not execute, report_type='real')
    ft = trace = None
    try:
        robot.connect()
        initial = robot.initialize_target_pose()
        start = placement.target_pose()
        start[3:] = continuous_rotvec(start[3:], initial[3:])
        above = start.copy()
        above[2] += placement.approach_clearance_mm / 1000
        from_above = initial[2] > above[2]
        if stop.is_set():
            return 'stopped'
        print(f'Base placement {"EXECUTE" if execute else "PREDICTION ONLY"}: '
              f'start_mm_rpy_deg={list(placement.pose_mm_deg)} at {placement.speed_mm_s} mm/s'
              + (f' ({placement.approach_speed_mm_s} mm/s to {placement.approach_clearance_mm:g} mm above it)'
                 if from_above else '') + f', {placement.accel_mm_s2:g} mm/s^2 ramps, '
              f'descend ({placement.probe_fast_mm:g} mm at {placement.fast_speed_mm_s:g} mm/s, then '
              f'{placement.probe_speed_mm_s:g} mm/s) until |dFz| > {placement.touch_force_n:g} N, '
              f'slide -X ({placement.slide_fast_mm:g} mm at {placement.fast_speed_mm_s:g} mm/s, then '
              f'{placement.slide_speed_mm_s:g} mm/s) until |dFxy| > {placement.ridge_force_n:g} N'
              f'{" (debug)" if placement.debug else ""}', flush=True)
        if not execute:
            return 'done'
        trace = _Trace(placement.debug, placement.debug_dir)
        robot.prepare_force_torque(enable=True)   # sampling on before enable(); restored by close()
        ft = ForceReader(config.robot_ip, ft_factory)
        ft.connect()
        robot.enable()
        deadline = time.monotonic() + placement.timeout_sec
        if from_above:
            result = _move_goals(robot, initial, (above,), approach, stop, placement.timeout_sec, 'Base placement',
                                 accel=accel, arrival_sec=BASE_ARRIVAL_SEC)
            if result != 'done':
                return result
            initial = above   # the commanded target the arm now holds
        result = _move_goals(robot, initial, (start,), motion, stop, max(0., deadline - time.monotonic()),
                             'Base placement', accel=accel, arrival_sec=BASE_ARRIVAL_SEC)
        if result != 'done':
            return result
        return _seat(robot, ft, start, placement, motion, stop, deadline, trace)
    finally:
        try:
            if ft is not None:
                ft.close()
        finally:
            try:
                robot.close()
            finally:
                if trace is not None:
                    trace.close()


def _seat(robot, ft, start, placement, motion, stop, deadline, trace):
    """Steps 2-3 of run_base_placement, starting held at `start`."""
    target = start.copy()
    period = 1 / motion.loop_hz
    ctx = SimpleNamespace(robot=robot, ft=ft, placement=placement, period=period, stop=stop, deadline=deadline,
                          trace=trace, rate=ControlRate(period, time.monotonic, stop.wait))
    hold = placement.settle_sec + placement.baseline_sec

    held = _hold(ctx, target, target, placement.probe_speed_mm_s, hold, 'baseline')
    if held is None:
        return 'stopped'
    target, air = held
    trace.note(f'in-air baseline F_N={np.round(air, 4).tolist()}')
    touch = lambda wrench: abs(wrench[2] - air[2])
    descent = placement.probe_depth_mm / 1000
    found = _guarded_move(ctx, target, 2, -1, placement.probe_speed_mm_s, descent, touch,
                          placement.touch_force_n, 'descend', placement.fast_speed_mm_s, placement.probe_fast_mm / 1000)
    if found is None:
        return 'stopped'
    target, measured, value = found
    print(f'Base placement: touch at z={measured[2] * 1000:.2f} mm '
          f'({(start[2] - measured[2]) * 1000:.2f} mm below the start), |dFz|={value:.2f} N', flush=True)

    seated = target.copy()
    seated[2] = measured[2]   # hold at the touch height, not where the lagging target pressed to
    held = _hold(ctx, target, seated, placement.probe_speed_mm_s, hold, 'contact')
    if held is None:
        return 'stopped'
    target, rest = held
    trace.note(f'contact baseline F_N={np.round(rest, 4).tolist()}')
    ridge = lambda wrench: math.hypot(wrench[0] - rest[0], wrench[1] - rest[1])
    found = _guarded_move(ctx, target, 0, -1, placement.slide_speed_mm_s, placement.slide_max_mm / 1000, ridge,
                          placement.ridge_force_n, 'slide', placement.fast_speed_mm_s, placement.slide_fast_mm / 1000)
    if found is None:
        return 'stopped'
    target, measured, value = found
    print(f'Base placement: ridge at x={measured[0] * 1000:.2f} mm '
          f'({(start[0] - measured[0]) * 1000:.2f} mm along -X from the start), |dFxy|={value:.2f} N', flush=True)

    final = target.copy()
    final[0] = measured[0]    # take the push off the ridge
    held = _hold(ctx, target, final, placement.slide_speed_mm_s, placement.settle_sec, 'release')
    if held is None:
        return 'stopped'
    trace.note(f'final F_N={np.round(held[1], 4).tolist()}')
    return 'done'


def _hold(ctx, target, goal, speed_mm_s, seconds, phase):
    """Servo the target to `goal`'s XYZ at `speed_mm_s`, then keep it there until `seconds` have passed.

    Returns (target, median F/T over the last placement.baseline_sec), or None if stopped.
    """
    target, goal = target.copy(), np.asarray(goal, dtype=float)
    step = speed_mm_s / 1000 * ctx.period
    started = time.monotonic()
    samples = []
    while True:
        now = time.monotonic()
        if ctx.stop.is_set():
            return None
        if now >= ctx.deadline:
            raise RuntimeError('Base placement timed out')
        measured, _ = ctx.robot.observation()
        wrench = ctx.ft.read()
        ctx.trace.row(now, phase, measured, target, wrench)
        remaining = goal[:3] - target[:3]
        at_goal = np.linalg.norm(remaining) < 1e-9
        if at_goal and now - started >= seconds - ctx.placement.baseline_sec:
            samples.append(wrench)
        if at_goal and now - started >= seconds and samples:
            break
        if not at_goal:
            target[:3] += remaining * min(1., step / np.linalg.norm(remaining))
        ctx.robot.servo(target, ctx.period, measured=measured)
        ctx.rate.wait()
    samples = np.asarray(samples)
    if len(samples) > 1 and not np.ptp(samples[:, :3], axis=0).any():
        raise RuntimeError('F/T force readings did not change while holding; is the sensor reporting?')
    return target, np.median(samples, axis=0)


def _guarded_move(ctx, target, axis, direction, speed_mm_s, travel, metric, threshold, phase,
                  fast_mm_s=None, fast_travel=0.):
    """Step the target along base axis `axis` (0 X, 2 Z) by `direction` at `speed_mm_s` for at most `travel` m,
    until `metric(wrench)` exceeds `threshold` for placement.force_count consecutive ticks. The first
    `fast_travel` m go at `fast_mm_s` instead, with the same force check.

    Returns (target, measured pose, metric) at detection, or None if stopped. Reaching the end of travel (and
    0.5 s for the TCP to catch up) without contact fails.
    """
    target = target.copy()
    origin = target[axis]
    end = origin + direction * travel
    step = speed_mm_s / 1000 * ctx.period
    fast_step = (fast_mm_s or speed_mm_s) / 1000 * ctx.period
    hits, end_since = 0, None
    while True:
        now = time.monotonic()
        if ctx.stop.is_set():
            return None
        if now >= ctx.deadline:
            raise RuntimeError('Base placement timed out')
        measured, _ = ctx.robot.observation()
        wrench = ctx.ft.read()
        value = float(metric(wrench))
        hits = hits + 1 if value > threshold else 0
        ctx.trace.row(now, phase, measured, target, wrench, value, hits)
        if hits >= ctx.placement.force_count:
            return target, measured, value
        remaining = (end - target[axis]) * direction
        if remaining <= 1e-9:
            end_since = now if end_since is None else end_since
            if now - end_since >= .5:
                raise RuntimeError(f'Base placement: no contact during {phase} within {travel * 1000:.1f} mm '
                                   f'(last {value:.2f} N, threshold {threshold:g} N)')
        else:
            fast_left = fast_travel - (target[axis] - origin) * direction
            this_step = min(fast_step, fast_left) if fast_left > 1e-9 else step
            target[axis] += direction * min(this_step, remaining)
        ctx.robot.servo(target, ctx.period, measured=measured)
        ctx.rate.wait()


BASE_POLICY_START_MM_DEG = (32.055321, 271.166351, 116.803917, -179.675382, -1.57403, -0.737282)


def move_to_base_policy_start(config, placement, execute=False, stop=None, max_distance_mm=200.0):
    """One straight move to `placement.pose_mm_deg` (the base policy's start pose, ~80 mm above the bracket), ending
    after measured arrival (1 mm / 0.5 degrees for 0.3 s). Uses the base-placement speed and rotation limits.

    Refuses to move from more than `max_distance_mm` away (place_base starts it from home_offset, ~113 mm away).
    Prediction only reads the TCP and logs the destination.
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
        distance_mm = float(np.linalg.norm(goal[:3] - initial[:3])) * 1000
        if stop.is_set():
            return 'stopped'
        print(f'Base policy start {"EXECUTE" if execute else "PREDICTION ONLY"}: '
              f'goal_mm_rpy_deg={list(placement.pose_mm_deg)}, {distance_mm:.1f} mm away, '
              f'speed={placement.speed_mm_s} mm/s, rotation={placement.rotation_speed_deg_s} deg/s', flush=True)
        if distance_mm > max_distance_mm:
            raise RuntimeError(f'Base policy start pose is {distance_mm:.0f} mm away (limit {max_distance_mm:.0f} mm); '
                               'start place_base from home_offset')
        if not execute:
            return 'done'
        robot.enable()
        return _move_goals(robot, initial, (goal,), motion, stop, placement.timeout_sec, 'Base policy start')
    finally:
        robot.close()


def _move_goals(robot, initial, goals, motion, stop, timeout_sec, label, hold_xyz_tolerance=None, accel=None,
                arrival_sec=.3):
    """Follow ordered waypoints, requiring measured arrival (1 mm / 0.5 degrees for `arrival_sec`) before advancing.

    `accel` (m/s^2): the translation speeds up from rest at this rate and slows down to stop at each waypoint,
    up to motion.translation_speed. None: full speed at once (rotation is never ramped)."""
    target = initial.copy()
    period = 1 / motion.loop_hz
    radius = max(float(np.linalg.norm(goal[:3] - initial[:3])) for goal in goals) + 1e-6
    deadline = time.monotonic() + timeout_sec
    rate = ControlRate(period, time.monotonic, stop.wait)
    for goal in goals:
        arrived_since = None
        speed = 0.
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
                if tick - arrived_since >= arrival_sec:
                    break
            else:
                arrived_since = None
            step_motion = motion
            if accel is not None:
                remaining = float(np.linalg.norm(goal[:3] - target[:3]))
                speed = min(motion.translation_speed, speed + accel * period, math.sqrt(2 * accel * remaining))
                step_motion = replace(motion, translation_speed=min(motion.translation_speed,
                                                                    max(speed, accel * period)))
            target = limit_action(target, goal, step_motion, period, initial, True, radius)
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


def jog_base_xyz(config, offset_m, speed_mm_s, execute=False, stop=None):
    """Move the TCP by `offset_m` (x, y, z metres, xArm base frame) in a straight line at `speed_mm_s`,
    holding its orientation."""
    stop = stop if stop is not None else threading.Event()
    offset = np.asarray(offset_m, dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError('Jog offset must be a finite (x, y, z) in metres')
    if not math.isfinite(speed_mm_s) or speed_mm_s <= 0:
        raise ValueError('Jog speed must be positive')
    if stop.is_set():
        return 'stopped'
    motion = replace(config.motion, translation_speed=speed_mm_s / 1000)
    robot = XArm6Robot(config.robot_ip, motion, read_only=not execute)
    try:
        robot.connect()
        initial = robot.initialize_target_pose()
        goal = initial.copy()
        goal[:3] += offset
        if stop.is_set():
            return 'stopped'
        print(f'Jog {"EXECUTE" if execute else "PREDICTION ONLY"}: base-frame offset mm='
              f'{(offset * 1000).round(1).tolist()} at {speed_mm_s:g} mm/s; '
              f'TCP mm={(initial[:3] * 1000).round(3).tolist()} -> {(goal[:3] * 1000).round(3).tolist()}', flush=True)
        if not execute:
            return 'done'
        robot.enable()
        timeout = float(np.linalg.norm(offset)) * 1000 / speed_mm_s + 10.0
        return _move_goals(robot, initial, (goal,), motion, stop, timeout, 'Jog')
    finally:
        robot.close()


def _turn_rz(rotvec, angle_rad):
    """Orientation `rotvec` turned by `angle_rad` about the xArm base-frame z axis (RZ)."""
    return continuous_rotvec((Rotation.from_euler('z', angle_rad) * Rotation.from_rotvec(rotvec)).as_rotvec(), rotvec)


def jog_base_rz(config, angle_deg, speed_deg_s, execute=False, stop=None):
    """Turn the TCP by `angle_deg` about the xArm base-frame z axis (RZ) at `speed_deg_s`, holding its XYZ."""
    stop = stop if stop is not None else threading.Event()
    if not math.isfinite(angle_deg) or abs(angle_deg) >= 180:
        raise ValueError('Jog angle must be finite and below 180 degrees')
    if not math.isfinite(speed_deg_s) or speed_deg_s <= 0:
        raise ValueError('Jog rotation speed must be positive')
    if stop.is_set():
        return 'stopped'
    motion = replace(config.motion, rotation_speed=math.radians(speed_deg_s))
    robot = XArm6Robot(config.robot_ip, motion, read_only=not execute)
    try:
        robot.connect()
        initial = robot.initialize_target_pose()
        goal = initial.copy()
        goal[3:] = _turn_rz(initial[3:], math.radians(angle_deg))
        if stop.is_set():
            return 'stopped'
        rpy = lambda pose: Rotation.from_rotvec(pose[3:]).as_euler('xyz', degrees=True).round(2).tolist()
        print(f'Jog {"EXECUTE" if execute else "PREDICTION ONLY"}: base-frame RZ {angle_deg:+g} deg at '
              f'{speed_deg_s:g} deg/s; TCP RPY deg={rpy(initial)} -> {rpy(goal)}', flush=True)
        if not execute:
            return 'done'
        robot.enable()
        timeout = abs(angle_deg) / speed_deg_s + 10.0
        return _move_goals(robot, initial, (goal,), motion, stop, timeout, 'Jog')
    finally:
        robot.close()


# The planner's plan_home_offset joint goal (global_planner move_to_pose_node.cpp), radians.
HOME_OFFSET_JOINTS = (1.6284, -0.3927, -0.5812, 0.0, 0.9738, 1.6284)
TELEOP_HOME = 'home'   # teleop command: drive straight to the home pose


def _pose_text(pose):
    rpy = Rotation.from_rotvec(pose[3:]).as_euler('xyz', degrees=True)
    return f'mm={(pose[:3] * 1000).round(1).tolist()} RPY deg={rpy.round(2).tolist()}'


def move_home(config, joints, speed_mm_s, rotation_speed_deg_s, execute=False, stop=None):
    """Move the TCP straight to its pose at `joints` (rad, e.g. HOME_OFFSET_JOINTS; the controller's forward
    kinematics) at up to `speed_mm_s` and `rotation_speed_deg_s`, the controller's IK following it. No planning:
    nothing on the way is avoided."""
    stop = stop if stop is not None else threading.Event()
    joints = np.asarray(joints, dtype=float)
    if joints.shape != (6,) or not np.isfinite(joints).all():
        raise ValueError('Home joints must be six finite angles in radians')
    if not math.isfinite(speed_mm_s) or speed_mm_s <= 0:
        raise ValueError('Home speed must be positive')
    if not math.isfinite(rotation_speed_deg_s) or rotation_speed_deg_s <= 0:
        raise ValueError('Home rotation speed must be positive')
    if stop.is_set():
        return 'stopped'
    motion = replace(config.motion, translation_speed=speed_mm_s / 1000,
                     rotation_speed=math.radians(rotation_speed_deg_s))
    robot = XArm6Robot(config.robot_ip, motion, read_only=not execute)
    try:
        robot.connect()
        initial = robot.initialize_target_pose()
        goal = robot.forward_kinematics(joints)
        goal[3:] = continuous_rotvec(goal[3:], initial[3:])
        distance = float(np.linalg.norm(goal[:3] - initial[:3]))
        angle = math.degrees((Rotation.from_rotvec(goal[3:]) * Rotation.from_rotvec(initial[3:]).inv()).magnitude())
        if stop.is_set():
            return 'stopped'
        print(f'Home {"EXECUTE" if execute else "PREDICTION ONLY"}: straight from TCP {_pose_text(initial)} to '
              f'{_pose_text(goal)} ({distance * 1000:.0f} mm, {angle:.0f} deg) at {speed_mm_s:g} mm/s, '
              f'{rotation_speed_deg_s:g} deg/s', flush=True)
        if not execute:
            return 'done'
        robot.enable()
        timeout = distance * 1000 / speed_mm_s + angle / rotation_speed_deg_s + 10.0
        return _move_goals(robot, initial, (goal,), motion, stop, timeout, 'Home')
    finally:
        robot.close()


def clear_arm_errors(config, execute=False, stop=None):
    """Clear the xArm's error and warning and enable it again (motors on, servo mode 1, state 0) without commanding
    a pose: the arm holds where it is. execute=False only reads the codes."""
    stop = stop if stop is not None else threading.Event()
    if stop.is_set():
        return 'stopped'
    robot = XArm6Robot(config.robot_ip, config.motion, read_only=not execute)
    try:
        robot.connect(check_health=False)   # a faulted arm is what this is for
        error, warning, state = robot.error_state()
        print(f'Clear errors {"EXECUTE" if execute else "PREDICTION ONLY"}: xArm error C{error}, warning {warning}, '
              f'state {state}', flush=True)
        if stop.is_set():
            return 'stopped'
        if not execute:
            return 'done'
        robot.clear_errors()
        error, warning, state = robot.error_state()
        print(f'Cleared: xArm error C{error}, warning {warning}, state {state}; motors enabled in servo mode 1',
              flush=True)
        return 'done'
    finally:
        robot.close()


def teleop_base_xyz(config, command, speed_mm_s, radius_m, execute=False, stop=None, rotation_speed_deg_s=30.0,
                    home_joints=None):
    """Drive the TCP in the xArm base frame from a live command (the gamepad).

    Every control tick `command()` returns (x, y, z, rz) velocity fractions in [-1, 1], TELEOP_HOME, or None to
    end: x, y, z move the TCP at up to `speed_mm_s`, rz turns it about the base z axis (XYZ held) at up to
    `rotation_speed_deg_s`; TELEOP_HOME drives it straight to its pose at `home_joints` (see move_home) at those
    speeds. The XYZ target stays within `radius_m` of the start pose, or of where the last home move left it. Once
    the command ends, the arm settles on the last target and this returns 'done'; `stop` returns 'stopped'."""
    stop = stop if stop is not None else threading.Event()
    if not math.isfinite(speed_mm_s) or speed_mm_s <= 0:
        raise ValueError('Teleop speed must be positive')
    if not math.isfinite(rotation_speed_deg_s) or rotation_speed_deg_s <= 0:
        raise ValueError('Teleop rotation speed must be positive')
    if not math.isfinite(radius_m) or radius_m <= 0:
        raise ValueError('Teleop radius must be positive')
    if stop.is_set():
        return 'stopped'
    motion = replace(config.motion, translation_speed=speed_mm_s / 1000,
                     rotation_speed=math.radians(rotation_speed_deg_s))
    period = 1 / motion.loop_hz
    robot = XArm6Robot(config.robot_ip, motion, read_only=not execute)
    try:
        robot.connect()
        initial = robot.initialize_target_pose()
        home = robot.forward_kinematics(home_joints) if home_joints is not None else None
        print(f'Arm teleop {"EXECUTE" if execute else "PREDICTION ONLY"}: base-frame XYZ at up to {speed_mm_s:g} mm/s '
              f'within {radius_m * 1000:g} mm of TCP mm={(initial[:3] * 1000).round(3).tolist()}, '
              f'RZ at up to {rotation_speed_deg_s:g} deg/s'
              + (f', home at TCP {_pose_text(home)}' if home is not None else ''), flush=True)
        rate = ControlRate(period, time.monotonic, stop.wait)
        if not execute:
            while not stop.is_set() and command() is not None:
                rate.wait()
            return 'stopped' if stop.is_set() else 'done'
        robot.enable()
        target = initial.copy()
        anchor = initial.copy()   # the centre of the radius_m sphere
        while True:
            if stop.is_set():
                return 'stopped'
            measured, _ = robot.observation()
            velocity = command()
            if velocity is None:
                break
            if isinstance(velocity, str):
                if velocity != TELEOP_HOME or home is None:
                    raise ValueError(f'Unknown teleop command {velocity!r}')
                goal = home.copy()
                goal[3:] = continuous_rotvec(home[3:], target[3:])
                target = limit_action(target, goal, motion, period, target, True, math.inf)
                anchor = target.copy()
            else:
                velocity = np.asarray(velocity, dtype=float)
                if velocity.shape != (4,) or not np.isfinite(velocity).all():
                    raise ValueError('Teleop command must be a finite (x, y, z, rz)')
                velocity = np.clip(velocity, -1., 1.)
                goal = target.copy()
                goal[:3] += velocity[:3] * motion.translation_speed * period
                offset = goal[:3] - anchor[:3]
                distance = float(np.linalg.norm(offset))
                if distance > radius_m:
                    goal[:3] = anchor[:3] + offset * (radius_m / distance)
                goal[3:] = _turn_rz(target[3:], velocity[3] * motion.rotation_speed * period)
                target = limit_action(target, goal, motion, period, anchor, True, radius_m + 1e-6)
            if stop.is_set():
                return 'stopped'
            robot.servo(target, period, measured=measured)
            rate.wait()
        # Hold the last target until the measured TCP has arrived.
        return _move_goals(robot, target, (target,), motion, stop, 5.0, 'Arm teleop')
    finally:
        robot.close()
