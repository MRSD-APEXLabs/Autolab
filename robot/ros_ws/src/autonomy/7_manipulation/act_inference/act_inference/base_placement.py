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


@dataclass(frozen=True)
class BasePlacementConfig:
    pose_mm_deg: tuple = (51.565662, 257.200928, 47.687004, 179.332982, -2.176667, 1.803213)   # probing start
    speed_mm_s: float = 15.0
    rotation_speed_deg_s: float = 6.0
    timeout_sec: float = 180.0
    # F/T seating (run_base_placement): from pose_mm_deg, descend until |Fz - in-air baseline| > touch_force_n,
    # then slide along base -X until the sideways force |(Fx, Fy) - baseline after the touch| > ridge_force_n
    # (the plate edge meets the ridge).
    probe_speed_mm_s: float = 2.0
    probe_depth_mm: float = 11.0          # no touch this far below the start fails the run
    touch_force_n: float = 1.0
    slide_speed_mm_s: float = 2.0
    slide_max_mm: float = 30.0            # no ridge after this much -X travel fails the run
    ridge_force_n: float = 1.0
    force_count: int = 2                  # consecutive 50 Hz reads above a threshold that count as contact
    settle_sec: float = 0.5               # still before each F/T baseline
    baseline_sec: float = 0.3             # F/T baseline: median over this window
    debug: bool = False                   # per-tick CSV + 10 Hz console trace of pose and force
    debug_dir: str = DEFAULT_DEBUG_DIR

    def __post_init__(self):
        pose = np.asarray(self.pose_mm_deg, dtype=float)
        if pose.shape != (6,) or not np.isfinite(pose).all():
            raise ValueError('base_pose_mm_deg must contain six finite XYZ/RPY values')
        for name in ('speed_mm_s', 'rotation_speed_deg_s', 'timeout_sec', 'probe_speed_mm_s', 'probe_depth_mm',
                     'touch_force_n', 'slide_speed_mm_s', 'slide_max_mm', 'ridge_force_n',
                     'settle_sec', 'baseline_sec'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'base {name} must be finite and positive')
        if self.probe_speed_mm_s > self.speed_mm_s or self.slide_speed_mm_s > self.speed_mm_s:
            raise ValueError('base probe/slide speeds must not exceed speed_mm_s')
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

    1. Move to the probing start `placement.pose_mm_deg` (measured arrival).
    2. Hold still, take the in-air F/T baseline, descend at probe_speed_mm_s until |dFz| > touch_force_n.
    3. Hold at the touch height, re-baseline, slide along base -X at slide_speed_mm_s until the sideways force
       |(dFx, dFy)| > ridge_force_n: the plate edge is against the ridge. Then step back to the measured X
       (no push left on the ridge) and finish; the node then releases and lifts.

    No touch within probe_depth_mm below the start, or no ridge within slide_max_mm, fails the run.
    Prediction only reads the initial TCP and logs the plan. No camera, policy inference, or gripper commands.
    `ft_factory` replaces the F/T report connection (tests).
    """
    stop = stop if stop is not None else threading.Event()
    if stop.is_set():
        return 'stopped'
    motion = replace(config.motion, translation_speed=placement.speed_mm_s / 1000,
                     rotation_speed=math.radians(placement.rotation_speed_deg_s))
    robot = XArm6Robot(config.robot_ip, motion, read_only=not execute)
    ft = trace = None
    try:
        robot.connect()
        initial = robot.initialize_target_pose()
        start = placement.target_pose()
        start[3:] = continuous_rotvec(start[3:], initial[3:])
        if stop.is_set():
            return 'stopped'
        print(f'Base placement {"EXECUTE" if execute else "PREDICTION ONLY"}: '
              f'start_mm_rpy_deg={list(placement.pose_mm_deg)} at {placement.speed_mm_s} mm/s, '
              f'descend until |dFz| > {placement.touch_force_n:g} N, '
              f'slide -X until |dFxy| > {placement.ridge_force_n:g} N'
              f'{" (debug)" if placement.debug else ""}', flush=True)
        if not execute:
            return 'done'
        trace = _Trace(placement.debug, placement.debug_dir)
        robot.prepare_force_torque(enable=True)   # sampling on before enable(); restored by close()
        ft = ForceReader(config.robot_ip, ft_factory)
        ft.connect()
        robot.enable()
        deadline = time.monotonic() + placement.timeout_sec
        result = _move_goals(robot, initial, (start,), motion, stop, placement.timeout_sec, 'Base placement')
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
                          placement.touch_force_n, 'descend')
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
                          placement.ridge_force_n, 'slide')
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


def _guarded_move(ctx, target, axis, direction, speed_mm_s, travel, metric, threshold, phase):
    """Step the target along base axis `axis` (0 X, 2 Z) by `direction` at `speed_mm_s` for at most `travel` m,
    until `metric(wrench)` exceeds `threshold` for placement.force_count consecutive ticks.

    Returns (target, measured pose, metric) at detection, or None if stopped. Reaching the end of travel (and
    0.5 s for the TCP to catch up) without contact fails.
    """
    target = target.copy()
    end = target[axis] + direction * travel
    step = speed_mm_s / 1000 * ctx.period
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
            target[axis] += direction * min(step, remaining)
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
