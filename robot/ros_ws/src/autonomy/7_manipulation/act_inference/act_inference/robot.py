"""xArm boundary: SI units and rotation vectors everywhere outside this module.

Copied from teleop/robot.py. With read_only=True (every run without --execute) only
state is read; enable() and servo() refuse to run.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def continuous_rotvec(vector, reference):
    v = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(v)
    if norm < 1e-10:
        return v
    axis = v / norm
    k = round(float(np.dot(reference - v, axis)) / (2 * np.pi))
    return min((v + (k + j) * 2 * np.pi * axis for j in (-1, 0, 1)),
               key=lambda x: np.linalg.norm(x - reference))


class XArm6Robot:
    def __init__(self, robot_ip, motion, dry_run=False, api_factory=None, read_only=False):
        self.robot_ip, self.motion, self.dry_run = robot_ip, motion, dry_run
        self._factory, self._arm = api_factory, None
        self.read_only = read_only
        self._target = np.zeros(6)
        self._reference = None
        self._enabled = False

    @staticmethod
    def _check(code, operation):
        if code != 0:
            raise RuntimeError(f'xArm {operation} failed: code {code}')

    def connect(self):
        if self.dry_run:
            return
        if self._factory is None:
            from xarm.wrapper import XArmAPI
            self._factory = XArmAPI
        try:
            self._arm = self._factory(self.robot_ip, is_radian=True)
        except Exception as exc:   # the SDK raises a bare Exception when the socket does not connect
            raise RuntimeError(f'xArm connection to {self.robot_ip} failed: {exc}') from exc
        if not self._arm.connected:
            raise RuntimeError('xArm connection failed')
        if self._arm.axis != 6:
            raise RuntimeError('Expected a six-axis xArm')
        self.check_health()

    def check_health(self):
        if self.dry_run:
            return
        arm = self._arm
        if not arm.connected:
            raise RuntimeError('xArm disconnected')
        code, errors = arm.get_err_warn_code()
        self._check(code, 'get_err_warn_code')
        code, state = arm.get_state()
        self._check(code, 'get_state')
        # Controller states (the SDK's wait_move logic): 0 motion state (just after set_state(0)),
        # 1 in motion, 2 standby ("sleeping"), 3 paused, 4 stopped, 5 mode/configuration changed, 6 deceleration stop.
        if errors[0]:
            raise RuntimeError(f'xArm fault: errors={errors}, state={state}; clear it in xArm Studio')
        if self.read_only:
            return  # pose monitoring while stopped does not require motion readiness
        if self._enabled:
            if state not in (0, 1, 2):
                raise RuntimeError(f'xArm left the running state: errors={errors}, state={state}')
        elif state == 1 or state == 3:
            raise RuntimeError(f'xArm is {"in motion" if state == 1 else "paused"} (state={state}); '
                               'stop the running program in xArm Studio before teleop')
        elif state not in (0, 2) and not (state in (4, 5) and not errors[1]):
            raise RuntimeError(f'xArm is not ready: errors={errors}, state={state}; prepare it in xArm Studio')
        # Clean states 4 (software stop) and 5 (mode/configuration changed) need
        # set_state(0). Only enable() re-arms them after an explicit session start;
        # a transition to either state during execution remains a stop condition.

    def observation(self):
        if self.dry_run:
            return self._target.copy(), np.zeros(6)
        self.check_health()
        code, pose = self._arm.get_position_aa(is_radian=True)
        self._check(code, 'get_position_aa')
        code, joints = self._arm.get_servo_angle(is_radian=True)
        self._check(code, 'get_servo_angle')
        pose = np.asarray(pose, dtype=float)
        if pose.shape != (6,) or not np.isfinite(pose).all():
            raise RuntimeError('Invalid TCP feedback from xArm')
        if len(joints) < 6 or not np.isfinite(joints[:6]).all():
            raise RuntimeError('Invalid joint feedback from xArm')
        pose[:3] /= 1000
        if self._reference is not None:
            pose[3:] = continuous_rotvec(pose[3:], self._reference)
        self._reference = pose[3:].copy()
        return pose, np.asarray(joints[:6], dtype=float)

    def initialize_target_pose(self):
        self._target = self.observation()[0].copy()
        return self._target.copy()

    def enable(self):
        if self.read_only:
            raise RuntimeError('Read-only robot cannot be enabled')
        if self.dry_run:
            return
        self.check_health()
        # Mark ownership before enabling, so partial failures still trigger stop.
        self._enabled = True
        for operation, args in [('motion_enable', (True,)), ('set_mode', (1,)), ('set_state', (0,))]:
            self._check(getattr(self._arm, operation)(*args), operation)
        self.check_health()

    def servo(self, target, period, measured=None):
        if self.read_only:
            raise RuntimeError('Read-only robot cannot receive motion commands')
        if not np.isfinite(period) or period <= 0:
            raise ValueError('Control period must be finite and positive')
        target = np.asarray(target, dtype=float)
        if target.shape != (6,) or not np.isfinite(target).all():
            raise ValueError('Target must be a finite six-dimensional pose')
        distance = np.linalg.norm(target[:3] - self._target[:3])
        angle = (Rotation.from_rotvec(target[3:]) * Rotation.from_rotvec(self._target[3:]).inv()).magnitude()
        if distance > self.motion.translation_speed * period + 1e-8 or angle > self.motion.rotation_speed * period + 1e-8:
            raise RuntimeError('Target exceeds configured speed limit')
        if measured is not None and np.linalg.norm(target[:3] - measured[:3]) > self.motion.max_tracking_error:
            raise RuntimeError('TCP tracking error exceeds limit')
        if not self.dry_run:
            self.check_health()
            if not self._enabled:
                raise RuntimeError('Robot is not enabled for teleop')
            sdk_pose = target.copy()
            sdk_pose[:3] *= 1000
            self._check(self._arm.set_servo_cartesian_aa(sdk_pose.tolist(), is_radian=True,
                                                       is_tool_coord=False, relative=False), 'servo')
        self._target = target.copy()

    def close(self):
        if self._arm is not None:
            try:
                if self._enabled:
                    self._check(self._arm.set_state(4), 'stop')
            finally:
                self._arm.disconnect()
                self._arm = None
