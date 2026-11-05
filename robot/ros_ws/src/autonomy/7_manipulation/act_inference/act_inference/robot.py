"""xArm boundary: SI units and rotation vectors everywhere outside this module.

Copied from teleop/robot.py. With read_only=True (every run without --execute) only
state is read; enable() and servo() refuse to run.
"""

import logging
import threading
import time

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
        self._ft_started_by_us = False
        self.force_torque_config = None
        self._report_lock = threading.Lock()
        self._report_event = threading.Event()
        self._report = None
        self._reporting = False
        self._last_servo_time = None
        self._last_timing_warning = float('-inf')

    @staticmethod
    def _check(code, operation):
        if code != 0:
            raise RuntimeError(f'xArm {operation} failed: code {code}')

    def connect(self):
        if self.dry_run:
            return
        if self._arm is not None and self._arm.connected:
            return
        if self._factory is None:
            from xarm.wrapper import XArmAPI
            self._factory = XArmAPI
        try:
            self._arm = self._factory(self.robot_ip, is_radian=True, enable_report=True, report_type='rich')
        except Exception as exc:   # the SDK raises a bare Exception when the socket does not connect
            raise RuntimeError(f'xArm connection to {self.robot_ip} failed: {exc}') from exc
        if not self._arm.connected:
            raise RuntimeError('xArm connection failed')
        if self._arm.axis != 6:
            raise RuntimeError('Expected a six-axis xArm')
        self._report_event.clear()
        with self._report_lock:
            self._report = None
        self.check_health()
        # The SDK already receives pose, joints and health on its report socket. Polling
        # six getters per 20 ms tick starves the servo command socket with round trips.
        if not self._arm.register_report_callback(self._on_report, report_mtable=False,
                                                  report_mtbrake=False, report_cmd_num=False):
            raise RuntimeError('Could not register xArm feedback reports')
        self._reporting = True
        if not self._report_event.wait(2.0):
            raise RuntimeError('No xArm feedback report received')

    def _on_report(self, data):
        snapshot = dict(data)
        for key in ('cartesian', 'joints'):
            snapshot[key] = tuple(data[key])
        with self._report_lock:
            self._report = (time.monotonic(), snapshot)
        self._report_event.set()

    def _feedback(self):
        with self._report_lock:
            report = self._report
        if report is None or time.monotonic() - report[0] > .25:
            raise RuntimeError('xArm feedback reports stale (>250 ms); stopping')
        return report

    def check_health(self, poll=False):
        if self.dry_run:
            return
        arm = self._arm
        if not arm.connected:
            raise RuntimeError('xArm disconnected')
        if self._enabled and self._reporting and not poll:
            _, report = self._feedback()
            errors, state = (report['error_code'], report['warn_code']), report['state']
        else:
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

    def prepare_force_torque(self, enable=False):
        """Sampling only; never zero, calibrate, or select a force control mode."""
        if self.dry_run:
            self.force_torque_config = {'simulated': True}
            return
        code, config = self._arm.get_ft_sensor_config()
        self._check(code, 'get_ft_sensor_config')
        if config[0] != 0:
            raise RuntimeError('F/T observations require non-force mode (0)')
        if not config[1]:
            if not enable or self.read_only:
                raise RuntimeError('F/T sampling is disabled; enable sensor sampling in xArm Studio before prediction-only inference')
            self._ft_started_by_us = True
            self._check(self._arm.set_ft_sensor_enable(1), 'set_ft_sensor_enable')
        self.force_torque_config = config
        self._check_ft_sensor_error()
        self.force_torque()  # validate before motion or file creation

    def _check_ft_sensor_error(self):
        # On firmware 2.7.0 this query knocks an enabled arm into state 5 ("configuration changed"), which
        # stops it (measured 2026-10-29), so it runs only before enable() and is never polled while running.
        if self._enabled:
            return
        code, error = self._arm.get_ft_sensor_error()
        self._check(code, 'get_ft_sensor_error')
        if error:
            raise RuntimeError(f'F/T sensor error: {error}')

    def force_torque(self):
        """[Fx,Fy,Fz,Tx,Ty,Tz], N and Nm, SDK native sensor coordinates.

        Filtered, load/offset-compensated external wrench; no TCP transform.
        Poll the controller rather than using a potentially stale SDK report cache.
        """
        if self.dry_run:
            return np.zeros(6)
        code, config = self._arm.get_ft_sensor_config()
        self._check(code, 'get_ft_sensor_config')
        if not config[1] or config[0] != 0:
            raise RuntimeError('F/T sampling disabled or controller in force mode')
        # The sensor-error register is checked once, before enable() (see _check_ft_sensor_error); config and
        # data are queried every sample; controller faults are also checked by observation().
        code, data = self._arm.get_ft_sensor_data(is_raw=False)
        self._check(code, 'get_ft_sensor_data')
        wrench = np.asarray(data, dtype=float)
        if wrench.shape != (6,) or not np.isfinite(wrench).all():
            raise RuntimeError('Invalid F/T sensor reading')
        self.force_torque_config = config
        return wrench

    def observation(self):
        if self.dry_run:
            return self._target.copy(), np.zeros(6)
        self.check_health()
        if self._reporting:
            _, report = self._feedback()
            # Report cartesian coordinates are XYZ mm + Euler RPY radians because
            # this connection was created with is_radian=True, not axis-angle.
            rpy_pose = np.asarray(report['cartesian'], dtype=float)
            if rpy_pose.shape != (6,) or not np.isfinite(rpy_pose).all():
                raise RuntimeError('Invalid TCP feedback from xArm')
            pose = np.r_[rpy_pose[:3], Rotation.from_euler('xyz', rpy_pose[3:]).as_rotvec()]
            joints = report['joints']
        else:
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
        if self._reporting and not self._enabled:
            # Validate the new report-based path against the controller's direct
            # axis-angle query before any enable/motion command, not on every tick.
            code, queried = self._arm.get_position_aa(is_radian=True)
            self._check(code, 'initial get_position_aa')
            queried = np.asarray(queried, dtype=float)
            if queried.shape != (6,) or not np.isfinite(queried).all():
                raise RuntimeError('Invalid initial TCP query from xArm')
            queried[:3] /= 1000
            distance = float(np.linalg.norm(queried[:3] - self._target[:3]))
            angle = (Rotation.from_rotvec(queried[3:]) *
                     Rotation.from_rotvec(self._target[3:]).inv()).magnitude()
            if distance > .001 or angle > np.deg2rad(.5):
                raise RuntimeError(f'xArm report/query pose mismatch before motion: '
                                   f'{distance * 1000:.3f} mm, {np.rad2deg(angle):.3f} deg')
            queried[3:] = continuous_rotvec(queried[3:], self._target[3:])
            self._target = queried
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
        self.check_health(poll=True)
        if self._reporting:
            # Do not mistake the pre-enable stopped/configuration report for a stop
            # during motion. Wait for a new, healthy report before starting the loop.
            enabled_at = time.monotonic()
            deadline = enabled_at + 2.0
            while True:
                stamp, report = self._feedback()
                if stamp >= enabled_at and report['state'] in (0, 1, 2):
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError('No ready xArm feedback after enabling')
                time.sleep(.005)
            self.check_health()
        self._last_servo_time = None

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
            now = time.monotonic()
            if (self._last_servo_time is not None and now - self._last_servo_time > 1.5 * period
                    and now - self._last_timing_warning > 2.):
                logging.getLogger(__name__).warning(
                    'Servo command gap %.1f ms (expected %.1f ms); control loop missed its deadline',
                    (now - self._last_servo_time) * 1000, period * 1000)
                self._last_timing_warning = now
            self._check(self._arm.set_servo_cartesian_aa(sdk_pose.tolist(), is_radian=True,
                                                       is_tool_coord=False, relative=False), 'servo')
            self._last_servo_time = now
        self._target = target.copy()

    def open_gripper(self):
        """Open the custom RS485 gripper with the same frame as visual_servoing/teleop.

        The reply confirms command acceptance, not measured jaw position.
        """
        if self.read_only:
            raise RuntimeError('Read-only robot cannot command the gripper')
        if self.dry_run:
            return
        self.check_health()
        code, baudrate = self._arm.get_tgpio_modbus_baudrate()
        self._check(code, 'get_tgpio_modbus_baudrate')
        if baudrate != 57600:
            self._check(self._arm.set_tgpio_modbus_baudrate(57600), 'set_tgpio_modbus_baudrate')
        self._check(self._arm.set_tgpio_modbus_timeout(100), 'set_tgpio_modbus_timeout')
        request = [0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00]
        code, reply = self._arm.getset_tgpio_modbus_data(request, is_transparent_transmission=False)
        self._check(code, 'open_gripper')
        if reply is None or list(reply)[:6] != request[:6]:
            raise RuntimeError(f'Gripper open command was not acknowledged: {reply}')

    def hand_back(self):
        """Release a healthy, ready arm to ROS without clearing faults or issuing a pose."""
        if self.read_only:
            raise RuntimeError('Read-only robot cannot hand back motion control')
        self.check_health()
        self._enabled = False  # close() must leave mode 1/state 0 for the ROS controller.

    def close(self):
        if self._arm is not None:
            try:
                if self._enabled:
                    self._check(self._arm.set_state(4), 'stop')
            finally:
                try:
                    if self._ft_started_by_us:
                        self._check(self._arm.set_ft_sensor_enable(0), 'restore_ft_sensor_sampling')
                finally:
                    self._ft_started_by_us = False
                    try:
                        if self._reporting:
                            self._arm.release_report_callback(self._on_report)
                    finally:
                        self._reporting = False
                        try:
                            self._arm.disconnect()
                        finally:
                            self._arm = None
                            self._enabled = False
