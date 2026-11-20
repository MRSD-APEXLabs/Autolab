"""Regression checks for steady servo delivery without hardware/network polling."""
from collections import Counter
import logging
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from act_inference.config import MotionConfig
from act_inference.control_rate import ControlRate
from act_inference.robot import XArm6Robot


class ReportApi:
    connected, axis = True, 6

    def __init__(self):
        self.state = 4
        self.error = 0
        self.estop = False   # pressed: clean_error cannot clear the error
        self.calls = Counter()
        self.callback = None
        self.timer = None

    def report(self):
        if self.callback:
            self.callback(dict(cartesian=[100., 200., 300., -3.1, -.02, 1.5], joints=[0.] * 7,
                               state=self.state, error_code=self.error, warn_code=0))

    def get_err_warn_code(self):
        self.calls['errors'] += 1
        return 0, [self.error, 0]

    def get_state(self):
        self.calls['state'] += 1
        return 0, self.state

    def get_position_aa(self, **kwargs):
        self.calls['pose'] += 1
        return 0, np.r_[[100., 200., 300.], Rotation.from_euler('xyz', [-3.1, -.02, 1.5]).as_rotvec()]

    def register_report_callback(self, callback, **kwargs):
        self.callback = callback
        self.report()
        return True

    def release_report_callback(self, callback):
        assert self.callback == callback
        self.callback = None

    def clean_error(self):
        self.calls['clean_error'] += 1
        if not self.estop:
            self.error = 0
        return 0

    def clean_warn(self):
        self.calls['clean_warn'] += 1
        return 0

    def motion_enable(self, enabled):
        return 0

    def set_mode(self, mode):
        return 0

    def set_state(self, state):
        self.state = state
        if state == 0:
            # The initial stopped report must not cause a false stop after enable.
            self.timer = threading.Timer(.01, self.report)
            self.timer.start()
        return 0

    def set_servo_cartesian_aa(self, pose, **kwargs):
        self.calls['servo'] += 1
        return 0

    def disconnect(self):
        if self.timer is not None:
            self.timer.join()
        self.calls['disconnect'] += 1


def connected_robot():
    api = ReportApi()
    robot = XArm6Robot('unused', MotionConfig(), api_factory=lambda *a, **kw: api)
    robot.connect()
    robot.initialize_target_pose()
    robot.enable()
    return robot, api


def test_active_ticks_use_report_socket_instead_of_six_getter_round_trips():
    robot, api = connected_robot()
    try:
        api.calls.clear()
        for _ in range(10):
            api.report()
            pose, joints = robot.observation()
            robot.servo(pose, .02, measured=pose)
        assert api.calls == {'servo': 10}
        np.testing.assert_allclose(pose[:3], [.1, .2, .3])
        np.testing.assert_allclose(Rotation.from_rotvec(pose[3:]).as_matrix(),
                                   Rotation.from_euler('xyz', [-3.1, -.02, 1.5]).as_matrix())
        assert joints.shape == (6,)
    finally:
        robot.close()
    assert api.calls['disconnect'] == 1 and api.callback is None


@pytest.mark.parametrize('state,error', [(3, 0), (4, 0), (5, 0), (0, 31)])
def test_reported_stop_pause_or_fault_prevents_servo(state, error):
    robot, api = connected_robot()
    try:
        api.calls.clear()
        api.state, api.error = state, error
        api.report()
        with pytest.raises(RuntimeError, match='fault|running state'):
            robot.servo(robot._target, .02)
        assert api.calls['servo'] == 0
    finally:
        robot.close()


def test_stale_reports_stop_before_sending_any_servo():
    robot, api = connected_robot()
    try:
        api.calls.clear()
        _, payload = robot._report
        robot._report = (time.monotonic() - .3, payload)
        with pytest.raises(RuntimeError, match='stale'):
            robot.servo(robot._target, .02)
        assert api.calls['servo'] == 0
    finally:
        robot.close()


def test_initial_pose_mismatch_blocks_enable(monkeypatch):
    api = ReportApi()
    robot = XArm6Robot('unused', MotionConfig(), api_factory=lambda *a, **kw: api)
    robot.connect()
    monkeypatch.setattr(api, 'get_position_aa', lambda **kw: (0, [100., 200., 350., 0., 0., 0.]))
    try:
        with pytest.raises(RuntimeError, match='report/query pose mismatch'):
            robot.initialize_target_pose()
        assert not robot._enabled and api.calls['servo'] == 0
    finally:
        robot.close()


def test_control_rate_accounts_for_work_without_accumulating_drift():
    now, ticks = [0.], []

    def wait(delay):
        now[0] += delay

    rate = ControlRate(.02, lambda: now[0], wait)
    for work in (.002, .008, .003, .005):
        now[0] += work
        rate.wait()
        ticks.append(now[0])
    np.testing.assert_allclose(ticks, [.02, .04, .06, .08])
    now[0] += .05  # injected 50 ms stall
    rate.wait()
    assert now[0] == pytest.approx(.15)  # skip missed slots; no catch-up burst


def test_servo_gap_warning_exposes_remaining_runtime_stalls(monkeypatch):
    warnings = []
    monkeypatch.setattr(logging.getLogger('act_inference.robot'), 'warning',
                        lambda message, *args: warnings.append((message, args)))
    robot, api = connected_robot()
    try:
        robot.servo(robot._target, .02)
        robot._last_servo_time -= .06
        robot.servo(robot._target, .02)
        assert len(warnings) == 1 and 'Servo command gap' in warnings[0][0]
        assert warnings[0][1][0] >= 60 and warnings[0][1][1] == 20
    finally:
        robot.close()


def test_forward_kinematics_matches_the_observation_format():
    robot, api = connected_robot()
    seen = []
    # the same xArm pose (mm + RPY rad) as the report: forward kinematics must convert it as observation() does
    api.get_forward_kinematics = lambda angles, **kwargs: seen.append((angles, kwargs)) or (
        0, [100., 200., 300., -3.1, -.02, 1.5])
    pose = robot.forward_kinematics((1.6284, -0.3927, -0.5812, 0.0, 0.9738, 1.6284))
    assert seen == [([1.6284, -0.3927, -0.5812, 0.0, 0.9738, 1.6284],
                     dict(input_is_radian=True, return_is_radian=True))]
    np.testing.assert_allclose(pose, robot.observation()[0], atol=1e-12)
    api.get_forward_kinematics = lambda angles, **kwargs: (1, [])
    with pytest.raises(RuntimeError, match='get_forward_kinematics failed'):
        robot.forward_kinematics([0.] * 6)


def test_clear_errors_enables_a_faulted_arm_without_motion():
    api = ReportApi()
    api.error = 31
    robot = XArm6Robot('unused', MotionConfig(), api_factory=lambda *a, **kw: api)
    with pytest.raises(RuntimeError, match='xArm fault'):
        robot.connect()
    robot.close()
    robot = XArm6Robot('unused', MotionConfig(), api_factory=lambda *a, **kw: api)
    robot.connect(check_health=False)
    assert robot.error_state() == (31, 0, 4)
    robot.clear_errors()
    assert robot.error_state() == (0, 0, 0)
    assert api.calls['clean_error'] == api.calls['clean_warn'] == 1 and api.calls['servo'] == 0
    robot.close()
    assert api.state == 0   # never enabled for motion, so close() leaves it ready for the ROS driver


def test_clear_errors_reports_an_arm_that_stays_faulted(monkeypatch):
    import act_inference.robot as robot_module
    api = ReportApi()
    api.error, api.estop = 2, True
    robot = XArm6Robot('unused', MotionConfig(), api_factory=lambda *a, **kw: api)
    robot.connect(check_health=False)
    clock = iter(range(100))
    monkeypatch.setattr(robot_module, 'time', SimpleNamespace(monotonic=lambda: next(clock), sleep=lambda s: None))
    with pytest.raises(RuntimeError, match='not ready after clearing: error C2.*emergency stop'):
        robot.clear_errors()
    monkeypatch.undo()
    robot.close()
    with pytest.raises(RuntimeError, match='Read-only'):
        XArm6Robot('unused', MotionConfig(), read_only=True).clear_errors()
