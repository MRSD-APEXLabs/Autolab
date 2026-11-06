from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from act_inference import base_placement as bp
from act_inference.config import InferenceConfig


class ClockStop:
    def __init__(self, cancel_at=float('inf')):
        self.now = 0.
        self.cancel_at = cancel_at

    def is_set(self):
        return self.now >= self.cancel_at

    def wait(self, seconds):
        self.now += max(seconds, .00001)
        return self.is_set()


def setup_robot(monkeypatch, stop, tracks=True, initial_pose=None):
    placement = bp.BasePlacementConfig()
    goal = placement.target_pose()
    initial = goal.copy()
    initial[:3] -= [.002, .001, 0]
    initial[3:] = (Rotation.from_euler('z', .5, degrees=True) *
                   Rotation.from_rotvec(goal[3:])).as_rotvec()
    if initial_pose is not None:
        initial = initial_pose.copy()
    calls, commands = [], []

    class Robot:
        def __init__(self, ip, motion, read_only):
            self.pose = initial.copy()
            self.previous = initial.copy()
            self.motion = motion
            self.read_only = read_only

        def connect(self):
            calls.append('connect')

        def initialize_target_pose(self):
            return self.pose.copy()

        def enable(self):
            assert not self.read_only
            calls.append('enable')

        def observation(self):
            return self.pose.copy(), np.zeros(6)

        def servo(self, target, period, measured):
            assert not self.read_only
            assert np.linalg.norm(target[:3] - self.previous[:3]) <= .015 * period + 1e-10
            angle = (Rotation.from_rotvec(target[3:]) *
                     Rotation.from_rotvec(self.previous[3:]).inv()).magnitude()
            assert angle <= self.motion.rotation_speed * period + 1e-10
            commands.append(target.copy())
            self.previous = target.copy()
            if tracks:
                self.pose = target.copy()

        def close(self):
            calls.append('close')

    monkeypatch.setattr(bp, 'XArm6Robot', Robot)
    monkeypatch.setattr(bp, 'time', SimpleNamespace(monotonic=lambda: stop.now))
    return placement, calls, commands


def test_get_position_pose_is_mm_and_euler_degrees():
    placement = bp.BasePlacementConfig()
    goal = placement.target_pose()
    np.testing.assert_allclose(goal[:3], [.032074207, .257162689, .040484722])
    np.testing.assert_allclose(Rotation.from_rotvec(goal[3:]).as_euler('xyz', degrees=True),
                               [-179.67395, -1.5738, -0.3], atol=1e-8)


def test_slow_move_reaches_measured_goal_and_closes(monkeypatch):
    stop = ClockStop()
    placement, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop) == 'done'
    assert calls == ['connect', 'enable', 'close']
    np.testing.assert_allclose(commands[-1][:3], placement.target_pose()[:3], atol=1e-9)
    assert stop.now >= np.hypot(.002, .001) / .015 + .3
    goals = np.asarray(commands)
    hover_z = placement.target_pose()[2] + .010
    hover = np.flatnonzero(np.isclose(goals[:, 2], hover_z, atol=1e-10, rtol=0))
    assert len(hover) >= 15  # measured arrival is held before starting the descent
    descent = goals[hover[-1] + 1:]
    assert len(descent) > 0
    np.testing.assert_allclose(descent[:, :2], np.tile(placement.target_pose()[:2], (len(descent), 1)), atol=1e-10)
    assert np.all(np.diff(descent[:, 2]) <= 1e-10)
    assert stop.now >= .010 / .015 + .010 / .015 + .6


def test_prediction_only_never_enables_or_servos(monkeypatch):
    stop = ClockStop()
    placement, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.run_base_placement(InferenceConfig(), placement, stop=stop) == 'done'
    assert calls == ['connect', 'close'] and not commands


def test_stop_interrupts_base_move_and_closes(monkeypatch):
    stop = ClockStop(cancel_at=.06)
    placement, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop) == 'stopped'
    assert calls[-1] == 'close' and 0 < len(commands) <= 4


def test_preexisting_stop_never_connects(monkeypatch):
    stop = ClockStop(cancel_at=0)
    placement, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop) == 'stopped'
    assert not calls and not commands


def test_stop_during_descent(monkeypatch):
    stop = ClockStop(cancel_at=1.2)
    placement, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop) == 'stopped'
    assert max(p[2] for p in commands) == pytest.approx(placement.target_pose()[2] + .010)
    assert commands[-1][2] > placement.target_pose()[2] and calls[-1] == 'close'


@pytest.mark.parametrize('yaw', [89., -179.])
def test_yaw_alignment_holds_position_roll_and_pitch(monkeypatch, yaw):
    stop = ClockStop()
    initial = np.r_[[.1, .2, .3], Rotation.from_euler('xyz', [-179., -1.5, yaw], degrees=True).as_rotvec()]
    _, calls, commands = setup_robot(monkeypatch, stop, initial_pose=initial)
    assert bp.align_policy_yaw(InferenceConfig(), execute=True, stop=stop) == 'done'
    assert calls == ['connect', 'enable', 'close']
    np.testing.assert_allclose(np.asarray(commands)[:, :3], np.tile(initial[:3], (len(commands), 1)), atol=1e-10)
    rpy = Rotation.from_rotvec(np.asarray(commands)[:, 3:]).as_euler('xyz', degrees=True)
    first_step_deg = abs((rpy[0, 2] - yaw + 180.) % 360. - 180.)
    assert first_step_deg == pytest.approx(9. / InferenceConfig().motion.loop_hz)
    np.testing.assert_allclose(rpy[:, :2], np.tile([-179., -1.5], (len(commands), 1)), atol=1e-8)
    assert rpy[-1, 2] == pytest.approx(90.)


@pytest.mark.parametrize('execute,yaw', [(False, 0.), (True, 90.)])
def test_yaw_prediction_only_or_already_aligned_does_not_command(monkeypatch, execute, yaw):
    stop = ClockStop()
    initial = np.r_[[.1, .2, .3], Rotation.from_euler('xyz', [-179., -1., yaw], degrees=True).as_rotvec()]
    _, calls, commands = setup_robot(monkeypatch, stop, initial_pose=initial)
    assert bp.align_policy_yaw(InferenceConfig(), execute=execute, stop=stop) == 'done'
    assert calls == ['connect', 'close'] and not commands


def test_yaw_alignment_cancellation(monkeypatch):
    stop = ClockStop(cancel_at=.06)
    setup_robot(monkeypatch, stop)
    assert bp.align_policy_yaw(InferenceConfig(), execute=True, stop=stop) == 'stopped'


def test_yaw_stops_on_unexpected_vertical_motion(monkeypatch):
    stop = ClockStop()
    _, calls, commands = setup_robot(monkeypatch, stop)
    base_robot = bp.XArm6Robot

    class DriftingRobot(base_robot):
        def observation(self):
            pose, joints = super().observation()
            pose[2] -= .003
            return pose, joints

    monkeypatch.setattr(bp, 'XArm6Robot', DriftingRobot)
    with pytest.raises(RuntimeError, match='unexpected XYZ drift'):
        bp.align_policy_yaw(InferenceConfig(), execute=True, stop=stop)
    assert not commands and calls[-1] == 'close'


def test_timeout_requires_measured_arrival_and_closes(monkeypatch):
    stop = ClockStop()
    placement, calls, commands = setup_robot(monkeypatch, stop, tracks=False)
    with pytest.raises(RuntimeError, match='timed out'):
        bp.run_base_placement(InferenceConfig(), replace(placement, timeout_sec=1), execute=True, stop=stop)
    assert commands and calls[-1] == 'close'


@pytest.mark.parametrize('changes', [{'pose_mm_deg': (1, 2)}, {'speed_mm_s': 0},
                                     {'rotation_speed_deg_s': float('nan')}, {'timeout_sec': -1}])
def test_invalid_configuration(changes):
    with pytest.raises(ValueError):
        bp.BasePlacementConfig(**changes)
