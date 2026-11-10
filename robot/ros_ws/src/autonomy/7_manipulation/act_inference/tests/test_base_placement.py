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


STIFFNESS_N_PER_M = 5000.   # fake deck and ridge: 5 N/mm


def setup_robot(monkeypatch, stop, tracks=True, initial_pose=None, deck_z=None, ridge_x=None, reports=True):
    """Fake arm and F/T report socket. By default the deck is 8 mm below the probing start and the ridge 13 mm
    along -X from it; deck_z/ridge_x = nan removes them."""
    placement = bp.BasePlacementConfig()
    goal = placement.target_pose()
    initial = goal.copy()
    initial[:3] -= [.002, .001, 0]
    initial[3:] = (Rotation.from_euler('z', .5, degrees=True) *
                   Rotation.from_rotvec(goal[3:])).as_rotvec()
    if initial_pose is not None:
        initial = initial_pose.copy()
    deck_z = goal[2] - .008 if deck_z is None else deck_z
    ridge_x = goal[0] - .013 if ridge_x is None else ridge_x
    calls, commands = [], []
    state = {'pose': initial.copy(), 'report': None, 'tick': 0}

    def wrench():
        pose = state['pose']
        fz = STIFFNESS_N_PER_M * max(0., deck_z - pose[2]) if np.isfinite(deck_z) else 0.
        fx = STIFFNESS_N_PER_M * max(0., ridge_x - pose[0]) if np.isfinite(ridge_x) else 0.
        noise = .01 * (-1) ** state['tick']
        return [.2 + fx + noise, -.1 + noise, -.3 - fz + noise, 0., 0., 0.]

    class Robot:
        def __init__(self, ip, motion, read_only):
            self.previous = initial.copy()
            self.motion = motion
            self.read_only = read_only

        @property
        def pose(self):
            return state['pose']

        def connect(self):
            calls.append('connect')

        def initialize_target_pose(self):
            return state['pose'].copy()

        def prepare_force_torque(self, enable):
            assert enable and not self.read_only
            calls.append('ft_prepare')

        def enable(self):
            assert not self.read_only
            calls.append('enable')

        def observation(self):
            state['tick'] += 1
            if reports and state['report'] is not None:
                state['report']({})
            return state['pose'].copy(), np.zeros(6)

        def servo(self, target, period, measured):
            assert not self.read_only
            assert np.linalg.norm(target[:3] - self.previous[:3]) <= .015 * period + 1e-10
            angle = (Rotation.from_rotvec(target[3:]) *
                     Rotation.from_rotvec(self.previous[3:]).inv()).magnitude()
            assert angle <= self.motion.rotation_speed * period + 1e-10
            commands.append(target.copy())
            self.previous = target.copy()
            if tracks:
                state['pose'] = target.copy()

        def close(self):
            calls.append('close')

    class FTApi:
        connected = True

        @property
        def ft_ext_force(self):
            return wrench()

        def register_report_callback(self, callback, **kwargs):
            calls.append('ft_connect')
            state['report'] = callback
            callback({})
            return True

        def release_report_callback(self, callback):
            state['report'] = None

        def disconnect(self):
            calls.append('ft_close')

    def factory(ip):
        return FTApi()

    monkeypatch.setattr(bp, 'XArm6Robot', Robot)
    monkeypatch.setattr(bp, 'time', SimpleNamespace(monotonic=lambda: stop.now, sleep=stop.wait))
    original = bp.run_base_placement
    monkeypatch.setattr(bp, 'run_base_placement',
                        lambda *args, **kwargs: original(*args, ft_factory=kwargs.pop('ft_factory', factory), **kwargs))
    return placement, calls, commands


def test_get_position_pose_is_mm_and_euler_degrees():
    placement = bp.BasePlacementConfig()
    goal = placement.target_pose()
    np.testing.assert_allclose(goal[:3], [.051565662, .257200928, .047687004])
    np.testing.assert_allclose(Rotation.from_rotvec(goal[3:]).as_euler('xyz', degrees=True),
                               [179.332982, -2.176667, 1.803213], atol=1e-8)


def phases(commands, approach):
    """Split commanded targets into the approach move and the seating that starts once it is held."""
    goals = np.asarray(commands)
    held = np.flatnonzero(np.all(np.isclose(goals[:, :3], approach[:3], atol=1e-10, rtol=0), axis=1))
    return goals, held


def test_touch_then_slide_to_the_ridge(monkeypatch, capsys):
    stop = ClockStop()
    placement, calls, commands = setup_robot(monkeypatch, stop)
    approach = placement.target_pose()
    assert bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop) == 'done'
    assert calls == ['connect', 'ft_prepare', 'ft_connect', 'enable', 'ft_close', 'close']
    deck_z = approach[2] - .008
    goals, held = phases(commands, approach)
    assert len(held) >= 15 + 40   # measured arrival, then settle + in-air baseline, before descending
    seating = goals[held[-1] + 1:]
    descent = seating[seating[:, 0] == approach[0]]
    slide = seating[seating[:, 0] < approach[0]]
    # descend straight down at 2 mm/s to the deck (5 N/mm: 1 N above the baseline is 0.2 mm into it)
    np.testing.assert_allclose(descent[:, 1], approach[1], atol=1e-10)
    assert np.all(np.diff(descent[:, 2]) <= 1e-12)
    assert np.abs(np.diff(descent[:, 2])).max() == pytest.approx(.002 / 50)
    touch_z = descent[-1, 2]
    assert deck_z - .0004 < touch_z < deck_z - .0002
    # then along -X at the touch height until the ridge (1.0 N is 0.2 mm into it), and no further
    np.testing.assert_allclose(slide[:, 1:3], np.tile([approach[1], touch_z], (len(slide), 1)), atol=1e-10)
    assert np.all(np.diff(slide[:, 0]) <= 1e-12)
    assert np.abs(np.diff(slide[:, 0])).max() == pytest.approx(.002 / 50)
    ridge_x = approach[0] - .013
    assert ridge_x - .0004 < slide[:, 0].min() < ridge_x - .0002
    np.testing.assert_allclose(Rotation.from_rotvec(seating[:, 3:]).as_rotvec(),
                               np.tile(commands[-1][3:], (len(seating), 1)), atol=1e-10)
    out = capsys.readouterr().out
    assert 'touch at z=' in out and 'mm below the start' in out and 'ridge at x=' in out
    assert '[base debug]' not in out


def test_prediction_only_never_enables_servos_or_opens_ft(monkeypatch):
    stop = ClockStop()
    placement, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.run_base_placement(InferenceConfig(), placement, stop=stop) == 'done'
    assert calls == ['connect', 'close'] and not commands


def test_stop_interrupts_base_move_and_closes(monkeypatch):
    stop = ClockStop(cancel_at=.06)
    placement, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop) == 'stopped'
    assert calls[-2:] == ['ft_close', 'close'] and 0 < len(commands) <= 4


def test_preexisting_stop_never_connects(monkeypatch):
    stop = ClockStop(cancel_at=0)
    placement, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop) == 'stopped'
    assert not calls and not commands


def test_stop_during_descent(monkeypatch):
    stop = ClockStop(cancel_at=3.)
    placement, calls, commands = setup_robot(monkeypatch, stop)
    goal = placement.target_pose()
    assert bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop) == 'stopped'
    assert max(p[2] for p in commands) == pytest.approx(goal[2])
    assert goal[2] - .008 < commands[-1][2] < goal[2] and calls[-1] == 'close'


def test_no_touch_fails_within_the_probe_depth(monkeypatch):
    stop = ClockStop()
    placement, calls, commands = setup_robot(monkeypatch, stop, deck_z=float('nan'))
    with pytest.raises(RuntimeError, match='no contact during descend within 11.0 mm'):
        bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop)
    assert min(p[2] for p in commands) == pytest.approx(placement.target_pose()[2] - .011)
    assert max(p[0] for p in commands) == min(p[0] for p in commands[-100:])   # never slid
    assert calls[-2:] == ['ft_close', 'close']


def test_no_ridge_fails_within_the_slide_limit(monkeypatch):
    stop = ClockStop()
    placement, calls, commands = setup_robot(monkeypatch, stop, ridge_x=float('nan'))
    with pytest.raises(RuntimeError, match='no contact during slide within 30.0 mm'):
        bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop)
    assert min(p[0] for p in commands) == pytest.approx(placement.target_pose()[0] - .030)
    assert calls[-1] == 'close'


def test_stale_ft_reports_stop_the_run(monkeypatch):
    stop = ClockStop()
    placement, calls, commands = setup_robot(monkeypatch, stop, reports=False)
    with pytest.raises(RuntimeError, match='F/T reports stale'):
        bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop)
    assert calls[-2:] == ['ft_close', 'close']


def test_debug_logs_every_tick(monkeypatch, capsys, tmp_path):
    stop = ClockStop()
    placement, calls, commands = setup_robot(monkeypatch, stop)
    placement = replace(placement, debug=True, debug_dir=str(tmp_path))
    assert bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop) == 'done'
    files = list(tmp_path.glob('base_placement_*.csv'))
    assert len(files) == 1
    rows = files[0].read_text().splitlines()
    assert rows[0].split(',') == list(bp._Trace.FIELDS)
    logged = [row.split(',')[1] for row in rows[1:]]
    assert ['baseline', 'descend', 'contact', 'slide', 'release'] == list(dict.fromkeys(logged))
    out = capsys.readouterr().out
    assert 'in-air baseline' in out and 'contact baseline' in out and '[base debug] slide' in out


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
                                     {'rotation_speed_deg_s': float('nan')}, {'timeout_sec': -1},
                                     {'probe_depth_mm': 0}, {'touch_force_n': -1}, {'ridge_force_n': float('nan')},
                                     {'probe_speed_mm_s': 20},
                                     {'slide_speed_mm_s': 20}, {'force_count': 0}, {'force_count': 1.5}])
def test_invalid_configuration(changes):
    with pytest.raises(ValueError):
        bp.BasePlacementConfig(**changes)


def start_placement():
    return replace(bp.BasePlacementConfig(), pose_mm_deg=bp.BASE_POLICY_START_MM_DEG)


def test_base_policy_start_pose_is_the_requested_get_position():
    goal = start_placement().target_pose()
    np.testing.assert_allclose(goal[:3], [.032055321, .271166351, .116803917])
    np.testing.assert_allclose(Rotation.from_rotvec(goal[3:]).as_euler('xyz', degrees=True),
                               [-179.675382, -1.57403, -0.737282], atol=1e-8)


def test_base_policy_start_is_one_straight_move_from_home_offset(monkeypatch):
    stop = ClockStop()
    placement = start_placement()
    home_offset = np.concatenate(([-.0206, .3438, .1853], Rotation.from_euler('xyz', [180, 0, 0], degrees=True).as_rotvec()))
    _, calls, commands = setup_robot(monkeypatch, stop, initial_pose=home_offset)
    assert bp.move_to_base_policy_start(InferenceConfig(), placement, execute=True, stop=stop) == 'done'
    assert calls == ['connect', 'enable', 'close']
    goal = placement.target_pose()
    np.testing.assert_allclose(commands[-1][:3], goal[:3], atol=1e-9)
    path = np.asarray(commands)[:, :3] - home_offset[:3]
    direction = (goal[:3] - home_offset[:3]) / np.linalg.norm(goal[:3] - home_offset[:3])
    assert np.abs(path - np.outer(path @ direction, direction)).max() < 1e-9   # straight line: no hover or detour
    assert stop.now >= np.linalg.norm(goal[:3] - home_offset[:3]) / .015 + .3


def test_base_policy_start_refuses_a_far_start(monkeypatch):
    stop = ClockStop()
    far = start_placement().target_pose()
    far[0] += .25
    _, calls, commands = setup_robot(monkeypatch, stop, initial_pose=far)
    with pytest.raises(RuntimeError, match='250 mm away'):
        bp.move_to_base_policy_start(InferenceConfig(), start_placement(), execute=True, stop=stop)
    assert calls == ['connect', 'close'] and not commands


def test_base_policy_start_prediction_only_never_enables(monkeypatch):
    stop = ClockStop()
    _, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.move_to_base_policy_start(InferenceConfig(), start_placement(), stop=stop) == 'done'
    assert calls == ['connect', 'close'] and not commands
