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
# The fake controller's forward kinematics, whatever the joints: about where home_offset is.
HOME_POSE = np.r_[[-.0206, .3438, .1853], Rotation.from_euler('xyz', [180., 0., 0.], degrees=True).as_rotvec()]


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
        report_types = []

        def __init__(self, ip, motion, read_only, report_type='rich'):
            self.previous = initial.copy()
            self.motion = motion
            self.read_only = read_only
            Robot.report_types.append(report_type)

        @property
        def pose(self):
            return state['pose']

        def connect(self):
            calls.append('connect')

        def initialize_target_pose(self):
            return state['pose'].copy()

        def forward_kinematics(self, joints):
            assert len(joints) == 6
            return HOME_POSE.copy()

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
            assert np.linalg.norm(target[:3] - self.previous[:3]) <= self.motion.translation_speed * period + 1e-10
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
    assert bp.XArm6Robot.report_types == ['real']   # 100 Hz measured pose, not the 5 Hz rich report
    deck_z = approach[2] - .008
    goals, held = phases(commands, approach)
    assert len(held) >= 5 + 20   # measured arrival (0.1 s), then settle + in-air baseline (0.4 s), before descending
    # from about the start's height: one move at speed_mm_s, no fast approach
    assert np.linalg.norm(np.diff(goals[:held[0] + 1, :3], axis=0), axis=1).max() <= .04 / 50 + 1e-10
    seating = goals[held[-1] + 1:]
    descent = seating[seating[:, 0] == approach[0]]
    slide = seating[seating[:, 0] < approach[0]]
    # descend straight down, 3.5 mm at 10 mm/s then 2 mm/s, to the deck (5 N/mm: 1 N above the baseline is
    # 0.2 mm into it)
    np.testing.assert_allclose(descent[:, 1], approach[1], atol=1e-10)
    assert np.all(np.diff(descent[:, 2]) <= 1e-12)
    down = np.abs(np.diff(np.r_[approach[2], descent[:, 2]]))
    fast = approach[2] - descent[:, 2] <= .0035 + 1e-10
    assert down[fast].max() == pytest.approx(.010 / 50) and down[~fast].max() == pytest.approx(.002 / 50)
    assert approach[2] - descent[fast][-1, 2] == pytest.approx(.0035)
    touch_z = descent[-1, 2]
    assert deck_z - .0004 < touch_z < deck_z - .0002
    # then along -X at the touch height until the ridge (1.0 N is 0.2 mm into it), and no further
    np.testing.assert_allclose(slide[:, 1:3], np.tile([approach[1], touch_z], (len(slide), 1)), atol=1e-10)
    assert np.all(np.diff(slide[:, 0]) <= 1e-12)
    side = np.abs(np.diff(np.r_[approach[0], slide[:, 0]]))
    fast = approach[0] - slide[:, 0] <= .012 + 1e-10    # 12 mm at 10 mm/s, then 4 mm/s
    assert side[fast].max() == pytest.approx(.010 / 50) and side[~fast].max() == pytest.approx(.004 / 50)
    ridge_x = approach[0] - .013
    assert ridge_x - .0004 < slide[:, 0].min() < ridge_x - .0002
    np.testing.assert_allclose(Rotation.from_rotvec(seating[:, 3:]).as_rotvec(),
                               np.tile(commands[-1][3:], (len(seating), 1)), atol=1e-10)
    out = capsys.readouterr().out
    assert 'touch at z=' in out and 'mm below the start' in out and 'ridge at x=' in out
    assert '[base debug]' not in out


def test_from_above_fast_to_the_clearance_then_straight_down_to_the_start(monkeypatch):
    stop = ClockStop()
    placement, calls, commands = setup_robot(monkeypatch, stop, initial_pose=HOME_POSE)
    start = placement.target_pose()
    assert bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop) == 'done'
    goals = np.asarray(commands)
    at = lambda point: np.flatnonzero(np.all(np.isclose(goals[:, :3], point, atol=1e-10, rtol=0), axis=1))
    above = at(start[:3] + [0., 0., .02])
    held = at(start[:3])
    assert len(above) >= 5 and above[-1] < held[0]   # measured arrival 20 mm above before going down
    fast = np.vstack([HOME_POSE[:3], goals[:above[0] + 1, :3]])
    steps = np.linalg.norm(np.diff(fast, axis=0), axis=1)
    assert steps.max() == pytest.approx(.15 / 50)
    # speeds up from rest and slows down to a stop at 500 mm/s^2 (one tick adds at most 0.5 * 0.02 m/s)
    ramp = .5 * (1 / 50) ** 2
    assert steps[0] == pytest.approx(ramp) and np.diff(steps).max() <= ramp + 1e-10
    assert np.all(np.diff(steps[-10:]) <= 1e-12) and steps[-1] <= 2 * ramp
    down = goals[above[-1]:held[0] + 1, :3]
    np.testing.assert_allclose(down[:, :2], np.tile(start[:2], (len(down), 1)), atol=1e-10)   # straight down
    down_steps = np.abs(np.diff(down[:, 2]))
    assert down_steps.max() == pytest.approx(.04 / 50)
    assert np.diff(down_steps[down_steps > 0]).max() <= ramp + 1e-10 and down_steps[-1] <= 2 * ramp
    assert np.all(np.diff(down[:, 2]) <= 1e-12)
    assert np.all(goals[:held[0] + 1, 2] >= start[2] - 1e-10)   # never below the start before probing
    assert len(held) >= 5 + 20 and goals[-1, 0] < start[0]     # then the same seating


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
    with pytest.raises(RuntimeError, match='no contact during slide within 50.0 mm'):
        bp.run_base_placement(InferenceConfig(), placement, execute=True, stop=stop)
    assert min(p[0] for p in commands) == pytest.approx(placement.target_pose()[0] - .050)
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
                                     {'probe_speed_mm_s': 50},
                                     {'slide_speed_mm_s': 50}, {'force_count': 0}, {'force_count': 1.5},
                                     {'approach_speed_mm_s': 0}, {'approach_clearance_mm': -1},
                                     {'lift_speed_mm_s': float('nan')}, {'accel_mm_s2': 0},
                                     {'fast_speed_mm_s': 50}, {'fast_speed_mm_s': 0}, {'probe_fast_mm': -1},
                                     {'probe_fast_mm': 11}, {'slide_fast_mm': 50}, {'open_wait_sec': 0}])
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
    assert stop.now >= np.linalg.norm(goal[:3] - home_offset[:3]) / .04 + .3


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


@pytest.mark.parametrize('axis', range(3))
@pytest.mark.parametrize('sign', [1., -1.])
def test_jog_moves_along_one_base_axis_holding_orientation(monkeypatch, axis, sign):
    stop = ClockStop()
    initial = np.r_[[.1, .2, .3], Rotation.from_euler('xyz', [-179., -1.5, 90.], degrees=True).as_rotvec()]
    _, calls, commands = setup_robot(monkeypatch, stop, initial_pose=initial)
    offset = np.zeros(3)
    offset[axis] = sign * .01
    assert bp.jog_base_xyz(InferenceConfig(), offset, 10., execute=True, stop=stop) == 'done'
    assert calls == ['connect', 'enable', 'close']
    goals = np.asarray(commands)
    np.testing.assert_allclose(goals[-1, :3], initial[:3] + offset, atol=1e-10)
    others = [i for i in range(3) if i != axis]
    np.testing.assert_allclose(goals[:, others], np.tile(initial[others], (len(goals), 1)), atol=1e-10)
    np.testing.assert_allclose(goals[:, 3:], np.tile(initial[3:], (len(goals), 1)), atol=1e-10)
    steps = np.linalg.norm(np.diff(np.vstack([initial[:3], goals[:, :3]]), axis=0), axis=1)
    assert steps.max() <= .010 / InferenceConfig().motion.loop_hz + 1e-10


def test_jog_prediction_only_does_not_command(monkeypatch):
    stop = ClockStop()
    _, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.jog_base_xyz(InferenceConfig(), (0., 0., .01), 10., execute=False, stop=stop) == 'done'
    assert calls == ['connect', 'close'] and not commands


def test_jog_cancellation(monkeypatch):
    stop = ClockStop(cancel_at=.06)
    _, calls, _ = setup_robot(monkeypatch, stop)
    assert bp.jog_base_xyz(InferenceConfig(), (0., 0., .01), 10., execute=True, stop=stop) == 'stopped'
    assert calls[-1] == 'close'


@pytest.mark.parametrize('offset,speed', [((0., 0.), 10.), ((0., np.nan, 0.), 10.), ((0., 0., .01), 0.)])
def test_jog_rejects_bad_requests_before_connecting(monkeypatch, offset, speed):
    stop = ClockStop()
    _, calls, _ = setup_robot(monkeypatch, stop)
    with pytest.raises(ValueError):
        bp.jog_base_xyz(InferenceConfig(), offset, speed, execute=True, stop=stop)
    assert not calls


def rpy_deg(rotvec):
    return Rotation.from_rotvec(rotvec).as_euler('xyz', degrees=True)


def rotation_steps(start, commands):
    rotvecs = np.vstack([start[3:], np.asarray(commands)[:, 3:]])
    return np.array([(Rotation.from_rotvec(b) * Rotation.from_rotvec(a).inv()).magnitude()
                     for a, b in zip(rotvecs[:-1], rotvecs[1:])])


@pytest.mark.parametrize('sign', [1., -1.])
def test_jog_rz_turns_about_the_base_z_axis_holding_xyz(monkeypatch, sign):
    stop = ClockStop()
    initial = np.r_[[.1, .2, .3], Rotation.from_euler('xyz', [-179., -1.5, 90.], degrees=True).as_rotvec()]
    _, calls, commands = setup_robot(monkeypatch, stop, initial_pose=initial)
    assert bp.jog_base_rz(InferenceConfig(), sign * 10., 10., execute=True, stop=stop) == 'done'
    assert calls == ['connect', 'enable', 'close']
    goals = np.asarray(commands)
    np.testing.assert_allclose(goals[:, :3], np.tile(initial[:3], (len(goals), 1)), atol=1e-10)
    np.testing.assert_allclose(rpy_deg(goals[-1, 3:]), rpy_deg(initial[3:]) + [0., 0., sign * 10.], atol=1e-6)
    assert rotation_steps(initial, commands).max() <= np.radians(10.) / InferenceConfig().motion.loop_hz + 1e-10
    assert stop.now >= 1.


def test_jog_rz_prediction_only_does_not_command(monkeypatch):
    stop = ClockStop()
    _, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.jog_base_rz(InferenceConfig(), 10., 10., execute=False, stop=stop) == 'done'
    assert calls == ['connect', 'close'] and not commands


@pytest.mark.parametrize('angle,speed', [(np.nan, 10.), (180., 10.), (10., 0.), (10., np.inf)])
def test_jog_rz_rejects_bad_requests_before_connecting(monkeypatch, angle, speed):
    stop = ClockStop()
    _, calls, _ = setup_robot(monkeypatch, stop)
    with pytest.raises(ValueError):
        bp.jog_base_rz(InferenceConfig(), angle, speed, execute=True, stop=stop)
    assert not calls


def scripted(*phases):
    """Teleop command(): each (velocity, ticks) phase in turn, then None (R2 released)."""
    queue = [velocity for velocity, ticks in phases for _ in range(ticks)]
    return lambda: queue.pop(0) if queue else None


TELEOP_START = np.r_[[.1, .2, .3], Rotation.from_euler('xyz', [-179., -1.5, 90.], degrees=True).as_rotvec()]


def teleop_steps(commands):
    return np.linalg.norm(np.diff(np.vstack([TELEOP_START[:3], np.asarray(commands)[:, :3]]), axis=0), axis=1)


def test_teleop_follows_the_sticks_holding_orientation(monkeypatch):
    stop = ClockStop()
    _, calls, commands = setup_robot(monkeypatch, stop, initial_pose=TELEOP_START)
    # 50 Hz at 10 mm/s: 0.2 mm per tick at full stick
    command = scripted(((1., 0., 0., 0.), 50), ((0., -.5, 0., 0.), 50), ((0., 0., 0., 0.), 10), ((0., 0., 1., 0.), 25))
    assert bp.teleop_base_xyz(InferenceConfig(), command, 10., .15, execute=True, stop=stop) == 'done'
    assert calls == ['connect', 'enable', 'close']
    goals = np.asarray(commands)
    np.testing.assert_allclose(goals[-1, :3], TELEOP_START[:3] + [.01, -.005, .005], atol=1e-10)
    np.testing.assert_allclose(goals[:, 3:], np.tile(TELEOP_START[3:], (len(goals), 1)), atol=1e-10)
    assert teleop_steps(commands).max() <= .010 / InferenceConfig().motion.loop_hz + 1e-10


def test_teleop_turns_about_the_base_z_axis(monkeypatch):
    stop = ClockStop()
    _, _, commands = setup_robot(monkeypatch, stop, initial_pose=TELEOP_START)
    # 50 Hz at 20 deg/s: 0.4 deg per tick at full stick; 50 ticks left = +20 deg, then 25 at half right = -5 deg
    command = scripted(((0., 0., 0., 1.), 50), ((0., 0., 0., -.5), 25))
    assert bp.teleop_base_xyz(InferenceConfig(), command, 10., .15, execute=True, stop=stop,
                              rotation_speed_deg_s=20.) == 'done'
    goals = np.asarray(commands)
    np.testing.assert_allclose(goals[:, :3], np.tile(TELEOP_START[:3], (len(goals), 1)), atol=1e-10)
    np.testing.assert_allclose(rpy_deg(goals[-1, 3:]), rpy_deg(TELEOP_START[3:]) + [0., 0., 15.], atol=1e-6)
    assert rotation_steps(TELEOP_START, commands).max() <= np.radians(20.) / InferenceConfig().motion.loop_hz + 1e-10


def test_teleop_diagonal_is_capped_at_the_speed(monkeypatch):
    stop = ClockStop()
    _, _, commands = setup_robot(monkeypatch, stop, initial_pose=TELEOP_START)
    assert bp.teleop_base_xyz(InferenceConfig(), scripted(((1., 3., -1., 0.), 50)), 10., .15, execute=True,
                              stop=stop) == 'done'
    np.testing.assert_allclose(np.asarray(commands)[-1, :3], TELEOP_START[:3] + np.array([1., 1., -1.]) / np.sqrt(3) * .01,
                               atol=1e-10)
    assert teleop_steps(commands).max() <= .010 / InferenceConfig().motion.loop_hz + 1e-10


def test_teleop_stays_within_the_radius(monkeypatch):
    stop = ClockStop()
    _, _, commands = setup_robot(monkeypatch, stop, initial_pose=TELEOP_START)
    command = scripted(((1., 0., 0., 0.), 200), ((0., 1., 0., 0.), 100))
    assert bp.teleop_base_xyz(InferenceConfig(), command, 10., .02, execute=True, stop=stop) == 'done'
    reach = np.linalg.norm(np.asarray(commands)[:, :3] - TELEOP_START[:3], axis=1)
    assert reach.max() <= .02 + 1e-9 and reach[-1] > .0199
    assert commands[-1][1] > TELEOP_START[1] + .005   # slides along the boundary


def test_teleop_prediction_only_does_not_command(monkeypatch):
    stop = ClockStop()
    _, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.teleop_base_xyz(InferenceConfig(), scripted(((1., 0., 0., 1.), 10)), 10., .15, stop=stop) == 'done'
    assert calls == ['connect', 'close'] and not commands


def test_teleop_cancellation(monkeypatch):
    stop = ClockStop(cancel_at=.5)
    _, calls, _ = setup_robot(monkeypatch, stop)
    assert bp.teleop_base_xyz(InferenceConfig(), lambda: (1., 0., 0., 1.), 10., .15, execute=True,
                              stop=stop) == 'stopped'
    assert calls[-1] == 'close'


@pytest.mark.parametrize('bad', [(1., 0., 0.), (np.nan, 0., 0., 0.), (0., 0., 0., np.inf)])
def test_teleop_rejects_a_bad_command(monkeypatch, bad):
    stop = ClockStop()
    _, calls, commands = setup_robot(monkeypatch, stop)
    with pytest.raises(ValueError):
        bp.teleop_base_xyz(InferenceConfig(), lambda: bad, 10., .15, execute=True, stop=stop)
    assert calls[-1] == 'close' and not commands


@pytest.mark.parametrize('speed,radius,rotation', [(0., .15, 30.), (np.nan, .15, 30.), (10., 0., 30.),
                                                   (10., np.inf, 30.), (10., .15, 0.), (10., .15, np.nan)])
def test_teleop_rejects_bad_settings_before_connecting(monkeypatch, speed, radius, rotation):
    stop = ClockStop()
    _, calls, _ = setup_robot(monkeypatch, stop)
    with pytest.raises(ValueError):
        bp.teleop_base_xyz(InferenceConfig(), lambda: None, speed, radius, execute=True, stop=stop,
                           rotation_speed_deg_s=rotation)
    assert not calls


def rotation_to(pose, reference):
    return (Rotation.from_rotvec(pose[3:]) * Rotation.from_rotvec(reference[3:]).inv()).magnitude()


def test_teleop_home_drives_straight_home_then_the_sticks_work_around_it(monkeypatch):
    stop = ClockStop()
    _, _, commands = setup_robot(monkeypatch, stop, initial_pose=TELEOP_START)
    # 220 mm from TELEOP_START at 10 mm/s: ~1100 ticks; then 10 ticks of +x (2 mm), beyond the start's 150 mm radius
    command = scripted(((0., 0., 0., 1.), 25), (bp.TELEOP_HOME, 1500), ((1., 0., 0., 0.), 10))
    assert bp.teleop_base_xyz(InferenceConfig(), command, 10., .15, execute=True, stop=stop,
                              home_joints=bp.HOME_OFFSET_JOINTS) == 'done'
    goals = np.asarray(commands)
    np.testing.assert_allclose(goals[-1, :3], HOME_POSE[:3] + [.002, 0., 0.], atol=1e-9)
    assert rotation_to(goals[-1], HOME_POSE) < 1e-8
    home_ticks = goals[25:25 + 1500, :3] - goals[24, :3]
    direction = (HOME_POSE[:3] - goals[24, :3]) / np.linalg.norm(HOME_POSE[:3] - goals[24, :3])
    assert np.abs(home_ticks - np.outer(home_ticks @ direction, direction)).max() < 1e-9   # a straight line
    assert teleop_steps(commands).max() <= .010 / InferenceConfig().motion.loop_hz + 1e-10
    assert rotation_steps(TELEOP_START, commands).max() <= np.radians(30.) / InferenceConfig().motion.loop_hz + 1e-10


def test_teleop_home_needs_home_joints(monkeypatch):
    stop = ClockStop()
    _, calls, commands = setup_robot(monkeypatch, stop)
    with pytest.raises(ValueError):
        bp.teleop_base_xyz(InferenceConfig(), lambda: bp.TELEOP_HOME, 10., .15, execute=True, stop=stop)
    assert calls[-1] == 'close' and not commands


def test_move_home_goes_straight_to_the_forward_kinematics_pose(monkeypatch):
    stop = ClockStop()
    _, calls, commands = setup_robot(monkeypatch, stop, initial_pose=TELEOP_START)
    assert bp.move_home(InferenceConfig(), bp.HOME_OFFSET_JOINTS, 15., 30., execute=True, stop=stop) == 'done'
    assert calls == ['connect', 'enable', 'close']
    goals = np.asarray(commands)
    np.testing.assert_allclose(goals[-1, :3], HOME_POSE[:3], atol=1e-9)
    assert rotation_to(goals[-1], HOME_POSE) < 1e-8
    path = goals[:, :3] - TELEOP_START[:3]
    direction = (HOME_POSE[:3] - TELEOP_START[:3]) / np.linalg.norm(HOME_POSE[:3] - TELEOP_START[:3])
    assert np.abs(path - np.outer(path @ direction, direction)).max() < 1e-9   # a straight line
    assert rotation_steps(TELEOP_START, commands).max() <= np.radians(30.) / InferenceConfig().motion.loop_hz + 1e-10
    assert stop.now >= np.linalg.norm(HOME_POSE[:3] - TELEOP_START[:3]) / .015


def test_move_home_prediction_only_does_not_command(monkeypatch):
    stop = ClockStop()
    _, calls, commands = setup_robot(monkeypatch, stop)
    assert bp.move_home(InferenceConfig(), bp.HOME_OFFSET_JOINTS, 15., 30., stop=stop) == 'done'
    assert calls == ['connect', 'close'] and not commands


def test_move_home_cancellation(monkeypatch):
    stop = ClockStop(cancel_at=.5)
    _, calls, _ = setup_robot(monkeypatch, stop, initial_pose=TELEOP_START)
    assert bp.move_home(InferenceConfig(), bp.HOME_OFFSET_JOINTS, 15., 30., execute=True, stop=stop) == 'stopped'
    assert calls[-1] == 'close'


@pytest.mark.parametrize('joints,speed,rotation', [((0.,) * 5, 15., 30.), ((np.nan,) + (0.,) * 5, 15., 30.),
                                                   (bp.HOME_OFFSET_JOINTS, 0., 30.), (bp.HOME_OFFSET_JOINTS, 15., np.inf)])
def test_move_home_rejects_bad_requests_before_connecting(monkeypatch, joints, speed, rotation):
    stop = ClockStop()
    _, calls, _ = setup_robot(monkeypatch, stop)
    with pytest.raises(ValueError):
        bp.move_home(InferenceConfig(), joints, speed, rotation, execute=True, stop=stop)
    assert not calls


def setup_clear_robot(monkeypatch):
    """Fake arm for clear_arm_errors: faulted (error C31, state 4) until clear_errors()."""
    calls, codes = [], [31, 0, 4]

    class Robot:
        def __init__(self, ip, motion, read_only):
            self.read_only = read_only

        def connect(self, check_health=True):
            assert not check_health   # a health check refuses the faulted arm this is for
            calls.append('connect')

        def error_state(self):
            return tuple(codes)

        def clear_errors(self):
            assert not self.read_only
            calls.append('clear')
            codes[:] = [0, 0, 0]

        def close(self):
            calls.append('close')

    monkeypatch.setattr(bp, 'XArm6Robot', Robot)
    return calls


def test_clear_errors_clears_and_enables_without_motion(monkeypatch, capsys):
    calls = setup_clear_robot(monkeypatch)
    assert bp.clear_arm_errors(InferenceConfig(), execute=True) == 'done'
    assert calls == ['connect', 'clear', 'close']
    out = capsys.readouterr().out
    assert 'xArm error C31, warning 0, state 4' in out and 'Cleared: xArm error C0, warning 0, state 0' in out


def test_clear_errors_prediction_only_only_reads(monkeypatch):
    calls = setup_clear_robot(monkeypatch)
    assert bp.clear_arm_errors(InferenceConfig(), execute=False) == 'done'
    assert calls == ['connect', 'close']


def test_clear_errors_preexisting_stop_never_connects(monkeypatch):
    calls = setup_clear_robot(monkeypatch)
    stop = ClockStop(cancel_at=0.)   # already set
    assert bp.clear_arm_errors(InferenceConfig(), execute=True, stop=stop) == 'stopped'
    assert not calls
