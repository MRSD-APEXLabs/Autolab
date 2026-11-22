from types import SimpleNamespace

import numpy as np
import pytest

from act_inference import base_placement as bp, post_placement as pp
from act_inference.config import InferenceConfig, MotionConfig
from act_inference.robot import XArm6Robot
from test_base_placement import ClockStop, setup_robot


def setup_release(monkeypatch, stop, open_error=False, open_wait=4., before_open=None):
    """`before_open`: a list that collects servo targets sent before the open command (else those fail)."""
    _, calls, commands = setup_robot(monkeypatch, stop)
    base_robot = bp.XArm6Robot
    opened_at = []

    class Robot(base_robot):
        def open_gripper(self):
            assert not self.read_only
            calls.append('open')
            opened_at.append(stop.now)
            if open_error:
                raise RuntimeError('gripper failed')

        def servo(self, target, period, measured):
            if 'open' in calls:
                assert stop.now >= opened_at[0] + open_wait
            else:
                assert before_open is not None
                before_open.append(target.copy())
            super().servo(target, period, measured)

    monkeypatch.setattr(pp, 'XArm6Robot', Robot)
    return calls, commands


@pytest.mark.parametrize('lift_mm', [60., 100.])
def test_open_then_lift_without_changing_xy_orientation(monkeypatch, lift_mm):
    stop = ClockStop()
    calls, commands = setup_release(monkeypatch, stop)
    initial = bp.BasePlacementConfig().target_pose()
    initial[:3] -= [.002, .001, 0]
    config = InferenceConfig(motion=MotionConfig(translation_speed=.01))
    lift_args = {'lift_mm': lift_mm} if lift_mm == 60. else {}  # Policy uses the default 100 mm.
    assert pp.release_and_lift(config, execute=True, stop=stop, **lift_args) == 'done'
    assert calls == ['connect', 'open', 'enable', 'close']
    assert commands[-1][2] == pytest.approx(initial[2] + lift_mm / 1000)
    np.testing.assert_allclose(np.asarray(commands)[:, :2], np.tile(initial[:2], (len(commands), 1)))
    np.testing.assert_allclose(np.asarray(commands)[:, 3:], np.tile(commands[0][3:], (len(commands), 1)))
    assert stop.now >= 4. + lift_mm / 1000 / .01 + .3


def test_ot2_shift_along_base_x_before_opening(monkeypatch):
    stop = ClockStop()
    before_open = []
    calls, commands = setup_release(monkeypatch, stop, before_open=before_open)
    initial = bp.BasePlacementConfig().target_pose()
    initial[:3] -= [.002, .001, 0]
    config = InferenceConfig(motion=MotionConfig(translation_speed=.01))
    assert pp.release_and_lift(config, execute=True, stop=stop, speed_mm_s=25., shift_x_mm=-2.) == 'done'
    assert calls == ['connect', 'enable', 'open', 'enable', 'close']
    shift = np.asarray(before_open)
    assert shift[-1][0] == pytest.approx(initial[0] - .002)
    np.testing.assert_allclose(shift[:, 1:3], np.tile(initial[1:3], (len(shift), 1)))
    np.testing.assert_allclose(shift[:, 3:], np.tile(shift[0][3:], (len(shift), 1)))
    assert np.diff(np.r_[initial[0], shift[:, 0]]).min() >= -.01 / 50 - 1e-12   # ACT speed, not the 25 mm/s lift
    lift = np.asarray(commands[len(shift):])
    np.testing.assert_allclose(lift[:, :2], np.tile([initial[0] - .002, initial[1]], (len(lift), 1)))
    assert lift[-1][2] == pytest.approx(initial[2] + .1)


def test_stop_during_shift_never_opens(monkeypatch):
    stop = ClockStop(cancel_at=.1)
    calls, commands = setup_release(monkeypatch, stop, before_open=[])
    config = InferenceConfig(motion=MotionConfig(translation_speed=.001))
    assert pp.release_and_lift(config, execute=True, stop=stop, shift_x_mm=-2.) == 'stopped'
    assert 'open' not in calls


def test_lift_at_the_given_speed(monkeypatch):
    stop = ClockStop()
    calls, commands = setup_release(monkeypatch, stop)
    config = InferenceConfig(motion=MotionConfig(translation_speed=.01))
    assert pp.release_and_lift(config, execute=True, stop=stop, lift_mm=60., speed_mm_s=40.) == 'done'
    assert calls == ['connect', 'open', 'enable', 'close']
    assert np.diff(np.asarray(commands)[:, 2]).max() == pytest.approx(.04 / 50)
    assert 4. + .06 / .04 + .3 <= stop.now < 4. + .06 / .01


def test_ramped_lift_speeds_up_and_slows_down(monkeypatch):
    stop = ClockStop()
    calls, commands = setup_release(monkeypatch, stop)
    config = InferenceConfig(motion=MotionConfig(translation_speed=.01))
    assert pp.release_and_lift(config, execute=True, stop=stop, lift_mm=60., speed_mm_s=150.,
                               accel_mm_s2=500.) == 'done'
    assert calls == ['connect', 'open', 'enable', 'close']
    start_z = bp.BasePlacementConfig().target_pose()[2]
    steps = np.diff(np.r_[start_z, np.asarray(commands)[:, 2]])
    steps = steps[steps > 0]
    ramp = .5 * (1 / 50) ** 2
    assert steps.max() == pytest.approx(.15 / 50)
    assert steps[0] == pytest.approx(ramp) and np.diff(steps).max() <= ramp + 1e-10 and steps[-1] <= 2 * ramp
    # 60 mm at 150 mm/s plus 0.3 s of ramps, then the 0.3 s measured arrival: well under the 1.5 s at 40 mm/s
    assert 4. + .06 / .15 + .3 + .3 - .05 <= stop.now < 4. + .06 / .15 + .3 + .3 + .1


def test_base_lift_waits_its_open_time_on_fast_reports(monkeypatch):
    stop = ClockStop()
    calls, commands = setup_release(monkeypatch, stop, open_wait=1.5)
    config = InferenceConfig(motion=MotionConfig(translation_speed=.01))
    assert pp.release_and_lift(config, execute=True, stop=stop, lift_mm=60., speed_mm_s=150., accel_mm_s2=500.,
                               open_wait_sec=1.5, report_type='real', arrival_sec=.1) == 'done'
    assert calls == ['connect', 'open', 'enable', 'close'] and bp.XArm6Robot.report_types[-1] == 'real'
    # 1.5 s open wait, 60 mm at 150 mm/s plus 0.3 s of ramps, then the 0.1 s measured arrival
    assert 1.5 + .06 / .15 + .3 + .1 - .05 <= stop.now < 1.5 + .06 / .15 + .3 + .1 + .1


def test_bad_shift_is_refused_before_connecting(monkeypatch):
    stop = ClockStop()
    calls, commands = setup_release(monkeypatch, stop)
    with pytest.raises(ValueError, match='shift_x_mm'):
        pp.release_and_lift(InferenceConfig(), execute=True, stop=stop, shift_x_mm=float('nan'))
    assert not calls and not commands


@pytest.mark.parametrize('open_wait_sec', [-1., float('nan')])
def test_bad_open_wait_is_refused_before_connecting(monkeypatch, open_wait_sec):
    stop = ClockStop()
    calls, commands = setup_release(monkeypatch, stop)
    with pytest.raises(ValueError, match='open_wait_sec'):
        pp.release_and_lift(InferenceConfig(), execute=True, stop=stop, open_wait_sec=open_wait_sec)
    assert not calls and not commands


@pytest.mark.parametrize('accel_mm_s2', [0., -5., float('nan')])
def test_bad_lift_accel_is_refused_before_connecting(monkeypatch, accel_mm_s2):
    stop = ClockStop()
    calls, commands = setup_release(monkeypatch, stop)
    with pytest.raises(ValueError, match='accel_mm_s2'):
        pp.release_and_lift(InferenceConfig(), execute=True, stop=stop, accel_mm_s2=accel_mm_s2)
    assert not calls and not commands


@pytest.mark.parametrize('speed_mm_s', [0., -5., float('nan')])
def test_bad_lift_speed_is_refused_before_connecting(monkeypatch, speed_mm_s):
    stop = ClockStop()
    calls, commands = setup_release(monkeypatch, stop)
    with pytest.raises(ValueError, match='speed_mm_s'):
        pp.release_and_lift(InferenceConfig(), execute=True, stop=stop, speed_mm_s=speed_mm_s)
    assert not calls and not commands


@pytest.mark.parametrize('cancel_at', [1., 4.1])
def test_stop_during_open_wait_or_lift(monkeypatch, cancel_at):
    stop = ClockStop(cancel_at=cancel_at)
    calls, commands = setup_release(monkeypatch, stop)
    assert pp.release_and_lift(InferenceConfig(motion=MotionConfig(translation_speed=.01)),
                               execute=True, stop=stop) == 'stopped'
    assert calls[-1] == 'close'
    assert bool(commands) == (cancel_at > 4.)


def test_failed_open_never_lifts(monkeypatch):
    stop = ClockStop()
    calls, commands = setup_release(monkeypatch, stop, open_error=True)
    with pytest.raises(RuntimeError, match='gripper failed'):
        pp.release_and_lift(InferenceConfig(), execute=True, stop=stop)
    assert calls == ['connect', 'open', 'close'] and not commands


def test_prediction_only_has_no_gripper_or_motion_commands(monkeypatch):
    stop = ClockStop()
    calls, commands = setup_release(monkeypatch, stop)
    assert pp.release_and_lift(InferenceConfig(), stop=stop) == 'done'
    assert calls == ['connect', 'close'] and not commands


@pytest.mark.parametrize('reply_kind', ['ok', 'bad', 'error'])
def test_custom_gripper_open_frame_and_errors(monkeypatch, reply_kind):
    robot = XArm6Robot('unused', MotionConfig())
    monkeypatch.setattr(robot, 'check_health', lambda: None)
    frames = []

    def transact(frame, is_transparent_transmission):
        assert not is_transparent_transmission
        frames.append(frame)
        return (1, []) if reply_kind == 'error' else (0, frame[:6] if reply_kind == 'ok' else [0])

    robot._arm = SimpleNamespace(get_tgpio_modbus_baudrate=lambda: (0, 57600),
                                  set_tgpio_modbus_timeout=lambda ms: 0 if ms == 100 else 1,
                                  getset_tgpio_modbus_data=transact)
    if reply_kind == 'ok':
        robot.open_gripper()
    else:
        with pytest.raises(RuntimeError):
            robot.open_gripper()
    assert frames == [[0x08, 0x10, 0x07, 0, 0, 2, 4, 0, 0, 0, 0]]


def test_read_only_robot_refuses_gripper_command():
    robot = XArm6Robot('unused', MotionConfig(), read_only=True)
    with pytest.raises(RuntimeError, match='Read-only'):
        robot.open_gripper()
