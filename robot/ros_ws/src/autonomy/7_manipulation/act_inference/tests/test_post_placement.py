from types import SimpleNamespace

import numpy as np
import pytest

from act_inference import base_placement as bp, post_placement as pp
from act_inference.config import InferenceConfig, MotionConfig
from act_inference.robot import XArm6Robot
from test_base_placement import ClockStop, setup_robot


def setup_release(monkeypatch, stop, open_error=False):
    _, calls, commands = setup_robot(monkeypatch, stop)
    base_robot = bp.XArm6Robot

    class Robot(base_robot):
        def open_gripper(self):
            assert not self.read_only
            calls.append('open')
            if open_error:
                raise RuntimeError('gripper failed')

        def servo(self, target, period, measured):
            assert 'open' in calls and stop.now >= 4.
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
