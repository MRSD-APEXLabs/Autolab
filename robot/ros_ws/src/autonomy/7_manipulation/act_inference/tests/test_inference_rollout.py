from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from act_inference import rollout as ar
from act_inference.cli import build_parser, configs_from_args
from act_inference.config import InferenceConfig, MotionConfig


def test_action_limits_and_fixed_orientation():
    initial = np.array([0., 0., 0., 3.1, 0., 0.])
    proposed = np.array([.05, .05, 0., 2.9, .3, .1])
    limited = ar.limit_action(initial, proposed, MotionConfig(), .02, initial, False, .1)
    assert np.linalg.norm(limited[:3]) == pytest.approx(.0005)
    np.testing.assert_array_equal(limited[3:], initial[3:])
    rotated = ar.limit_action(initial, proposed, MotionConfig(), .02, initial, True, .1)
    delta = Rotation.from_rotvec(rotated[3:]) * Rotation.from_rotvec(initial[3:]).inv()
    assert delta.magnitude() == pytest.approx(.002)


def test_invalid_target():
    with pytest.raises(RuntimeError):
        ar.limit_action(np.zeros(6), np.full(6, np.nan), MotionConfig(), .02, np.zeros(6), False, .1)


def setup_fake(monkeypatch, predict=None, rectified=True):
    calls = []

    class Camera:
        calibration = {'rectified': rectified}

        def start(self):
            calls.append('camera-start')

        def check_mode(self, **kwargs):
            pass

        def latest(self):
            return SimpleNamespace(image=np.zeros((30, 48, 3), np.uint8), depth=np.ones((30, 48), np.uint16),
                                   captured_monotonic=time.monotonic())

        def stop(self):
            calls.append('camera-stop')

    class Robot:
        def __init__(self, *args, **kw):
            calls.append(('robot', kw))

        def connect(self):
            calls.append('read-connect')

        def initialize_target_pose(self):
            return np.zeros(6)

        def observation(self):
            return np.zeros(6), np.zeros(6)

        def enable(self):
            calls.append('enable')

        def servo(self, *args, **kw):
            calls.append('servo')

        def close(self):
            calls.append('robot-close')
    monkeypatch.setattr(ar, 'load_policy', lambda p: (None, None, None))
    monkeypatch.setattr(ar, 'make_camera', lambda c: Camera())
    monkeypatch.setattr(ar, 'XArm6Robot', Robot)
    monkeypatch.setattr(ar, 'predict_chunk', predict or (lambda *args: np.zeros((50, 6))))
    return calls


def test_prediction_only_never_enables_or_commands(monkeypatch):
    calls = setup_fake(monkeypatch)
    ar.run_act_rollout(InferenceConfig(), ar.RolloutConfig(Path('unused'), duration_sec=.06))
    assert 'read-connect' in calls and 'enable' not in calls and 'servo' not in calls
    assert ('robot', {'dry_run': False, 'read_only': True}) in calls
    assert calls[-2:] == ['robot-close', 'camera-stop']


def test_execute_is_explicit_and_cleans_up(monkeypatch):
    calls = setup_fake(monkeypatch)
    ar.run_act_rollout(InferenceConfig(), ar.RolloutConfig(Path('unused'), execute=True, duration_sec=.06))
    assert calls.index('enable') < calls.index('servo')
    assert calls[-2:] == ['robot-close', 'camera-stop']


def test_initial_invalid_policy_never_enables(monkeypatch):
    calls = setup_fake(monkeypatch, lambda *args: np.full((50, 6), np.nan))
    with pytest.raises(RuntimeError, match='Nonfinite'):
        ar.run_act_rollout(InferenceConfig(), ar.RolloutConfig(Path('unused'), execute=True, duration_sec=.06))
    assert 'enable' not in calls
    assert calls[-2:] == ['robot-close', 'camera-stop']


def test_unrectified_camera_is_refused_before_the_robot_connects(monkeypatch):
    calls = setup_fake(monkeypatch, rectified=False)
    with pytest.raises(RuntimeError, match='rectified'):
        ar.run_act_rollout(InferenceConfig(camera_source='ros'), ar.RolloutConfig(Path('unused'), execute=True))
    assert 'read-connect' not in calls and 'enable' not in calls


def test_inference_timeout_stops(monkeypatch):
    n = 0

    def predictor(*args):
        nonlocal n
        n += 1
        if n > 1:
            time.sleep(.15)
        return np.zeros((50, 6))
    calls = setup_fake(monkeypatch, predictor)
    with pytest.raises(RuntimeError, match='deadline|fresh'):
        ar.run_act_rollout(InferenceConfig(), ar.RolloutConfig(Path('unused'), execute=True, duration_sec=.3,
                                                                query_interval=.01, prediction_timeout=.06))
    assert calls[-2:] == ['robot-close', 'camera-stop']


def test_control_rate_must_match_the_policy(monkeypatch):
    setup_fake(monkeypatch)
    with pytest.raises(ValueError, match='50 Hz'):
        ar.run_act_rollout(InferenceConfig(motion=MotionConfig(loop_hz=30)), ar.RolloutConfig(Path('unused')))


def test_cli_defaults_to_prediction_only():
    config, rollout = configs_from_args(build_parser().parse_args(['--ckpt-dir', 'a', '--duration-sec', '3']))
    assert not rollout.execute and rollout.duration_sec == 3 and rollout.ckpt_dir == Path('a')
    assert config == InferenceConfig() and config.camera_source == 'nano-stream' and config.depth_backend == 'neural'
    config, rollout = configs_from_args(build_parser().parse_args([
        '--ckpt-dir', 'a', '--execute', '--camera-source', 'ros', '--ros-namespace', '/wrist', '--depth-backend', 'sgbm',
        '--depth-models-dir', '/m', '--translation-speed', '0.01']))
    assert rollout.execute and config.camera_source == 'ros' and config.ros_namespace == '/wrist'
    assert config.depth_backend == 'sgbm' and config.depth_models_dir == '/m' and config.motion.translation_speed == 0.01
    with pytest.raises(SystemExit):
        build_parser().parse_args(['--execute'])   # checkpoint is required


@pytest.mark.parametrize('overrides', [{'camera_source': 'camera-edge'}, {'camera_port': 0}, {'depth_width': 10},
                                       {'depth_backend': 'magic'}, {'record_image_width': -1}, {'max_frame_age': 0}])
def test_config_validation(overrides):
    with pytest.raises(ValueError):
        replace(InferenceConfig(), **overrides)


def test_rollout_config_validation():
    with pytest.raises(ValueError):
        ar.RolloutConfig(Path('a'), workspace_radius=float('nan'))
    with pytest.raises(ValueError):
        MotionConfig(translation_speed=0)
