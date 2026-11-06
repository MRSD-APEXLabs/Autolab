from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import threading
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


def test_rollout_returns_done_after_duration(monkeypatch):
    setup_fake(monkeypatch)
    assert ar.run_act_rollout(InferenceConfig(), ar.RolloutConfig(Path('unused'), duration_sec=.06)) == 'done'


def test_stop_event_ends_the_loop_as_stopped(monkeypatch):
    calls = setup_fake(monkeypatch)
    stop = threading.Event()
    threading.Timer(.1, stop.set).start()
    t0 = time.monotonic()
    result = ar.run_act_rollout(InferenceConfig(), ar.RolloutConfig(Path('unused'), execute=True, duration_sec=5),
                                stop=stop)
    assert result == 'stopped' and time.monotonic() - t0 < 1.0
    assert calls[-2:] == ['robot-close', 'camera-stop']


def test_stop_set_before_start_never_enables(monkeypatch):
    calls = setup_fake(monkeypatch)
    stop = threading.Event()
    stop.set()
    assert ar.run_act_rollout(InferenceConfig(), ar.RolloutConfig(Path('unused'), execute=True), stop=stop) == 'stopped'
    assert 'enable' not in calls
    assert calls[-2:] == ['robot-close', 'camera-stop']


def test_stop_interrupts_initial_inference(monkeypatch):
    gate, entered, stop = threading.Event(), threading.Event(), threading.Event()

    def predict(*args):
        entered.set()
        gate.wait(3)
        return np.zeros((50, 6))

    calls = setup_fake(monkeypatch, predict)
    results = []
    worker = threading.Thread(target=lambda: results.append(ar.run_act_rollout(
        InferenceConfig(), ar.RolloutConfig(Path('unused'), execute=True), stop=stop)))
    try:
        worker.start()
        assert entered.wait(2)
        stop.set()
        worker.join(1)
        assert not worker.is_alive() and results == ['stopped']
        assert 'enable' not in calls and calls[-2:] == ['robot-close', 'camera-stop']
    finally:
        gate.set()
        worker.join(2)


def test_stop_interrupts_final_offset_before_servo():
    stop = threading.Event()
    stop.set()
    target = np.zeros(6)
    final, done = ar.move_final_offset(None, target, (0, .003, 0), MotionConfig(), target, .15,
                                       ar.ContactMonitor(), .02, stop=stop)
    assert not done
    np.testing.assert_array_equal(final, target)


def test_preloaded_policy_skips_loading(monkeypatch):
    setup_fake(monkeypatch)
    monkeypatch.setattr(ar, 'load_policy', lambda p: pytest.fail('policy reloaded'))
    ar.run_act_rollout(InferenceConfig(), ar.RolloutConfig(Path('unused'), duration_sec=.06), policy=(None, None, None))


def test_warm_up_runs_one_prediction_on_blank_inputs(monkeypatch):
    seen = []
    monkeypatch.setattr(ar, 'predict_chunk', lambda *args: seen.append(args) or np.zeros((50, 6)))
    ar.warm_up(('policy', 'stats', 'device'))
    (policy, stats, device, pose, rgb, width, depth), = seen
    assert (policy, stats, device) == ('policy', 'stats', 'device') and width == 480
    assert pose.shape == (6,) and rgb.shape == (600, 960, 3) and rgb.dtype == np.uint8
    assert depth.shape == (600, 960) and depth.dtype == np.uint16
    assert (depth > 0).all()   # an RGB-depth policy refuses an all-invalid frame


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


def test_executed_rollout_stops_when_pressing(monkeypatch, capsys):
    from act_inference.config import RolloutConfig
    calls = setup_fake(monkeypatch, lambda *args: np.tile([0., 0., -.05, 0., 0., 0.], (50, 1)))
    start = time.monotonic()
    ar.run_act_rollout(InferenceConfig(), RolloutConfig(Path('unused'), execute=True, duration_sec=3))
    assert time.monotonic() - start < 2 and 'Contact' in capsys.readouterr().out
    assert calls[-2:] == ['robot-close', 'camera-stop']


def test_contact_flags_reach_the_rollout_config():
    args = build_parser().parse_args(['--ckpt-dir', 'a', '--contact-gap', '0.002', '--contact-hold', '0.5'])
    rollout = configs_from_args(args)[1]
    assert (rollout.contact_gap, rollout.contact_hold) == (0.002, 0.5)


def test_hidden_state_reaches_the_policy_as_zeros():
    torch = pytest.importorskip('torch')
    seen = []

    class Policy:
        use_depth = False
        action_representation = 'relative_tcp_tool_v1'
        state_inputs = 'none'

        def __call__(self, state, image):
            seen.append(state.cpu().numpy()[0])
            return torch.zeros(1, 3, 6)

    stats = {'qpos_mean': np.full(6, .1), 'qpos_std': np.full(6, 2.), 'action_mean': np.zeros(6), 'action_std': np.ones(6)}
    pose = np.array([.4, .2, .1, 0, 0, np.pi / 2])
    ar.predict_chunk(Policy(), stats, 'cpu', pose, np.zeros((240, 320, 3), np.uint8))
    np.testing.assert_array_equal(seen[0], np.zeros(6))


def test_rollout_ends_when_the_policy_has_arrived(monkeypatch, capsys):
    from act_inference.config import RolloutConfig
    calls = setup_fake(monkeypatch, lambda *args: np.zeros((50, 6)))
    start = time.monotonic()
    ar.run_act_rollout(InferenceConfig(), RolloutConfig(Path('unused'), execute=True, duration_sec=5))
    assert time.monotonic() - start < 1.5 and 'Settled' in capsys.readouterr().out
    assert calls[-2:] == ['robot-close', 'camera-stop']
    args = build_parser().parse_args(['--ckpt-dir', 'a', '--settle-motion', '0.003', '--settle-queries', '4'])
    assert (configs_from_args(args)[1].settle_motion, configs_from_args(args)[1].settle_queries) == (0.003, 4)


def test_final_offset_moves_once_after_the_policy_settled(monkeypatch, capsys):
    from act_inference.config import RolloutConfig
    setup_fake(monkeypatch, lambda *args: np.zeros((50, 6)))
    servoed = []

    class Robot:
        def __init__(self, *args, **kw): self.pose = np.zeros(6)
        def connect(self): pass
        def initialize_target_pose(self): return np.zeros(6)
        def observation(self): return self.pose.copy(), np.zeros(6)
        def enable(self): pass
        def servo(self, target, period, measured=None):
            servoed.append(np.array(target))
            self.pose = np.array(target)
        def close(self): pass
    monkeypatch.setattr(ar, 'XArm6Robot', Robot)
    ar.run_act_rollout(InferenceConfig(), RolloutConfig(Path('unused'), execute=True, duration_sec=5, final_offset=(0, -.0035, 0)))
    assert 'Final offset done' in capsys.readouterr().out
    np.testing.assert_allclose(servoed[-1], [0, -.0035, 0, 0, 0, 0], atol=1e-9)
    args = build_parser().parse_args(['--ckpt-dir', 'a', '--final-offset-mm', '0', '-3.5', '0'])
    assert configs_from_args(args)[1].final_offset == (0.0, -.0035, 0.0)


def test_contact_monitor_ignores_free_motion_and_catches_a_press():
    # Free descent at 40 mm/s: the arm reaches each target one 20 ms tick later.
    monitor = ar.ContactMonitor(gap=.001, hold=.3, max_lag=.1)
    z = .2
    for tick in range(200):
        assert not monitor.update(tick * .02, z - .0008, z)
        z -= .0008
    # Blocked on a surface with the target 1.5 mm below: stops after the hold time, not before.
    monitor = ar.ContactMonitor(gap=.001, hold=.3, max_lag=.1)
    hits = [monitor.update(tick * .02, .0985, .1) for tick in range(30)]
    assert not any(hits[:15]) and hits[16]
    # Plate sliding up in the fingers: the arm still creeps down at 3 mm/s, 2 mm above its target.
    monitor = ar.ContactMonitor(gap=.001, hold=.3, max_lag=.1)
    assert any(monitor.update(tick * .02, .1 - tick * .00006 - .002, .1 - tick * .00006) for tick in range(40))
    # gap 0 disables it
    assert not any(ar.ContactMonitor(gap=0).update(tick * .02, 0., 1.) for tick in range(100))




def test_settle_monitor_holds_then_ends_on_arrived_chunks():
    settle = ar.SettleMonitor(motion=.002, queries=3)
    anchor = np.zeros(6)
    moving = np.tile([0., 0., -.01, 0., 0., 0.], (50, 1))
    arrived = np.tile([.001, 0., -.001, 0., 0., 0.], (50, 1))
    assert not settle.update(moving, anchor) and not settle.holding
    assert not settle.update(arrived, anchor) and settle.holding
    assert not settle.update(moving, anchor) and not settle.holding and settle.count == 0
    assert [settle.update(arrived, anchor) for _ in range(3)] == [False, False, True]
    assert not any(ar.SettleMonitor(motion=0).update(arrived, anchor) for _ in range(5))




def follower(**kw):
    return ar.ChunkFollower(MotionConfig(translation_speed=.01), np.zeros(6),
                            ar.RolloutConfig(Path('unused'), workspace_radius=1.0, **kw))


def toward(x, n=50):
    chunk = np.zeros((n, 6))
    chunk[:, 0] = x
    return chunk


def test_follower_slows_down_and_averages_chunks_near_the_goal():
    f = follower(approach_speed=.003, approach_distance=.012, ensemble=3)
    f.add(toward(.05), np.zeros(6), 0.0)
    target, proposed = f.step(np.zeros(6), 0.0)
    assert not f.near and proposed[0] == .05 and target[0] == pytest.approx(.01 / 50)
    for i, x in enumerate((.010, .011, .015, .011)):    # the first of these falls out of the 3-chunk window
        f.add(toward(x), np.zeros(6), .2 * (i + 1))
    target, proposed = f.step(np.zeros(6), .8)
    assert f.near and proposed[0] == pytest.approx(np.mean([.011, .015, .011]))
    assert target[0] == pytest.approx(.003 / 50)
    with pytest.raises(RuntimeError, match='No fresh policy action chunk'):
        f.step(target, .8 + 1.0)


def test_follower_without_approach_or_ensemble_follows_the_newest_chunk():
    f = follower(approach_speed=0, ensemble=1)
    f.add(toward(.011), np.zeros(6), 0.0)
    f.add(toward(.005), np.zeros(6), .2)
    target, proposed = f.step(np.zeros(6), .2)
    assert proposed[0] == .005 and target[0] == pytest.approx(.01 / 50)


def test_follower_holds_once_the_policy_reports_arrival():
    f = follower(settle_motion=.002, settle_queries=3)
    current = np.array([.001, 0, 0, 0, 0, 0])
    assert not f.add(toward(.0015), current, 0.0)
    target, _ = f.step(current, 0.0)
    np.testing.assert_array_equal(target, current)


def test_capture_anchored_chunks_start_where_the_image_was_taken(monkeypatch):
    seen, t0 = [], time.monotonic()

    def predict(policy, stats, device, pose, *args):
        seen.append(pose.copy())
        return np.tile(pose, (50, 1))
    setup_fake(monkeypatch, predict)
    monkeypatch.setattr(ar, 'load_policy', lambda p: (SimpleNamespace(anchor_at_capture=True), None, None))
    captured = []

    class Camera:
        calibration = {'rectified': True}
        def start(self): pass
        def check_mode(self, **kwargs): pass
        def latest(self):
            captured.append(time.monotonic() - .1)       # every image is 0.1 s old
            return SimpleNamespace(image=np.zeros((30, 48, 3), np.uint8), captured_monotonic=captured[-1])
        def stop(self): pass

    class Robot:
        def __init__(self, *args, **kw): pass
        def connect(self): pass
        def initialize_target_pose(self): return np.zeros(6)
        def observation(self):                           # moving along x at 10 mm/s
            return np.array([.01 * (time.monotonic() - t0), 0, 0, 0, 0, 0]), np.zeros(6)
        def close(self): pass
    monkeypatch.setattr(ar, 'make_camera', lambda c: Camera())
    monkeypatch.setattr(ar, 'XArm6Robot', Robot)
    ar.run_act_rollout(InferenceConfig(), ar.RolloutConfig(Path('unused'), duration_sec=2, settle_motion=0))
    later = np.array([pose[0] for pose in seen[2:]])
    assert len(later) >= 3
    # anchored about 1 mm (0.1 s at 10 mm/s) behind where the arm was when the query was sent
    lag = [np.min(np.abs(.01 * (np.array(captured) + .1 - t0) - x - .001)) for x in later]
    assert max(lag) < 3e-4


def test_predict_chunk_uses_the_checkpoint_image_size():
    torch = pytest.importorskip('torch')
    shapes = []

    class Policy:
        use_depth = False
        image_size = (48, 30)
        action_representation = 'relative_tcp_tool_v1'
        def __call__(self, state, image):
            shapes.append(tuple(image.shape))
            return torch.zeros(1, 3, 6)
    stats = {'qpos_mean': np.zeros(6), 'qpos_std': np.ones(6), 'action_mean': np.zeros(6), 'action_std': np.ones(6)}
    ar.predict_chunk(Policy(), stats, 'cpu', np.zeros(6), np.zeros((300, 480, 3), np.uint8))
    assert shapes == [(1, 1, 3, 30, 48)]
