import json
import os
import threading
import time
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize('move_result', ['done', 'stopped', 'error'])
@pytest.mark.parametrize('retreat_result', ['done', 'stopped', 'error'])
def test_base_release_and_60_mm_retreat_before_handback(move_result, retreat_result):
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    events = []
    def move(*args, **kwargs):
        events.append('place')
        if move_result == 'error': raise RuntimeError('move failed')
        return move_result
    def retreat(*args, **kwargs):
        assert kwargs['lift_mm'] == 60.
        events.append('release_and_lift')
        if retreat_result == 'error': raise RuntimeError('retreat failed')
        return retreat_result
    node = SimpleNamespace(config=None, base_placement=None, rollout=SimpleNamespace(execute=True), base_mode='scripted',
                           runner=SimpleNamespace(status=lambda: dict(target='base'), _check_abort=lambda: None),
                           _base_move_fn=move, _post_placement_fn=retreat,
                           handoff=SimpleNamespace(finish=lambda check: events.append('handback')))
    if move_result == 'error' or (move_result == 'done' and retreat_result == 'error'):
        with pytest.raises(RuntimeError):
            ActInferenceNode._run(node, threading.Event())
    else:
        expected = retreat_result if move_result == 'done' else move_result
        assert ActInferenceNode._run(node, threading.Event()) == expected
    assert events == (['place'] + (['release_and_lift'] if move_result == 'done' else [])
                      + (['handback'] if move_result == retreat_result == 'done' else []))


def test_cancelled_yaw_does_not_start_policy():
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    node = SimpleNamespace(_yaw_align_fn=lambda *args, **kwargs: 'stopped',
                           config=None, rollout=SimpleNamespace(execute=True))
    # No rollout callback is provided: cancellation must return before accessing it.
    assert ActInferenceNode._run_policy(node, threading.Event()) == 'stopped'


def test_cancelled_base_start_move_does_not_start_policy():
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    node = SimpleNamespace(_base_start_fn=lambda *args, **kwargs: 'stopped', base_start=None,
                           config=None, rollout=SimpleNamespace(execute=True))
    assert ActInferenceNode._run_policy(node, threading.Event(), 'base') == 'stopped'


@pytest.mark.parametrize('target', ['ot2', 'shaker', 'base'])
@pytest.mark.parametrize('outcome', ['done', 'stopped', 'error'])
def test_policy_then_release_only_on_success(outcome, target):
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    stages = []

    def rollout(config, rollout_config, policy, stop):
        assert rollout_config == target + ' rollout' and policy == target + ' policy'
        stages.append('policy')
        if outcome == 'error':
            raise RuntimeError('policy failed')
        return outcome

    def release(*args, **kwargs):
        stages.append(f'release/lift {kwargs.get("lift_mm", 100.0):g}')
        return 'done'

    node = SimpleNamespace(config=None, rollout=SimpleNamespace(execute=True),
                           rollouts={t: t + ' rollout' for t in ('ot2', 'shaker', 'base')},
                           _policies={t: t + ' policy' for t in ('ot2', 'shaker', 'base')},
                           _yaw_align_fn=lambda *a, **kw: stages.append('yaw') or 'done',
                           base_start='start pose',
                           _base_start_fn=lambda config, pose, **kw: stages.append(pose) or 'done',
                           _rollout_fn=rollout, _post_placement_fn=release)
    if outcome == 'error':
        with pytest.raises(RuntimeError, match='policy failed'):
            ActInferenceNode._run_policy(node, threading.Event(), target)
    else:
        assert ActInferenceNode._run_policy(node, threading.Event(), target) == outcome
    # Base moves to its start pose (no 90 degree yaw alignment) and lifts 60 mm.
    expected = (['start pose'] if target == 'base' else ['yaw']) + ['policy']
    if outcome == 'done':
        expected.append('release/lift 60' if target == 'base' else 'release/lift 100')
    assert stages == expected


@pytest.mark.parametrize('base_mode, switches_camera', [('policy', True), ('scripted', False)])
def test_base_camera_only_for_the_policy(base_mode, switches_camera):
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    calls = []
    node = SimpleNamespace(base_mode=base_mode, runner=SimpleNamespace(status=lambda: dict(target='base')),
                           _verify_motion_ownership=lambda check: calls.append('ownership'),
                           _prepare_camera=lambda check: calls.append('camera'))
    ActInferenceNode._prepare_run(node, lambda: None)
    assert calls == ['ownership'] + (['camera'] if switches_camera else [])


@pytest.mark.parametrize('target', ['ot2', 'shaker', 'base', None])
def test_start_stop_and_status_over_ros(target):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    from act_inference.node import ActInferenceNode

    aligned = []

    def yaw_align(config, execute, stop):
        assert target != 'base' and not execute
        aligned.append('yaw')
        return 'done'

    def base_start(config, placement, execute, stop):
        assert target == 'base' and not execute
        assert placement.pose_mm_deg == (32.055321, 271.166351, 116.803917, -179.675382, -1.57403, -0.737282)
        assert placement.speed_mm_s == 15 and placement.rotation_speed_deg_s == 6
        aligned.append('start pose')
        return 'done'

    checkpoints = {'ot2': 'xarm6_placing_act_ft_all_20261105b', 'shaker': 'xarm6_shaker_act_ft_20261108c',
                   'base': 'xarm6_base_act_ft_20261108b'}

    def rollout(config, rollout_config, policy, stop):
        assert aligned == (['start pose'] if target == 'base' else ['yaw'])
        assert policy == 'policy' and not rollout_config.execute
        assert rollout_config.ckpt_dir.name == checkpoints[target or 'ot2']
        assert rollout_config.workspace_radius == .15
        stop.wait(5)
        return 'stopped'

    def base_move(config, placement, execute, stop):
        pytest.fail('base_mode policy (the default) must not run the scripted base move')

    namespace = f'/test_act_node_{os.getpid()}'
    rclpy.init()
    node = ActInferenceNode(policy='policy', rollout_fn=rollout, base_move_fn=base_move,
                            yaw_align_fn=yaw_align, base_start_fn=base_start, namespace=namespace,
                            parameter_overrides=[rclpy.parameter.Parameter('execute', value=False),
                                                 rclpy.parameter.Parameter('select_service', value='')])
    client_node = rclpy.create_node('act_client', namespace=namespace)
    assert node.ckpt_dir.name == 'xarm6_placing_act_ft_all_20261105b'
    assert {t: d.name for t, d in node.ckpt_dirs.items()} == checkpoints and node.base_mode == 'policy'
    assert all(d.is_dir() for d in node.ckpt_dirs.values())
    assert node.rollout.duration_sec == 60 and node.rollout.enable_rotation
    assert node.rollout.workspace_radius == .15 and node.rollout.final_offset == (0., 0., 0.)
    assert node.config.motion.translation_speed == .01 and node.config.motion.rotation_speed == .15
    statuses = []
    client_node.create_subscription(String, 'act_inference/status', lambda m: statuses.append(json.loads(m.data)),
                                    QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    start = client_node.create_client(Trigger, 'act_inference/start' + (f'/{target}' if target else ''))
    other = client_node.create_client(Trigger, 'act_inference/start/ot2' if target == 'base'
                                     else 'act_inference/start/base')
    stop = client_node.create_client(Trigger, 'act_inference/stop')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client_node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    def call(client):
        assert client.wait_for_service(timeout_sec=5)
        return client.call(Trigger.Request())
    try:
        assert call(start).success
        deadline = time.monotonic() + 0.3   # well inside the 0.5 s status timer: published on start
        while not any(s['running'] for s in statuses):
            assert time.monotonic() < deadline, statuses
            time.sleep(0.01)
        second = call(start)
        assert not second.success and second.message == 'already running'
        assert not call(other).success  # all targets share one arm/run lock
        assert node.runner.status()['target'] == (target or 'ot2')
        assert call(stop).success
        deadline = time.monotonic() + 5
        while not (statuses and statuses[-1]['result'] == 'stopped'):
            assert time.monotonic() < deadline, statuses[-3:]
            time.sleep(0.05)
        assert statuses[-1]['running'] is False and statuses[-1]['started'] > 0
        assert statuses[-1]['target'] == (target or 'ot2')
    finally:
        executor.shutdown()
        node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
