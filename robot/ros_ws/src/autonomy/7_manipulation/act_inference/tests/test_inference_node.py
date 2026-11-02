import json
import os
import threading
import time
from types import SimpleNamespace

import pytest


def test_cancelled_yaw_does_not_start_policy():
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    node = SimpleNamespace(_yaw_align_fn=lambda *args, **kwargs: 'stopped',
                           config=None, rollout=SimpleNamespace(execute=True))
    # No rollout callback is provided: cancellation must return before accessing it.
    assert ActInferenceNode._run(node, threading.Event()) == 'stopped'


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
        aligned.append(True)
        return 'done'

    def rollout(config, rollout_config, policy, stop):
        assert target != 'base'
        assert aligned == [True]
        assert policy == 'policy' and not rollout_config.execute
        stop.wait(5)
        return 'stopped'

    def base_move(config, placement, execute, stop):
        assert target == 'base' and not execute
        assert placement.pose_mm_deg == (32.074207, 257.162689, 40.484722, -179.67395, -1.5738, -0.3)
        stop.wait(5)
        return 'stopped'

    namespace = f'/test_act_node_{os.getpid()}'
    rclpy.init()
    node = ActInferenceNode(policy='policy', rollout_fn=rollout, base_move_fn=base_move,
                            yaw_align_fn=yaw_align, namespace=namespace,
                            parameter_overrides=[rclpy.parameter.Parameter('execute', value=False),
                                                 rclpy.parameter.Parameter('select_service', value='')])
    client_node = rclpy.create_node('act_client', namespace=namespace)
    assert node.ckpt_dir.name == 'xarm6_placing_act_ft_20261029'
    assert node.rollout.duration_sec == 60 and node.rollout.enable_rotation
    assert node.rollout.workspace_radius == .15 and node.rollout.final_offset == (0., 0., 0.)
    assert node.config.motion.translation_speed == .01 and node.config.motion.rotation_speed == .15
    if target == 'base':
        class ForbiddenCameraClient:
            def wait_for_service(self, **kwargs):
                pytest.fail('base placement must not switch the camera')
        node._select_client = ForbiddenCameraClient()
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
