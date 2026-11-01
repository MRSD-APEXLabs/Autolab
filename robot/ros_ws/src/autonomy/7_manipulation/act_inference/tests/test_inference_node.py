import json
import os
import threading
import time

import pytest


def test_start_stop_and_status_over_ros():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    from act_inference.node import ActInferenceNode

    def rollout(config, rollout_config, policy, stop):
        assert policy == 'policy' and not rollout_config.execute
        stop.wait(5)
        return 'stopped'

    namespace = f'/test_act_node_{os.getpid()}'
    rclpy.init()
    node = ActInferenceNode(policy='policy', rollout_fn=rollout, namespace=namespace,
                            parameter_overrides=[rclpy.parameter.Parameter('execute', value=False),
                                                 rclpy.parameter.Parameter('select_service', value='')])
    client_node = rclpy.create_node('act_client', namespace=namespace)
    statuses = []
    client_node.create_subscription(String, 'act_inference/status', lambda m: statuses.append(json.loads(m.data)),
                                    QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    start = client_node.create_client(Trigger, 'act_inference/start')
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
        assert call(stop).success
        deadline = time.monotonic() + 5
        while not (statuses and statuses[-1]['result'] == 'stopped'):
            assert time.monotonic() < deadline, statuses[-3:]
            time.sleep(0.05)
        assert statuses[-1]['running'] is False and statuses[-1]['started'] > 0
    finally:
        executor.shutdown()
        node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
