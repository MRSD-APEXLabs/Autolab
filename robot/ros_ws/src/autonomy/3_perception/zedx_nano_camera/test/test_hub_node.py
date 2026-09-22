"""The camera hub bridge node against the fake hub (needs a sourced ROS 2 environment)."""
import json
import os
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from camera_fakes import FakeHub  # noqa: E402
from zedx_nano_camera.hub_node import STATUS_QOS, CameraHubNode  # noqa: E402


def wait_for(executor, condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    return condition()


class Bridge:
    """A CameraHubNode spinning on a MultiThreadedExecutor thread (as in main) plus a node that reads
    its status and calls its services, spun by the test."""

    def __init__(self, port, **params):
        namespace = f'/test_camera_hub_{os.getpid()}'
        overrides = [Parameter('host', value='127.0.0.1'), Parameter('port', value=port)]
        self.node = CameraHubNode(namespace=namespace, parameter_overrides=overrides + [
            Parameter(name, value=value) for name, value in params.items()])
        self.node_executor = MultiThreadedExecutor()
        self.node_executor.add_node(self.node)
        self.spinner = threading.Thread(target=self.node_executor.spin, daemon=True)
        self.spinner.start()
        self.io = rclpy.create_node('io', namespace=namespace)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.io)
        self.statuses = []

    def subscribe(self):
        self.io.create_subscription(String, 'camera_hub/status', lambda msg: self.statuses.append(json.loads(msg.data)),
                                    STATUS_QOS)

    def wait_for(self, condition, timeout=10.0):
        return wait_for(self.executor, condition, timeout)

    def call_async(self, service):
        client = self.io.create_client(Trigger, f'camera_hub/{service}')
        assert self.wait_for(client.service_is_ready)
        return client.call_async(Trigger.Request())

    def call(self, service):
        future = self.call_async(service)
        assert self.wait_for(future.done, timeout=20.0)
        return future.result()

    def close(self):
        self.node.stop()
        self.node_executor.shutdown()
        self.spinner.join(timeout=5.0)
        self.executor.shutdown()
        self.node.destroy_node()
        self.io.destroy_node()


@pytest.fixture
def ros():
    rclpy.init()
    try:
        yield
    finally:
        rclpy.try_shutdown()


@pytest.fixture
def logs(monkeypatch):
    """(level, message) of everything CameraHubNode logs."""
    records = []

    class Logger:
        def __getattr__(self, level):
            return lambda message, **kwargs: records.append((level, message))
    monkeypatch.setattr(CameraHubNode, 'get_logger', lambda self: Logger())
    return records


def test_status_is_latched_and_every_switch_publishes_the_new_state(ros):
    with FakeHub(active='zedx_nano') as hub:
        bridge = Bridge(hub.port, poll_period=60.0)   # after the first poll, only switches publish
        assert bridge.wait_for(lambda: hub.paths() == ['/status'])
        bridge.subscribe()                              # late subscriber: transient local delivers the last status
        assert bridge.wait_for(lambda: bridge.statuses)
        status = bridge.statuses[-1]
        assert status['server'] == 'camera_hub' and status['active'] == 'zedx_nano' and status['state'] == 'streaming'
        assert status['reachable'] is True and status['poll_error'] is None
        assert status['hub_url'] == f'http://127.0.0.1:{hub.port}/' and 'zedx' in status['cameras']

        reply = bridge.call('select_zedx')
        assert reply.success and reply.message == 'zedx streaming' and hub.active == 'zedx'
        assert bridge.wait_for(lambda: bridge.statuses[-1]['active'] == 'zedx')
        assert hub.selects == [{'timeout': ['30']}]

        reply = bridge.call('release')
        assert reply.success and reply.message == 'no camera idle' and hub.active is None
        assert bridge.wait_for(lambda: bridge.statuses[-1]['active'] is None)

        hub.failing.add('zedx_nano')
        reply = bridge.call('select_zedx_nano')
        assert not reply.success and reply.message == 'zedx_nano failed to start'
        assert bridge.wait_for(lambda: bridge.statuses[-1]['state'] == 'error')
        assert bridge.statuses[-1]['error'] == 'zedx_nano failed to start' and hub.active == 'zedx_nano'
        assert hub.paths().count('/status') == 4        # one per switch, none from the 60 s poll
        bridge.close()


def test_polling_continues_while_a_select_blocks(ros):
    with FakeHub(active='zedx_nano', select_delay=1.0) as hub:
        bridge = Bridge(hub.port, poll_period=0.05, select_timeout=5.0)
        reply = bridge.call('select_zedx')              # the service blocks for 1 s
        assert reply.success and hub.selects == [{'timeout': ['5']}]
        requests = hub.requests[:]
        during = requests[requests.index(('POST', '/select')):]
        assert during.count(('GET', '/status')) >= 5
        bridge.close()


def test_a_release_supersedes_a_select_that_is_still_waiting(ros):
    with FakeHub(active='zedx_nano', select_delay=5.0) as hub:
        bridge = Bridge(hub.port, poll_period=60.0)
        slow = bridge.call_async('select_zedx')         # the hub answers it after 5 s ...
        assert bridge.wait_for(lambda: hub.paths('POST') == ['/select'])
        hub.select_delay = 0.0
        start = time.monotonic()
        reply = bridge.call('release')                  # ... unless another select comes in first
        assert reply.success and reply.message == 'no camera idle' and hub.active is None
        assert time.monotonic() - start < 2.0 and not slow.done()
        assert bridge.wait_for(slow.done)
        reply = slow.result()
        assert not reply.success and reply.message == 'selection changed to None while waiting'
        bridge.close()


def test_stopping_does_not_wait_for_a_select(ros):
    with FakeHub(active='zedx_nano', select_delay=10.0) as hub:
        bridge = Bridge(hub.port, poll_period=60.0)
        slow = bridge.call_async('select_zedx')
        assert bridge.wait_for(lambda: hub.paths('POST') == ['/select'])
        start = time.monotonic()
        bridge.node.stop()
        assert bridge.wait_for(slow.done, timeout=3.0) and time.monotonic() - start < 3.0
        assert not slow.result().success and 'shutting down' in slow.result().message
        bridge.close()


def test_unreachable_hub_is_reported_not_raised(ros, logs):
    hub = FakeHub()
    hub.close()
    bridge = Bridge(hub.port, poll_period=0.05, timeout=0.5)
    bridge.subscribe()
    assert bridge.wait_for(lambda: len(bridge.statuses) >= 3)
    status = bridge.statuses[-1]
    assert status['reachable'] is False and status['poll_error'] and 'active' not in status
    assert status['hub_url'] == f'http://127.0.0.1:{hub.port}/'
    [(level, message)] = logs                           # unreachable from the first poll: said once
    assert level == 'warning' and message.startswith(f'Camera hub http://127.0.0.1:{hub.port}/ unreachable')
    reply = bridge.call('select_zedx')
    assert not reply.success and 'Connection refused' in reply.message

    del logs[:]
    with FakeHub(active='zedx_nano') as live:          # the hub comes up, then goes away again
        bridge.node.client.port = live.port
        assert bridge.wait_for(lambda: bridge.statuses[-1]['reachable'])
    bridge.node.client.port = hub.port
    assert bridge.wait_for(lambda: not bridge.statuses[-1]['reachable'])
    time.sleep(0.2)
    assert [level for level, _ in logs] == ['info', 'warning']
    assert logs[0][1] == 'Camera hub: zedx_nano streaming' and 'unreachable' in logs[1][1]
    bridge.close()


def test_camera_names_become_service_names(ros):
    with FakeHub(active=None) as hub:
        bridge = Bridge(hub.port, cameras=['zedx'])

        def services():
            return {name.rsplit('/', 1)[1] for name, _ in
                    bridge.node.get_service_names_and_types_by_node('camera_hub', bridge.node.get_namespace())}
        assert bridge.wait_for(lambda: {'select_zedx', 'release'} <= services())
        assert 'select_zedx_nano' not in services()
        bridge.close()
        with pytest.raises(ValueError, match='cameras'):
            CameraHubNode(parameter_overrides=[Parameter('cameras', value=['zedx', 'zed x'])])
