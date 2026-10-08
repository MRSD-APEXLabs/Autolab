"""The ROS node against the in-memory Xavier server and the fake camera hub (needs a sourced ROS 2 environment)."""
import os
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import CameraInfo, CompressedImage, Image  # noqa: E402

from camera_fakes import FakeHub, fake_server, jpeg  # noqa: E402
from zedx_nano_camera.camera_node import NanoCameraNode  # noqa: E402

PAIRS = 3
INACTIVE = ('info', 'zedx inactive (hub active: zedx_nano); waiting')


def wait_for(executor, condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    return condition()


class LogRecorder:
    """Stands in for the node's logger; keeps (level, message) of every call."""

    def __init__(self):
        self.records = []

    def __getattr__(self, level):
        return lambda message, **kwargs: self.records.append((level, message))

    def levels(self):
        return {level for level, _ in self.records}


def test_node_publishes_stamped_stereo_pairs_with_calibration(monkeypatch):
    left = np.random.RandomState(0).randint(0, 255, (600, 960, 3)).astype(np.uint8)
    right = np.roll(left, -12, axis=1)
    pairs = [(jpeg(left), jpeg(right))] * PAIRS
    gate = threading.Event()
    fake_server(monkeypatch, pairs=pairs, gate=gate)
    namespace = f'/test_zedx_nano_camera_{os.getpid()}'
    rclpy.init()
    try:
        node = NanoCameraNode(namespace=namespace, parameter_overrides=[
            Parameter('host', value='xavier'), Parameter('reconnect_delay', value=30.0)])
        listener = rclpy.create_node('listener', namespace=namespace)
        received = {}

        def keep(key):
            return lambda msg: received.setdefault(key, []).append(msg)
        for eye in ('left', 'right'):
            listener.create_subscription(CompressedImage, f'{eye}/image_raw/compressed', keep((eye, 'jpeg')), 10)
            listener.create_subscription(CameraInfo, f'{eye}/camera_info', keep((eye, 'info')), 10)
        listener.create_subscription(Image, 'left/image_raw', keep(('left', 'raw')), 10)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(listener)
        publishers = [pub for eye in node.pubs.values() for kind, pub in eye.items() if kind != 'raw'] + [node.pubs['left']['raw']]
        assert wait_for(executor, lambda: all(pub.get_subscription_count() for pub in publishers))
        gate.set()
        keys = [(eye, kind) for eye in ('left', 'right') for kind in ('jpeg', 'info')] + [('left', 'raw')]
        assert wait_for(executor, lambda: all(len(received.get(key, [])) == PAIRS for key in keys)), \
            {key: len(value) for key, value in received.items()}
        node.stop()

        now_ns = node.get_clock().now().nanoseconds
        for i in range(PAIRS):
            stamps = {key: received[key][i].header.stamp for key in keys}
            assert len({(s.sec, s.nanosec) for s in stamps.values()}) == 1   # both eyes carry the left stamp
            stamp_ns = stamps[('left', 'jpeg')].sec * 10**9 + stamps[('left', 'jpeg')].nanosec
            assert 0 < now_ns - stamp_ns < 10 * 10**9
        assert bytes(received[('left', 'jpeg')][0].data) == pairs[0][0]
        assert bytes(received[('right', 'jpeg')][0].data) == pairs[0][1]
        assert 'jpeg' in received[('left', 'jpeg')][0].format
        assert received[('left', 'jpeg')][0].header.frame_id == 'zedx_nano_left_camera_optical_frame'
        assert received[('right', 'jpeg')][0].header.frame_id == 'zedx_nano_right_camera_optical_frame'
        raw = received[('left', 'raw')][0]
        assert (raw.width, raw.height, raw.encoding) == (960, 600, 'bgr8')
        left_info, right_info = received[('left', 'info')][0], received[('right', 'info')][0]
        assert (left_info.width, left_info.height) == (960, 600) and left_info.distortion_model == 'plumb_bob'
        assert left_info.p[3] == 0 and -right_info.p[3] / right_info.p[0] == pytest.approx(0.01801, rel=0.01)
        assert left_info.k[0] == pytest.approx(475)
        node.destroy_node()
        listener.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_node_waits_quietly_while_its_camera_is_inactive(monkeypatch):
    log = LogRecorder()
    monkeypatch.setattr(NanoCameraNode, 'get_logger', lambda self: log)
    namespace = f'/test_zedx_camera_{os.getpid()}'
    with FakeHub(active='zedx_nano') as hub:
        rclpy.init()
        try:
            # a 30 s reconnect delay: any error instead of the quiet inactive path would stall the test
            node = NanoCameraNode(namespace=namespace, parameter_overrides=[
                Parameter('host', value='127.0.0.1'), Parameter('port', value=hub.port),
                Parameter('camera', value='zedx'), Parameter('inactive_poll_period', value=0.05),
                Parameter('reconnect_delay', value=30.0)])
            listener = rclpy.create_node('listener', namespace=namespace)
            received = []
            listener.create_subscription(CompressedImage, 'left/image_raw/compressed', received.append, 10)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            executor.add_node(listener)
            assert wait_for(executor, lambda: hub.paths().count('/cameras/zedx/info') >= 5)
            assert log.records == [INACTIVE] and not received

            hub.select('zedx')
            assert wait_for(executor, lambda: len(received) >= 3)
            assert received[0].header.frame_id == 'zedx_left_camera_optical_frame'
            assert bytes(received[0].data) == hub.pairs['zedx'][0]
            assert ('info', 'zedx is active again') in log.records
            assert any(message.startswith('Streaming stereo 960x600 from http://127.0.0.1:%d/cameras/zedx/' % hub.port)
                       for _, message in log.records)

            hub.select('zedx_nano')   # ends the zedx feed
            polls = hub.paths().count('/cameras/zedx/info')
            assert wait_for(executor, lambda: hub.paths().count('/cameras/zedx/info') >= polls + 5)
            assert log.records.count(INACTIVE) == 2 and log.records[-1] == INACTIVE
            assert log.levels() == {'info'}   # no warnings or errors for a switch
            node.stop()
            executor.shutdown()
            node.destroy_node()
            listener.destroy_node()
        finally:
            rclpy.try_shutdown()


@pytest.mark.parametrize('camera, overrides, expected', [
    ('', [], ('zedx_nano_left_camera_optical_frame', 'zedx_nano_right_camera_optical_frame')),
    ('zedx', [], ('zedx_left_camera_optical_frame', 'zedx_right_camera_optical_frame')),
    ('zedx', [Parameter('right_frame_id', value='base_right')], ('zedx_left_camera_optical_frame', 'base_right')),
])
def test_frame_ids_follow_the_camera_unless_given(camera, overrides, expected):
    with FakeHub(active=None) as hub:   # no camera streams: the node only polls
        rclpy.init()
        try:
            node = NanoCameraNode(parameter_overrides=[
                Parameter('host', value='127.0.0.1'), Parameter('port', value=hub.port),
                Parameter('camera', value=camera)] + overrides)
            assert (node.frame_ids['left'], node.frame_ids['right']) == expected
            assert node.client.camera == camera
            node.stop()
            node.destroy_node()
        finally:
            rclpy.try_shutdown()


@pytest.mark.parametrize('parameter', [Parameter('sensor', value='middle'), Parameter('camera', value='../zedx')])
def test_node_rejects_unknown_sensor_or_camera(parameter):
    rclpy.init()
    try:
        with pytest.raises(ValueError, match=parameter.name):
            NanoCameraNode(parameter_overrides=[parameter])
    finally:
        rclpy.try_shutdown()
