"""The node on a synthetic image topic: lazy subscription, JPEG frames, health (needs ROS 2)."""
import json
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
np = pytest.importorskip('numpy')
cv2 = pytest.importorskip('cv2')
from cv_bridge import CvBridge  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

from http_fakes import open_stream, read_parts, request  # noqa: E402
from image_stream_server.server_node import ImageStreamServer  # noqa: E402

TOPIC = '/test_stream/annotated_image'


def wait_until(condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.05)
    return condition()


@pytest.fixture
def stack():
    context = rclpy.Context()
    rclpy.init(context=context)
    node = ImageStreamServer(context=context, parameter_overrides=[
        Parameter('host', value='127.0.0.1'), Parameter('port', value=0),
        Parameter('streams', value=[f'annotated/cam={TOPIC}'])])
    publisher_node = rclpy.create_node('fake_perception', context=context)
    publisher = publisher_node.create_publisher(Image, TOPIC, 2)
    executor = MultiThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(publisher_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield node, publisher
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        publisher_node.destroy_node()
        rclpy.shutdown(context=context)
        thread.join(2.0)


def image():
    bgr = np.zeros((48, 64, 3), np.uint8)
    bgr[:, :32] = (0, 0, 255)
    return CvBridge().cv2_to_imgmsg(bgr, encoding='bgr8')


def test_subscribes_only_while_watched(stack):
    node, publisher = stack
    port = node.server.server_port
    assert wait_until(lambda: publisher.get_subscription_count() == 0)
    with open_stream(port, '/stream/annotated/cam.mjpg') as response:
        assert wait_until(lambda: publisher.get_subscription_count() == 1)
        stop = threading.Event()

        def publish():
            while not stop.is_set():
                publisher.publish(image())
                time.sleep(0.05)
        threading.Thread(target=publish, daemon=True).start()
        try:
            jpeg = read_parts(response, 1)[0]
        finally:
            stop.set()
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape == (48, 64, 3) and decoded[24, 10, 2] > 200
        status, _, body = request(port, '/health')
        health = json.loads(body)['annotated/cam']
        assert status == 200 and health['subscribed'] is True and health['viewers'] == 1
    assert wait_until(lambda: publisher.get_subscription_count() == 0)
    assert json.loads(request(port, '/health')[2])['annotated/cam']['subscribed'] is False


def test_invalid_streams_param_fails_start():
    context = rclpy.Context()
    rclpy.init(context=context)
    try:
        with pytest.raises(ValueError, match='bad stream entry'):
            ImageStreamServer(context=context, parameter_overrides=[
                Parameter('port', value=0), Parameter('streams', value=['nonsense'])])
    finally:
        rclpy.shutdown(context=context)
