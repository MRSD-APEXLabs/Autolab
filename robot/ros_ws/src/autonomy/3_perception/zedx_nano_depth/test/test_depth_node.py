"""The ROS node on a synthetic stereo pair with the SGBM backend (needs a sourced ROS 2 environment)."""
import os
import time

import cv2
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from cv_bridge import CvBridge  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import CameraInfo, CompressedImage, Image  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

from depth_fakes import EXPECTED_MM, FX, H, W, camera_info, ideal_eyes, synthetic_pair, valid_center  # noqa: E402
from zedx_nano_depth.depth_node import NanoDepthNode  # noqa: E402


def wait_for(executor, condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    return condition()


def info_msg(eye, header):
    fake = camera_info(eye)
    msg = CameraInfo(header=header, width=fake.width, height=fake.height, distortion_model='plumb_bob')
    msg.k, msg.d, msg.r, msg.p = fake.k, fake.d, fake.r, fake.p
    return msg


@pytest.mark.parametrize('transport', ['compressed', 'raw'])
def test_node_publishes_depth_aligned_to_the_rectified_left_image(transport):
    namespace = f'/test_zedx_nano_depth_{os.getpid()}_{transport}'
    rclpy.init()
    try:
        node = NanoDepthNode(namespace=namespace, parameter_overrides=[
            Parameter('backend', value='sgbm'), Parameter('input_transport', value=transport),
            Parameter('match_width', value=480)])
        io = rclpy.create_node('io', namespace=namespace)
        bridge = CvBridge()
        received = {}

        def keep(key):
            return lambda msg: received.setdefault(key, []).append(msg)
        for topic, kind in (('depth/image_rect', Image), ('depth/camera_info', CameraInfo),
                            ('left/image_rect_color', Image), ('depth/colorized', Image)):
            io.create_subscription(kind, topic, keep(topic), 10)
        compressed = transport == 'compressed'
        image_type = CompressedImage if compressed else Image
        suffix = '/compressed' if compressed else ''
        pubs = {eye: (io.create_publisher(image_type, f'{eye}/image_raw{suffix}', 10),
                      io.create_publisher(CameraInfo, f'{eye}/camera_info', 10)) for eye in ('left', 'right')}
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(io)
        assert wait_for(executor, lambda: all(pub.get_subscription_count() for pair in pubs.values() for pub in pair)
                        and node.pub_color.get_subscription_count() and node.pub_depth.get_subscription_count())

        images = dict(zip(('left', 'right'), synthetic_pair()))
        eyes = dict(zip(('left', 'right'), ideal_eyes()))
        header = Header(stamp=node.get_clock().now().to_msg(), frame_id='zedx_nano_left_camera_optical_frame')
        for eye in ('left', 'right'):
            pubs[eye][1].publish(info_msg(eyes[eye], header))
        assert wait_for(executor, lambda: node._infos['left'] is not None and node._infos['right'] is not None)
        for eye in ('left', 'right'):
            if compressed:
                msg = CompressedImage(header=header, format='bgr8; jpeg compressed bgr8')
                msg.data = cv2.imencode('.jpg', images[eye], [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
            else:
                msg = bridge.cv2_to_imgmsg(images[eye], encoding='bgr8')
                msg.header = header
            pubs[eye][0].publish(msg)
        topics = ('depth/image_rect', 'depth/camera_info', 'left/image_rect_color', 'depth/colorized')
        assert wait_for(executor, lambda: all(received.get(topic) for topic in topics)), sorted(received)
        assert node.fatal is None

        depth_msg, rect_msg, info = received['depth/image_rect'][0], received['left/image_rect_color'][0], received['depth/camera_info'][0]
        for msg in (depth_msg, rect_msg, info, received['depth/colorized'][0]):
            assert msg.header.stamp == header.stamp and msg.header.frame_id == header.frame_id
        assert depth_msg.encoding == '16UC1' and rect_msg.encoding == 'bgr8'
        depth = bridge.imgmsg_to_cv2(depth_msg)
        assert depth.shape == (H, W) and depth.dtype == np.uint16
        assert np.median(valid_center(depth)) == pytest.approx(EXPECTED_MM, rel=0.08)
        assert (info.width, info.height) == (W, H) and info.k[0] == FX and info.p[3] == 0.0
        rect = bridge.imgmsg_to_cv2(rect_msg)
        if not compressed:   # ideal rig: rectification is the identity up to interpolation
            assert np.abs(rect.astype(int) - images['left'].astype(int))[10:-10, 10:-10].mean() < 1.0
        node.stop()
        node.destroy_node()
        io.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_report_says_idle_once_while_no_pairs_arrive(monkeypatch):
    records = []

    class Logger:
        def __getattr__(self, level):
            return lambda message, **kwargs: records.append((level, message))
    monkeypatch.setattr(NanoDepthNode, 'get_logger', lambda self: Logger())
    rclpy.init()
    try:
        node = NanoDepthNode(namespace='/zedx', parameter_overrides=[Parameter('backend', value='sgbm')])

        def report():
            del records[:]
            node._report()
            # the worker's own "Waiting for left/right camera_info" is not part of the report
            return [record for record in records if not record[1].startswith('Waiting for')]
        idle = [('info', 'No stereo input on /zedx/left/image_raw/compressed; idle (camera inactive?)')]
        assert report() == idle                      # names the topic, so a wrong namespace shows
        assert report() == []                        # said once per idle spell
        header = Header(stamp=node.get_clock().now().to_msg())
        node._on_image(0, CompressedImage(header=header))
        node._on_image(1, CompressedImage(header=header))
        [(level, message)] = report()                # pairs arrived, no depth (no camera_info): still a warning
        assert level == 'warning' and message.startswith('No depth computed in the last 30 s')
        assert '/zedx/left/image_raw/compressed' in message
        assert report() == idle                      # input stopped again
        for eye in (0, 1):                           # one eye only: a warning every period, not idle
            for _ in range(2):
                node._on_image(eye, CompressedImage(header=Header(stamp=node.get_clock().now().to_msg())))
            [(level, message)] = report()
            assert level == 'warning' and message.startswith('2 images but no stereo pair in the last 30 s')
            assert '/zedx/left/image_raw/compressed and /zedx/right/image_raw/compressed' in message
        assert report() == idle
        node._frames, node._last_depth = 3, np.full((4, 4), 500, np.uint16)
        node._pairs_received, node._images_received = 3, 6
        [(level, message)] = report()
        assert level == 'info' and 'depth frames/s' in message and 'median 500 mm' in message
        assert report() == idle
        node.stop()
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_node_rejects_unknown_transport():
    rclpy.init()
    try:
        with pytest.raises(ValueError, match='input_transport'):
            NanoDepthNode(parameter_overrides=[Parameter('backend', value='sgbm'),
                                               Parameter('input_transport', value='theora')])
    finally:
        rclpy.try_shutdown()
