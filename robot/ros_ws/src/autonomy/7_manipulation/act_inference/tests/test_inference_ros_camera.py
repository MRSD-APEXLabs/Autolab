import os
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from act_inference.ros_camera import RosDepthCamera, frame_from_messages, image_to_array


def fake_image(array, encoding, stamp=(5, 0), pad=0, big_endian=False):
    height, width = array.shape[:2]
    dtype = array.dtype.newbyteorder('>' if big_endian else '<')
    rows = np.ascontiguousarray(array.astype(dtype)).view(np.uint8).reshape(height, -1)
    rows = np.hstack([rows, np.full((height, pad), 7, np.uint8)])
    return SimpleNamespace(encoding=encoding, height=height, width=width, step=rows.shape[1], is_bigendian=big_endian,
                           data=rows.tobytes(), header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp[0], nanosec=stamp[1]),
                                                                       frame_id='cam'))


@pytest.mark.parametrize('big_endian', [False, True])
def test_image_to_array_honours_step_and_endianness(big_endian):
    depth = np.arange(12, dtype=np.uint16).reshape(3, 4) * 1000
    np.testing.assert_array_equal(image_to_array(fake_image(depth, '16UC1', pad=6, big_endian=big_endian)), depth)
    bgr = np.random.RandomState(0).randint(0, 255, (3, 4, 3)).astype(np.uint8)
    np.testing.assert_array_equal(image_to_array(fake_image(bgr, 'bgr8', pad=4)), bgr)
    with pytest.raises(ValueError, match='encoding'):
        image_to_array(fake_image(bgr, '32FC1'))
    truncated = fake_image(bgr, 'bgr8')
    truncated.data = truncated.data[:-1]
    with pytest.raises(ValueError, match='truncated'):
        image_to_array(truncated)


def test_frame_from_messages():
    bgr = np.zeros((3, 4, 3), np.uint8)
    bgr[..., 0] = 200   # blue
    depth = np.full((3, 4), 450, np.uint16)
    now_ns, mono = 5_100_000_000, 50.0
    frame = frame_from_messages(fake_image(bgr, 'bgr8'), fake_image(depth, '16UC1'), now_ns, mono, 3)
    assert frame.image[0, 0].tolist() == [0, 0, 200] and frame.depth[0, 0] == 450
    assert frame.header['frame_id'] == 3 and frame.header['t_image_ns'] == 5_000_000_000
    assert frame.captured_monotonic == pytest.approx(49.9)
    with pytest.raises(ValueError, match='16UC1'):
        frame_from_messages(fake_image(bgr, 'bgr8'), fake_image(bgr, 'bgr8'), now_ns, mono, 0)
    with pytest.raises(ValueError, match='does not match'):
        frame_from_messages(fake_image(bgr, 'bgr8'), fake_image(depth[:2], '16UC1'), now_ns, mono, 0)


def test_messages_are_paired_by_stamp_and_must_move_forward():
    cam = RosDepthCamera()
    bgr, depth = np.zeros((3, 4, 3), np.uint8), np.ones((3, 4), np.uint16)
    now = time.time_ns()
    stamp = (now // 10**9, now % 10**9)
    cam._on_message(1, fake_image(depth, '16UC1', stamp))
    assert cam._latest is None
    cam._on_message(0, fake_image(bgr, 'bgr8', stamp))
    assert cam._latest is not None and cam.frames_received == 1 and cam._error is None
    older = (stamp[0] - 1, stamp[1])
    cam._on_message(0, fake_image(bgr, 'bgr8', older))
    cam._on_message(1, fake_image(depth, '16UC1', older))
    with pytest.raises(RuntimeError, match='backwards'):
        cam.check_mode()


def test_stale_frames_are_refused():
    cam = RosDepthCamera(max_age=0.25)
    now = time.time_ns() - 600_000_000
    stamp = (now // 10**9, now % 10**9)
    cam._on_message(0, fake_image(np.zeros((3, 4, 3), np.uint8), 'bgr8', stamp))
    cam._on_message(1, fake_image(np.ones((3, 4), np.uint16), '16UC1', stamp))
    with pytest.raises(RuntimeError, match='stale'):
        cam.latest()


def test_live_topics_from_a_publisher_node():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import CameraInfo, Image
    namespace = f'/test_act_inference_{os.getpid()}'
    rclpy.init()
    node = rclpy.create_node('fake_depth', namespace=namespace)
    pubs = (node.create_publisher(Image, 'left/image_rect_color', 10), node.create_publisher(Image, 'depth/image_rect', 10),
            node.create_publisher(CameraInfo, 'depth/camera_info', 10))
    bgr = np.zeros((6, 8, 3), np.uint8)
    bgr[..., 2] = 255   # red
    depth = np.full((6, 8), 321, np.uint16)
    running = threading.Event()
    running.set()

    def publish():
        stamp = node.get_clock().now().to_msg()
        if not running.is_set():
            return
        for pub, array, encoding in ((pubs[0], bgr, 'bgr8'), (pubs[1], depth, '16UC1')):
            msg = Image(height=6, width=8, encoding=encoding, step=array.strides[0])
            msg.header.stamp, msg.header.frame_id = stamp, 'cam'
            msg.data = array.tobytes()
            pub.publish(msg)
        info = CameraInfo(width=8, height=6)
        info.header.stamp = stamp
        info.k = [4.0, 0.0, 4.0, 0.0, 4.0, 3.0, 0.0, 0.0, 1.0]
        pubs[2].publish(info)
    node.create_timer(0.05, publish)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    cam = RosDepthCamera(namespace, max_age=0.25, start_timeout=10)
    try:
        cam.start()
        frame = cam.latest()
        assert frame.image[0, 0].tolist() == [255, 0, 0] and frame.depth[0, 0] == 321
        assert cam.calibration['rectified'] and cam.calibration['left']['fx'] == 4.0 and cam.calibration['resolution'] == [8, 6]
        time.sleep(0.3)
        assert cam.frames_received > 2 and 0.02 < cam.interval_s < 0.25
        cam.check_mode()
        running.clear()          # publisher stalls: frames must go stale
        time.sleep(0.8)
        with pytest.raises(RuntimeError, match='stale'):
            cam.latest()
    finally:
        cam.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    assert not cam._thread.is_alive()
