"""The UI node on synthetic camera/depth topics, a fake camera_hub status and fake Trigger services (needs ROS 2)."""
import json
import os
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from cv_bridge import CvBridge  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CameraInfo, CompressedImage, Image  # noqa: E402
from std_msgs.msg import Header, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from camera_ui import ui_node  # noqa: E402
from camera_ui.ui_node import CameraUINode  # noqa: E402
from ui_fakes import FX, H, W, decode, depth_ramp, hub_status, jpeg, open_stream, read_parts, request, rgb_image  # noqa: E402


def get_json(port, path):
    status, _, body = request(port, 'GET', path)
    return status, json.loads(body)


def post_select(port, camera):
    status, _, body = request(port, 'POST', '/api/select', json.dumps({'camera': camera}), timeout=50.0)
    return status, json.loads(body)


def wait_until(condition, timeout=15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = condition()
        if result:
            return result
        time.sleep(0.1)
    return condition()


class FakeCameraStack:
    """Publishes what the camera and depth nodes and the camera_hub node would, and serves the hub's Trigger services.

    `cameras[0]` has a depth node (rectified image + depth), `cameras[1]` only its camera node (raw JPEG);
    only cameras[0] has a select service.
    """

    def __init__(self, cameras, hub):
        self.node = rclpy.create_node('fake_camera_stack')
        self.bridge = CvBridge()
        self.calls = []
        qos = QoSProfile(depth=2)
        with_depth, raw_only = self.cameras = cameras
        self.pubs = {'rect': self.node.create_publisher(Image, f'/{with_depth}/left/image_rect_color', qos),
                     'depth': self.node.create_publisher(Image, f'/{with_depth}/depth/image_rect', qos),
                     'info': self.node.create_publisher(CameraInfo, f'/{with_depth}/depth/camera_info', qos),
                     'raw': self.node.create_publisher(CompressedImage, f'/{raw_only}/left/image_raw/compressed', qos)}
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.node.create_publisher(String, f'{hub}/status', status_qos)
        self.node.create_service(Trigger, f'{hub}/select_{with_depth}', self._trigger(with_depth))
        self.node.create_service(Trigger, f'{hub}/release', self._trigger('release'))
        self.image, self.depth, self.jpeg = rgb_image(), depth_ramp(400, 1600), jpeg(rgb_image())
        self.node.create_timer(0.05, self.publish)

    def _trigger(self, name):
        def serve(request, response):
            self.calls.append(name)
            response.success, response.message = True, f'{name}: done'
            return response
        return serve

    def publish(self):
        header = Header(stamp=self.node.get_clock().now().to_msg(), frame_id=f'{self.cameras[0]}_left_camera_optical_frame')
        rect = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        depth = self.bridge.cv2_to_imgmsg(self.depth, encoding='16UC1')
        rect.header = depth.header = header
        info = CameraInfo(header=header, width=W, height=H, distortion_model='plumb_bob')
        info.k = [FX, 0.0, W / 2, 0.0, FX, H / 2, 0.0, 0.0, 1.0]
        info.p = [FX, 0.0, W / 2, 0.0, 0.0, FX, H / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        raw = CompressedImage(header=header, format='bgr8; jpeg compressed bgr8')
        raw.data = self.jpeg
        for stream, msg in (('rect', rect), ('depth', depth), ('info', info), ('raw', raw)):
            self.pubs[stream].publish(msg)


def test_node_shows_the_cameras_and_switches_through_the_hub_services():
    prefix = f'ui_test_{os.getpid()}'
    cameras = [f'{prefix}_zedx', f'{prefix}_nano']
    hub = f'/{prefix}_hub'
    rclpy.init()
    executor = MultiThreadedExecutor()
    try:
        node = CameraUINode(parameter_overrides=[
            Parameter('host', value='127.0.0.1'), Parameter('port', value=0), Parameter('cameras', value=cameras),
            Parameter('labels', value=['Base']), Parameter('hub_node', value=hub)])
        fake = FakeCameraStack(cameras, hub)
        fake.status.publish(String(data=json.dumps(hub_status(active=cameras[0]))))
        executor.add_node(node)
        executor.add_node(fake.node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        port = node.server.server_port

        def ready():
            body = get_json(port, '/api/status')[1]
            cams = body['cameras']
            return body if (body['hub'] and cams[cameras[0]]['depth']['median_mm'] and cams[cameras[0]]['intrinsics']
                            and cams[cameras[1]]['rgb']['source']) else None
        status = wait_until(ready)
        assert status, get_json(port, '/api/status')[1]
        assert status['hub']['active'] == cameras[0] and status['hub']['reachable'] is True
        base, wrist = status['cameras'][cameras[0]], status['cameras'][cameras[1]]
        assert base['label'] == 'Base' and wrist['label'] == cameras[1]   # missing label: the camera name
        assert base['rgb']['source'] == 'rectified' and (base['rgb']['width'], base['rgb']['height']) == (W, H)
        assert base['depth']['median_mm'] == pytest.approx(1000, abs=2) and base['intrinsics']['fx'] == FX
        assert wrist['rgb']['source'] == 'raw' and wrist['depth']['age_s'] is None
        assert wait_until(lambda: get_json(port, '/api/status')[1]['cameras'][cameras[0]]['rgb']['fps'] > 5)

        with open_stream(port, f'/stream/{cameras[0]}/rgb.mjpg') as response:
            assert np.abs(decode(read_parts(response, 1)[0]).astype(int) - fake.image).mean() < 3
        with open_stream(port, f'/stream/{cameras[0]}/depth.mjpg?auto=1') as response:
            assert decode(read_parts(response, 1)[0]).shape == (H, W, 3)
        with open_stream(port, f'/stream/{cameras[1]}/rgb.mjpg') as response:
            assert read_parts(response, 1)[0] == fake.jpeg

        probe = get_json(port, f'/api/depth_at?camera={cameras[0]}&u=0.5&v=0.5')[1]
        assert probe['depth_mm'] == pytest.approx(1000, abs=3) and probe['xyz_mm'][2] == probe['depth_mm']

        assert post_select(port, cameras[0]) == (200, {'ok': True, 'message': f'{cameras[0]}: done'})
        assert post_select(port, None) == (200, {'ok': True, 'message': 'release: done'})
        assert fake.calls == [cameras[0], 'release']
        assert post_select(port, cameras[1]) == (503, {'ok': False, 'message': 'camera_hub node is not running'})
        assert post_select(port, 'zedx_mini')[0] == 400

        executor.shutdown()
        spinner.join(timeout=5)
        node.stop()
        node.destroy_node()
        fake.node.destroy_node()
    finally:
        executor.shutdown()
        rclpy.try_shutdown()


class SilentHub:
    """camera_hub Trigger services whose selects answer only once `answer` is set, like a camera that never delivers.

    Release answers at once. The services are reentrant, as the camera_hub node's are.
    """

    def __init__(self, cameras, hub):
        self.node = rclpy.create_node('fake_silent_hub')
        self.answer = threading.Event()
        self.calls = []
        group = ReentrantCallbackGroup()
        for camera in cameras:
            self.node.create_service(Trigger, f'{hub}/select_{camera}', self._serve(camera), callback_group=group)
        self.node.create_service(Trigger, f'{hub}/release', self._serve(None), callback_group=group)

    def _serve(self, camera):
        def serve(request, response):
            self.calls.append(camera)
            if camera is not None:
                self.answer.wait(20)
            response.success, response.message = True, f'{camera or "release"}: done'
            return response
        return serve


def test_node_gives_up_on_a_silent_hub_node_and_lets_a_release_through(monkeypatch):
    prefix = f'ui_silent_{os.getpid()}'
    cameras = [f'{prefix}_zedx', f'{prefix}_nano']
    hub = f'/{prefix}_hub'
    rclpy.init()
    executor = MultiThreadedExecutor()
    fake = None
    try:
        node = CameraUINode(parameter_overrides=[
            Parameter('host', value='127.0.0.1'), Parameter('port', value=0), Parameter('cameras', value=cameras),
            Parameter('hub_node', value=hub)])
        fake = SilentHub(cameras, hub)
        executor.add_node(node)
        executor.add_node(fake.node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        port = node.server.server_port

        # no reply within SELECT_TIMEOUT: 504, and the request is dropped
        monkeypatch.setattr(ui_node, 'SELECT_TIMEOUT', 1.0)
        start = time.monotonic()
        status, reply = post_select(port, cameras[0])
        assert status == 504 and reply == {'ok': False, 'message': f'no reply from {hub}/select_{cameras[0]} within 1 s'}
        assert time.monotonic() - start < 4.0   # plus the service discovery
        assert get_json(port, '/api/status')[1]['switching'] is None

        # a release is not queued behind a switch that waits
        monkeypatch.setattr(ui_node, 'SELECT_TIMEOUT', 20.0)
        replies = []
        waiting = threading.Thread(target=lambda: replies.append(post_select(port, cameras[1])))
        waiting.start()
        assert wait_until(lambda: (get_json(port, '/api/status')[1]['switching'] or {}).get('camera') == cameras[1])
        start = time.monotonic()
        assert post_select(port, None) == (200, {'ok': True, 'message': 'release: done'})
        assert time.monotonic() - start < 2.0 and waiting.is_alive()
        fake.answer.set()
        waiting.join(10)
        assert replies == [(200, {'ok': True, 'message': f'{cameras[1]}: done'})]
        assert post_select(port, cameras[0]) == (200, {'ok': True, 'message': f'{cameras[0]}: done'})   # after a timeout too
        assert fake.calls == [cameras[0], cameras[1], None, cameras[0]]

        # stop() ends the open MJPEG streams (this one waits for a first frame that never comes)
        with open_stream(port, f'/stream/{cameras[0]}/rgb.mjpg') as response:
            executor.shutdown()
            spinner.join(timeout=5)
            start = time.monotonic()
            node.stop()
            assert time.monotonic() - start < 3.0
            assert response.fp.read() == b''
        node.destroy_node()
        fake.node.destroy_node()
    finally:
        if fake is not None:
            fake.answer.set()   # frees the service threads
        executor.shutdown()
        rclpy.try_shutdown()


def test_node_rejects_bad_camera_names():
    rclpy.init()
    try:
        with pytest.raises(ValueError, match='cameras'):
            CameraUINode(parameter_overrides=[Parameter('cameras', value=['zedx', 'wrist cam'])])
    finally:
        rclpy.try_shutdown()
