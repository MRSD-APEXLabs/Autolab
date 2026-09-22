"""ROS 2 node serving the camera UI: switch the Xavier's camera hub between its cameras, see RGB and depth.

Subscribes, for each camera in `cameras` (absolute topics, as launch/cameras.launch.xml names them):

  /<cam>/left/image_rect_color       sensor_msgs/Image bgr8, rectified left image (zedx_nano_depth)
  /<cam>/depth/image_rect            sensor_msgs/Image 16UC1, mm, 0 = invalid, aligned to it
  /<cam>/depth/camera_info           their rectified intrinsics
  /<cam>/left/image_raw/compressed   the camera node's JPEG, shown while the rectified image is stale

plus <hub_node>/status (std_msgs/String, JSON) from zedx_nano_camera's camera_hub node, and
switches cameras with its std_srvs/Trigger services <hub_node>/select_<cam> and <hub_node>/release.
The page is served on http://<host>:<port>/ (see web.py). Callbacks only keep the newest message
of each stream; the HTTP threads encode images on demand.
"""
from functools import partial
import re
import socket
import threading

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .frames import FrameStore, Renderer
from .web import CameraUIApp, make_server

QOS = QoSProfile(depth=2)   # as the camera and depth nodes publish
STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# s to wait for the hub node's answer to a switch. It answers within its select_timeout (30 s) plus the HTTP
# timeout (3 s) of the POST, then polls the status (3 s), possibly after a poll already running (3 s): 39 s.
SELECT_TIMEOUT = 45.0
SERVICE_WAIT = 2.0          # s to discover the hub node's services before reporting it as not running
DEPTH_ENCODINGS = ('16UC1', 'mono16')


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def intrinsics(info):
    """Rectified pinhole intrinsics of a CameraInfo (its projection matrix P)."""
    return {'fx': info.p[0], 'fy': info.p[5], 'cx': info.p[2], 'cy': info.p[6],
            'width': int(info.width), 'height': int(info.height)}


class CameraUINode(Node):
    def __init__(self, **kwargs):
        # kwargs go to rclpy Node (namespace, parameter_overrides, context, ...) for embedding and tests
        super().__init__('camera_ui', **kwargs)
        host = self.declare_parameter('host', '0.0.0.0').value
        port = int(self.declare_parameter('port', 8001).value)
        cameras = list(self.declare_parameter('cameras', ['zedx', 'zedx_nano']).value)
        labels = list(self.declare_parameter('labels', ['ZED X (base)', 'ZED X Nano (wrist)']).value)
        max_fps = float(self.declare_parameter('max_fps', 15.0).value)
        jpeg_quality = int(self.declare_parameter('jpeg_quality', 80).value)
        stale_after = float(self.declare_parameter('stale_after', 1.0).value)
        hub = '/' + self.declare_parameter('hub_node', '/camera_hub').value.strip('/')
        bad = [camera for camera in cameras if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', camera)]
        if not cameras or bad:
            raise ValueError(f'cameras must be non-empty ROS names, got {cameras!r}')

        self.bridge = CvBridge()
        self.store = FrameStore(cameras, stale_after)
        # a missing label falls back to the camera name
        self.app = CameraUIApp(Renderer(self.store, jpeg_quality), dict(zip(cameras, labels)),
                               select=self._select, max_fps=max_fps)
        for camera in cameras:
            for stream, kind, topic in (('rect', Image, 'left/image_rect_color'), ('depth', Image, 'depth/image_rect'),
                                        ('info', CameraInfo, 'depth/camera_info'),
                                        ('raw', CompressedImage, 'left/image_raw/compressed')):
                self.create_subscription(kind, f'/{camera}/{topic}', partial(self._on_message, camera, stream), QOS)
        self.create_subscription(String, f'{hub}/status', self._on_hub_status, STATUS_QOS)
        self._select_clients = {camera: self.create_client(Trigger, f'{hub}/select_{camera}') for camera in cameras}
        self._select_clients[None] = self.create_client(Trigger, f'{hub}/release')

        self.server = make_server(self.app, host, port)
        self._thread = threading.Thread(target=self.server.serve_forever, name='camera-ui-http', daemon=True)
        self._thread.start()
        shown = socket.gethostname() if host in ('0.0.0.0', '::', '') else host
        self.get_logger().info(f'Camera UI on http://{shown}:{self.server.server_port}/ for {", ".join(cameras)} '
                               f'(camera hub node {hub})')

    # -- inputs -------------------------------------------------------------------------------
    def _on_message(self, camera, stream, msg):
        try:
            if stream == 'rect':
                data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif stream == 'depth':
                if msg.encoding not in DEPTH_ENCODINGS:
                    raise ValueError(f'expected 16UC1 depth in mm, got {msg.encoding!r}')
                data = self.bridge.imgmsg_to_cv2(msg)
            elif stream == 'info':
                data = intrinsics(msg)
            else:
                data = bytes(msg.data)
        except Exception as exc:
            self.get_logger().warning(f'Ignoring a {stream} message of {camera}: {exc}', throttle_duration_sec=30.0)
            return
        self.store.update(camera, stream, data, stamp_ns(msg.header.stamp))

    def _on_hub_status(self, msg):
        try:
            self.app.set_hub_status(msg.data)
        except ValueError as exc:
            self.get_logger().warning(f'Ignoring an invalid camera hub status: {exc}', throttle_duration_sec=30.0)

    # -- switching (HTTP threads) ---------------------------------------------------------------
    def _select(self, camera):
        """Call the hub node's select_<camera> (release for None) service; (HTTP status, {'ok', 'message'})."""
        client = self._select_clients[camera]
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT):
            return 503, {'ok': False, 'message': 'camera_hub node is not running'}
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not future.done() and not done.wait(SELECT_TIMEOUT):
            client.remove_pending_request(future)
            return 504, {'ok': False, 'message': f'no reply from {client.srv_name} within {SELECT_TIMEOUT:.0f} s'}
        result = future.result()
        return 200, {'ok': bool(result.success), 'message': result.message}

    def stop(self):
        self.app.close()
        self.server.shutdown()
        self.server.server_close()
        self._thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraUINode()
    except Exception as exc:   # e.g. the port is taken
        rclpy.try_shutdown()
        raise SystemExit(f'camera_ui: {exc}') from exc
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
