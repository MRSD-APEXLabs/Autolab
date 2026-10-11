"""ROS 2 node serving image topics as MJPEG over HTTP, subscribed only while a browser watches them.

Parameters: host, port, streams (['<kind>/<camera>=<topic>', ...]), max_fps, jpeg_quality.
Routes (see web.py): /stream/<kind>/<camera>.mjpg and /health.

A 10 Hz timer on the executor reconciles subscriptions with StreamHub.wanted(): HTTP threads only
change viewer counts, so subscriptions are created and destroyed on the executor thread. Callbacks
keep the newest message; HTTP threads encode it to JPEG on demand.
"""
from functools import partial
import socket
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image

from .streams import StreamHub, parse_streams
from .web import make_server

QOS = QoSProfile(depth=2)   # as the camera, depth and perception nodes publish
DEFAULT_STREAMS = ['annotated/zedx=/inspect/annotated_image', 'annotated/zedx_nano=/servo/annotated_image',
                   'raw/zedx=/zedx/left/image_rect_color', 'raw/zedx_nano=/zedx_nano/left/image_rect_color']
RECONCILE_PERIOD = 0.1   # s between subscription updates


class ImageStreamServer(Node):
    def __init__(self, **kwargs):
        # kwargs go to rclpy Node (context, parameter_overrides, ...) for tests
        super().__init__('image_stream_server', **kwargs)
        host = self.declare_parameter('host', '0.0.0.0').value
        port = int(self.declare_parameter('port', 8002).value)
        streams = parse_streams(list(self.declare_parameter('streams', DEFAULT_STREAMS).value))
        max_fps = float(self.declare_parameter('max_fps', 15.0).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 80).value)

        self.bridge = CvBridge()
        self.hub = StreamHub(streams, self._encode)
        self._stream_subscriptions = {}
        self.create_timer(RECONCILE_PERIOD, self._reconcile)

        self.server = make_server(self.hub, host, port, max_fps=max_fps)
        self._thread = threading.Thread(target=self.server.serve_forever, name='image-stream-http', daemon=True)
        self._thread.start()
        shown = socket.gethostname() if host in ('0.0.0.0', '::', '') else host
        paths = ', '.join(f'/stream/{kind}/{camera}.mjpg' for kind, camera in streams)
        self.get_logger().info(f'Image streams on http://{shown}:{self.server.server_port}: {paths}')

    def _encode(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, buffer = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            raise ValueError('JPEG encoding failed')
        return buffer.tobytes()

    def _reconcile(self):
        wanted = self.hub.wanted()
        for key in wanted - set(self._stream_subscriptions):
            topic = self.hub.streams[key]
            self._stream_subscriptions[key] = self.create_subscription(
                Image, topic, partial(self.hub.update, key), QOS)
            self.get_logger().info(f'Watching {topic} ({key[0]}/{key[1]})')
        for key in set(self._stream_subscriptions) - wanted:
            self.destroy_subscription(self._stream_subscriptions.pop(key))
            self.get_logger().info(f'Released {self.hub.streams[key]} ({key[0]}/{key[1]})')
        self.hub.set_subscribed(self._stream_subscriptions)

    def stop(self):
        self.server.closed.set()
        self.server.shutdown()
        self.server.server_close()
        self._thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImageStreamServer()
    except Exception as exc:   # e.g. the port is taken or a bad streams entry
        rclpy.try_shutdown()
        raise SystemExit(f'image_stream_server: {exc}') from exc
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
