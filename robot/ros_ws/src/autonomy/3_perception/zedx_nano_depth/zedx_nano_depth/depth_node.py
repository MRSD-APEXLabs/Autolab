"""ROS 2 node turning a ZED X Nano (or ZED X) stereo pair into a depth image.

Subscribes, relative to the node namespace (`zedx_nano` in the launch file):

  left/image_raw/compressed, right/image_raw/compressed   input_transport: compressed (default)
  left/image_raw, right/image_raw                         input_transport: raw
  left/camera_info, right/camera_info                     K, D, R, P per eye, as zedx_nano_camera publishes

Publishes, with the stamp and frame of the left image:

  depth/image_rect       sensor_msgs/Image 16UC1: depth in millimetres, 0 = invalid, aligned to left/image_rect_color
  depth/camera_info      rectified pinhole model shared by depth/image_rect and left/image_rect_color
  left/image_rect_color  sensor_msgs/Image bgr8: the rectified left image the depth belongs to
  depth/colorized        bgr8 preview (near red, far blue, invalid black), computed only while subscribed

Left and right are paired by exact stamp. The newest pair is processed on a worker thread and
pairs that arrive meanwhile are dropped, not queued, so latency stays at one compute period.
The node exits if the depth backend fails (e.g. its GPU worker process dies). Without input
(the camera hub streams another camera) it idles and resumes with the next pair.
"""
from functools import partial
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from .stereo import StereoMatcher, StereoRectifier, colorize_depth, depth_stats, make_backend
from .sync import PairBuffer

QOS = QoSProfile(depth=2)
TRANSPORTS = ('compressed', 'raw')


def camera_info_msg(fields):
    msg = CameraInfo()
    msg.width, msg.height = fields['width'], fields['height']
    msg.distortion_model = fields['distortion_model']
    msg.d, msg.k, msg.r, msg.p = fields['d'], fields['k'], fields['r'], fields['p']
    return msg


def calibration_key(left, right):
    return tuple((info.width, info.height, tuple(info.k), tuple(info.d), tuple(info.r), tuple(info.p))
                 for info in (left, right))


class NanoDepthNode(Node):
    def __init__(self, **kwargs):
        # kwargs go to rclpy Node (namespace, parameter_overrides, context, ...) for embedding and tests
        super().__init__('zedx_nano_depth', **kwargs)
        backend = self.declare_parameter('backend', 'neural').value
        model = self.declare_parameter('model', 'raft-realtime').value
        repo = self.declare_parameter('repo', '').value
        models_dir = self.declare_parameter('models_dir', '').value
        self.match_width = int(self.declare_parameter('match_width', 640).value)
        self.max_depth_mm = float(self.declare_parameter('max_depth_mm', 4000.0).value)
        transport = self.declare_parameter('input_transport', 'compressed').value
        self.colorize_range = (float(self.declare_parameter('colorize_near_mm', 100.0).value),
                               float(self.declare_parameter('colorize_far_mm', 1000.0).value))
        if transport not in TRANSPORTS:
            raise ValueError(f'input_transport must be one of {TRANSPORTS}, got {transport!r}')

        self.get_logger().info(f'Loading the {backend} depth backend'
                               + (' (missing weights are downloaded on the first start)' if backend == 'neural' else ''))
        self.backend = make_backend(backend, model=model, repo=repo or None, models_dir=models_dir or None)
        self.get_logger().info(f'Depth backend ready: {getattr(self.backend, "describe", lambda: {"backend": backend})()}')

        self.bridge = CvBridge()
        self.pub_rect = self.create_publisher(Image, 'left/image_rect_color', QOS)
        self.pub_depth = self.create_publisher(Image, 'depth/image_rect', QOS)
        self.pub_info = self.create_publisher(CameraInfo, 'depth/camera_info', QOS)
        self.pub_color = self.create_publisher(Image, 'depth/colorized', QOS)
        self._infos = {'left': None, 'right': None}
        self._pairs = PairBuffer()
        self._pending = None
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._matcher = self._key = self._rect_info = None
        self._frames, self._compute_s, self._last_depth = 0, 0.0, None
        self._images_received, self._pairs_received, self._idle = 0, 0, False
        self.fatal = None

        compressed = transport == 'compressed'
        self._decode = self._decode_compressed if compressed else self._decode_raw
        self._inputs = []   # resolved image topics (namespace and remaps applied), named in the report
        for index, eye in enumerate(('left', 'right')):
            self.create_subscription(CameraInfo, f'{eye}/camera_info', partial(self._on_info, eye), QOS)
            self._inputs.append(self.create_subscription(CompressedImage if compressed else Image,
                                                         f'{eye}/image_raw' + ('/compressed' if compressed else ''),
                                                         partial(self._on_image, index), QOS).topic_name)
        self._thread = threading.Thread(target=self._work, name='nano-depth', daemon=True)
        self._thread.start()
        self.create_timer(30.0, self._report)

    # -- inputs -------------------------------------------------------------------------------
    def _on_info(self, eye, msg):
        self._infos[eye] = msg

    def _on_image(self, index, msg):
        self._images_received += 1
        pair = self._pairs.add(index, (msg.header.stamp.sec, msg.header.stamp.nanosec), msg)
        if pair is not None:
            self._pairs_received += 1
            with self._cond:        # single slot: the worker always takes the newest pair
                self._pending = pair
                self._cond.notify()

    @staticmethod
    def _decode_compressed(msg):
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError(f'Undecodable compressed image ({msg.format!r})')
        return bgr

    def _decode_raw(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # -- depth --------------------------------------------------------------------------------
    def _new_rectifier(self, left_info, right_info, shape):
        """(key, StereoRectifier) if the calibration changed, else (key, None); raises if it does not fit the image."""
        key = calibration_key(left_info, right_info)
        rectifier = StereoRectifier.from_camera_info(left_info, right_info) if key != self._key else None
        width, height = (rectifier or self._matcher.calibration).size
        if shape[:2] != (height, width):
            raise ValueError(f'Image is {shape[1]}x{shape[0]} but camera_info is {width}x{height}')
        return key, rectifier

    def _set_rectifier(self, key, rectifier):
        # the backend is shared and stays open; StereoMatcher warms it up at the new matching size
        self._matcher = StereoMatcher(rectifier, match_width=self.match_width, max_depth_mm=self.max_depth_mm,
                                      backend=self.backend)
        self._rect_info = camera_info_msg(rectifier.rectified_camera_info_fields())
        self._key = key
        self.get_logger().info(f'Rectifying {rectifier.size[0]}x{rectifier.size[1]} (fx {rectifier.fx:.1f} px, '
                               f'baseline {rectifier.baseline_mm:.2f} mm), matching at '
                               f'{self._matcher.match_size[0]}x{self._matcher.match_size[1]}')

    def _work(self):
        while not self._stop.is_set():
            with self._cond:
                while self._pending is None and not self._stop.is_set():
                    self._cond.wait(0.5)
                pending, self._pending = self._pending, None
            if pending is None:
                continue
            left_msg, right_msg = pending
            left_info, right_info = self._infos['left'], self._infos['right']
            if left_info is None or right_info is None:
                self.get_logger().warning('Waiting for left/right camera_info', throttle_duration_sec=10.0)
                continue
            try:
                left, right = self._decode(left_msg), self._decode(right_msg)
                key, rectifier = self._new_rectifier(left_info, right_info, left.shape)
            except Exception as exc:
                self.get_logger().error(f'Skipping stereo pair: {exc}', throttle_duration_sec=10.0)
                continue
            try:
                if rectifier is not None:
                    self._set_rectifier(key, rectifier)
                t0 = time.monotonic()
                left_rect, depth = self._matcher.compute(left, right)
                self._compute_s += time.monotonic() - t0
            except Exception as exc:
                self.fatal = exc
                self.get_logger().fatal(f'Depth backend failed: {exc}')
                return
            try:
                self._publish(left_msg.header, left_rect, depth)
            except Exception as exc:
                if self._stop.is_set() or not self.context.ok():
                    return   # Ctrl-C shuts the context down before stop() runs
                self.get_logger().error(f'Publishing depth failed: {exc}', throttle_duration_sec=10.0)
                continue
            self._frames += 1
            self._last_depth = depth

    def _publish(self, header, left_rect, depth):
        rect = self.bridge.cv2_to_imgmsg(left_rect, encoding='bgr8')
        rect.header = header
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='16UC1')
        depth_msg.header = header
        info = self._rect_info
        info.header = header
        self.pub_rect.publish(rect)
        self.pub_depth.publish(depth_msg)
        self.pub_info.publish(info)
        if self.pub_color.get_subscription_count():
            color = self.bridge.cv2_to_imgmsg(colorize_depth(depth, *self.colorize_range), encoding='bgr8')
            color.header = header
            self.pub_color.publish(color)

    def _report(self):
        frames, compute_s, depth = self._frames, self._compute_s, self._last_depth
        images, pairs = self._images_received, self._pairs_received
        self._frames, self._compute_s, self._images_received, self._pairs_received = 0, 0.0, 0, 0
        if images:
            self._idle = False
        if frames and depth is not None:
            stats = depth_stats(depth)
            self.get_logger().info(f'{frames / 30.0:.1f} depth frames/s, {1000 * compute_s / frames:.0f} ms each; '
                                   f'last frame {100 * stats["valid_fraction"]:.0f}% valid, median {stats["median_mm"]} mm')
        elif pairs:
            self.get_logger().warning(f'No depth computed in the last 30 s; is the stereo feed ({self._inputs[0]}, '
                                      '.../camera_info) publishing?')
        elif images:
            # images that never pair up: one eye only (camera node sensor left/right) or unequal stamps
            self.get_logger().warning(f'{images} images but no stereo pair in the last 30 s on {self._inputs[0]} '
                                      f'and {self._inputs[1]}; both eyes must arrive with equal stamps')
        elif not self._idle:
            # no input at all: normal while the camera hub streams another camera, so say it once, but
            # name the topic so a wrong namespace or remap is not mistaken for an inactive camera
            self._idle = True
            self.get_logger().info(f'No stereo input on {self._inputs[0]}; idle (camera inactive?)')

    def stop(self):
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        self._thread.join(timeout=5)
        close = getattr(self.backend, 'close', None)
        if close is not None:
            close()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = NanoDepthNode()
    except Exception as exc:
        rclpy.try_shutdown()
        raise SystemExit(f'zedx_nano_depth: {exc}') from exc
    try:
        while rclpy.ok() and node.fatal is None:
            try:
                rclpy.spin_once(node, timeout_sec=0.5)
            except Exception:
                if rclpy.ok():
                    raise
                # Ctrl-C shut the context down during spin_once (wait set or message take), not a failure
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    if node.fatal is not None:
        raise SystemExit(f'zedx_nano_depth: depth backend failed: {node.fatal}')


if __name__ == '__main__':
    main()
