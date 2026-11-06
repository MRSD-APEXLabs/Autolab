"""Camera source that reads the zedx_nano_depth node's topics (--camera-source ros).

Subscribes to <namespace>/left/image_rect_color, <namespace>/depth/image_rect and
<namespace>/depth/camera_info, pairs image and depth by exact stamp and returns them as
CameraFrame, like NanoStreamCamera does from the raw stream. Use it when the ROS nodes already
run, so a single depth network serves the robot stack and the policy.

It uses its own rclpy context and executor thread, so it neither needs nor disturbs an rclpy
setup of the surrounding process. Stamps are the camera node's ROS time of capture; run it on
the same machine as the camera node, or keep both clocks synchronized (chrony/PTP).
"""
import threading
import time
import logging

import cv2
import numpy as np

from .cameras import CameraFrame

ENCODINGS = {'bgr8': (np.uint8, 3), 'rgb8': (np.uint8, 3), '16UC1': (np.uint16, 1), 'mono16': (np.uint16, 1)}


class StaleCameraFrame(RuntimeError):
    """A valid image/depth pair is too old to use; startup may wait for its replacement."""


class ImageDepthPairs:
    """Exact-stamp matching that preserves newer halves when messages arrive out of order."""

    def __init__(self, keep=8):
        self.keep = keep
        self.pending = {}
        self.last_stamp = -1
        self.lock = threading.Lock()

    def add(self, index, stamp, message):
        with self.lock:
            if stamp <= self.last_stamp:
                return None
            pair = self.pending.setdefault(stamp, [None, None])
            pair[index] = message
            while len(self.pending) > self.keep:
                del self.pending[min(self.pending)]
            if pair[0] is None or pair[1] is None:
                return None
            self.last_stamp = stamp
            self.pending = {key: value for key, value in self.pending.items() if key > stamp}
            return tuple(pair)


def image_to_array(msg):
    """numpy array (H, W[, C]) of a sensor_msgs/Image in one of ENCODINGS, honouring step and endianness."""
    if msg.encoding not in ENCODINGS:
        raise ValueError(f'Unsupported image encoding {msg.encoding!r}')
    dtype, channels = ENCODINGS[msg.encoding]
    dtype = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    height, width, step = int(msg.height), int(msg.width), int(msg.step)
    row_bytes = width * channels * dtype.itemsize
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if height <= 0 or width <= 0 or step < row_bytes or data.size < step * height:
        raise ValueError('Image message is truncated or has an invalid step')
    rows = data[:step * height].reshape(height, step)[:, :row_bytes]
    array = np.ascontiguousarray(rows).view(dtype).reshape((height, width, channels) if channels > 1 else (height, width))
    return array.astype(dtype.newbyteorder('='))


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def frame_from_messages(image_msg, depth_msg, received_ns, received_monotonic, index):
    """CameraFrame (RGB, uint16 depth mm) from a rectified image and the depth image aligned to it."""
    image = image_to_array(image_msg)
    if image_msg.encoding == 'bgr8':
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    if depth_msg.encoding not in ('16UC1', 'mono16'):
        raise ValueError(f'Expected 16UC1 depth in millimetres, got {depth_msg.encoding!r}')
    depth = image_to_array(depth_msg)
    if depth.shape != image.shape[:2]:
        raise ValueError('Depth image does not match the rectified image')
    captured_ns = stamp_ns(image_msg.header.stamp)
    header = {'type': 'ros_depth_frame', 'frame_id': index, 't_image_ns': captured_ns,
              'image_size': [image.shape[1], image.shape[0]], 'ros_frame_id': image_msg.header.frame_id}
    # Stamps are wall-clock (ROS time) of capture; anchor them to the local monotonic clock on receipt.
    return CameraFrame(image, depth, header, received_ns, received_monotonic,
                       received_monotonic - (received_ns - captured_ns) / 1e9)


class RosDepthCamera:
    """Rectified left RGB and depth from the zedx_nano_depth topics, with the NanoStreamCamera interface."""

    def __init__(self, namespace='/zedx_nano', max_age=0.25, start_timeout=10.0):
        self.namespace = '/' + namespace.strip('/') if namespace.strip('/') else ''
        self.max_age, self.start_timeout = max_age, start_timeout
        self.calibration = None
        self.uncertainty_ns = 0
        self.latency_s = self.interval_s = 0.0
        self.frames_received = 0
        self._last_received = None
        self._pairs = ImageDepthPairs()
        self.frames_dropped = 0
        self._last_drop_warning = float('-inf')
        self._last_publisher_check = float('-inf')
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest = None
        self._last_stamp = -1
        self._error = None
        self._context = self._node = self._executor = self._thread = None

    def _topic(self, name):
        return f'{self.namespace}/{name}'

    def start(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile
        from sensor_msgs.msg import CameraInfo, Image

        self._context = Context()
        rclpy.init(args=[], context=self._context)
        self._node = rclpy.create_node('act_inference_camera', context=self._context)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        qos = QoSProfile(depth=2)
        self._node.create_subscription(Image, self._topic('left/image_rect_color'), lambda msg: self._on_message(0, msg), qos)
        self._node.create_subscription(Image, self._topic('depth/image_rect'), lambda msg: self._on_message(1, msg), qos)
        self._node.create_subscription(CameraInfo, self._topic('depth/camera_info'), self._on_info, qos)
        self._thread = threading.Thread(target=self._spin, name='ros-camera', daemon=True)
        self._thread.start()
        self._wait_until_ready()

    def _wait_until_ready(self):
        """Learn a bounded frame cadence before accepting startup data from a busy ROS pipeline."""
        deadline = time.monotonic() + self.start_timeout
        last_stale = None
        while not self._stop.is_set():
            self.check_mode()
            # A first frame can arrive >300 ms after capture while inference/camera
            # workers start. With no second frame, interval_s is still zero and the
            # normal cadence allowance cannot yet be estimated.
            if self.frames_received >= 2 and self.calibration is not None:
                try:
                    self.latest()
                except StaleCameraFrame as exc:
                    last_stale = exc
                else:
                    logging.getLogger(__name__).info(
                        'ROS camera ready: %d pairs, capture-to-receipt %.0f ms, bounded cadence %.0f ms',
                        self.frames_received, self.latency_s * 1000, self.interval_s * 1000)
                    self._check_publishers()
                    return
            if time.monotonic() > deadline:
                raise TimeoutError(f'No usable camera stream on {self._topic("depth/image_rect")} and '
                                   f'{self._topic("left/image_rect_color")} within {self.start_timeout:g} s '
                                   f'({self.frames_received} matched pairs; need at least 2 and calibration). '
                                   f'Last stale frame: {last_stale}; check depth rate and ROS_DOMAIN_ID')
            self._stop.wait(0.01)
        raise RuntimeError('Camera stopped while waiting for startup frames')

    def _spin(self):
        try:
            while not self._stop.is_set() and self._context.ok():
                self._executor.spin_once(timeout_sec=0.1)
                if time.monotonic() - self._last_publisher_check >= .25:
                    self._check_publishers()
                    self._last_publisher_check = time.monotonic()
        except Exception as exc:
            if not self._stop.is_set():
                self._error = exc

    def _check_publishers(self):
        if self._node is None:
            return
        for name in ('left/image_rect_color', 'depth/image_rect'):
            topic = self._topic(name)
            count = self._node.count_publishers(topic)
            if count > 1:
                raise RuntimeError(f'Multiple publishers ({count}) on {topic}; duplicate camera/depth '
                                   'nodes can mix frames. Stop orphaned/duplicate camera launches.')

    def _on_info(self, msg):
        if self.calibration is None or self.calibration['resolution'] != [msg.width, msg.height]:
            fx, _, cx, _, fy, cy = (float(v) for v in msg.k[:6])
            self.calibration = {
                'resolution': [int(msg.width), int(msg.height)],
                'left': {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy, 'k1': 0.0, 'k2': 0.0, 'p1': 0.0, 'p2': 0.0, 'k3': 0.0},
                'rectified': True,
                'depth': {'unit': 'mm', 'dtype': 'uint16', 'invalid': 0, 'aligned_to': 'left', 'available': True},
                'source': f'ros:{self._topic("depth/camera_info")}',
            }

    def _on_message(self, index, msg):
        if self._error is not None:
            return
        try:
            stamp = stamp_ns(msg.header.stamp)
            if stamp <= self._last_stamp:
                # DDS queues/pair completion can deliver a duplicate or old half
                # after newer data. Never replace the accepted frame or rebase its
                # timestamp. A sustained reset still trips latest()'s age watchdog.
                self.frames_dropped += 1
                now = time.monotonic()
                if now - self._last_drop_warning >= 5.:
                    logging.getLogger(__name__).warning(
                        'Dropping duplicate/out-of-order ROS camera message: lag %.3f s, dropped %d; '
                        'newest accepted frame retained', (self._last_stamp - stamp) / 1e9, self.frames_dropped)
                    self._last_drop_warning = now
                return
            pair = self._pairs.add(index, stamp, msg)
            if pair is None:
                return
            received_ns, received_monotonic = time.time_ns(), time.monotonic()
            frame = frame_from_messages(pair[0], pair[1], received_ns, received_monotonic, self.frames_received)
            previous_stamp = self._last_stamp
            self._last_stamp = frame.header['t_image_ns']
            with self._lock:
                self._latest = frame
                self.latency_s = received_monotonic - frame.captured_monotonic
                if self._last_received is not None:   # capped, so one long gap does not relax the check for long
                    # Queued ROS pairs can arrive back-to-back after slow startup.
                    # Their capture stamps still describe the true camera cadence.
                    capture_interval = (self._last_stamp - previous_stamp) / 1e9
                    self.interval_s = min(max(received_monotonic - self._last_received, capture_interval),
                                          self.max_age)
                self._last_received = received_monotonic
                self.frames_received += 1
        except Exception as exc:
            self._error = exc

    def check_mode(self, force=False):
        """Health check: the ROS receiver must be alive and error free."""
        if self._error is not None:
            raise RuntimeError(f'Camera disconnected or invalid stream: {self._error}') from self._error
        if self._thread is not None and not self._thread.is_alive():
            raise RuntimeError('ROS camera receiver stopped')

    def latest(self):
        self.check_mode()
        with self._lock:
            frame, interval = self._latest, self.interval_s
        if frame is None:
            raise RuntimeError('No camera frame')
        age = time.monotonic() - frame.captured_monotonic
        # As in NanoStreamCamera: depth is computed after capture and a frame stays current until the
        # next one arrives, so allow two depth frame intervals on top of the transport budget.
        allowance = 2 * interval + 0.05
        if age < -0.01:
            raise RuntimeError(f'Camera frame clock mismatch: age={age:.3f}s')
        if age > self.max_age + allowance:
            raise StaleCameraFrame(f'Camera frame stale: age={age:.3f}s, limit={self.max_age + allowance:.3f}s '
                                   f'(bounded depth interval {1000 * interval:.0f} ms, '
                                   f'{self.frames_received} matched pairs)')
        return frame

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=1)
        if self._node is not None:
            self._node.destroy_node()
        if self._context is not None:
            self._context.try_shutdown()
