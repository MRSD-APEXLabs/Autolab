"""ROS 2 node publishing a ZED X Nano or ZED X feed served by the Xavier (no ZED SDK needed).

Topics, relative to the node namespace (`zedx_nano` in the launch file), for each published eye:

  <eye>/image_raw/compressed  sensor_msgs/CompressedImage  the server's JPEG, forwarded without re-encoding
  <eye>/image_raw             sensor_msgs/Image, bgr8       decoded only while something subscribes
  <eye>/camera_info           sensor_msgs/CameraInfo        factory K, D plus the stereo rectification R, P

Images are unrectified. Header stamps are the capture time mapped to this machine's clock
(re-synchronized with the Xavier every `clock_sync_period` seconds); in stereo mode both eyes
of a pair carry the left capture stamp, so consumers can pair them exactly. The node
reconnects on its own when the Xavier server restarts or the network drops.

`camera` picks the camera on the Xavier camera hub (zedx, zedx_nano); '' uses the root API,
which serves the Nano (the hub's legacy routes, or the old Nano-only server). The hub streams
one camera at a time: while another one is selected the node polls quietly and resumes when
its camera is selected again.
"""
import array
import re
import threading
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Header

from .calibration import camera_info_fields, stereo_rectification
from .stream import (DEFAULT_CAMERA, CameraInactive, NanoStreamClient, capture_size, captured_monotonic, decode_jpeg,
                     frame_header, iter_mjpeg_parts, output_size, shutdown_response, split_stereo_part)

SENSORS = ('stereo', 'left', 'right')
QOS = QoSProfile(depth=2)


def camera_info_msg(fields):
    msg = CameraInfo()
    msg.width, msg.height = fields['width'], fields['height']
    msg.distortion_model = fields['distortion_model']
    msg.d, msg.k, msg.r, msg.p = fields['d'], fields['k'], fields['r'], fields['p']
    return msg


class NanoCameraNode(Node):
    def __init__(self, **kwargs):
        # kwargs go to rclpy Node (namespace, parameter_overrides, context, ...) for embedding and tests
        super().__init__('zedx_nano_camera', **kwargs)
        self.host = self.declare_parameter('host', '192.168.1.101').value
        self.port = int(self.declare_parameter('port', 8090).value)
        self.camera = self.declare_parameter('camera', '').value
        self.sensor = self.declare_parameter('sensor', 'stereo').value
        timeout = float(self.declare_parameter('timeout', 3.0).value)
        if self.camera and not re.fullmatch(r'\w+', self.camera):
            raise ValueError(f'camera must be a hub camera name such as zedx or zedx_nano, got {self.camera!r}')
        # '' derives the frame from the camera name: <camera>_<eye>_camera_optical_frame
        self.frame_ids = {eye: self.declare_parameter(f'{eye}_frame_id', '').value
                          or f'{self.camera or DEFAULT_CAMERA}_{eye}_camera_optical_frame' for eye in ('left', 'right')}
        clock_sync_period = float(self.declare_parameter('clock_sync_period', 10.0).value)
        self.max_sync_uncertainty = float(self.declare_parameter('max_sync_uncertainty', 0.05).value)
        self.reconnect_delay = float(self.declare_parameter('reconnect_delay', 1.0).value)
        self.inactive_poll_period = float(self.declare_parameter('inactive_poll_period', 1.0).value)
        if self.sensor not in SENSORS:
            raise ValueError(f'sensor must be one of {SENSORS}, got {self.sensor!r}')

        self.client = NanoStreamClient(self.host, self.port, timeout, self.camera)
        self.bridge = CvBridge()
        self.pubs = {eye: {'compressed': self.create_publisher(CompressedImage, f'{eye}/image_raw/compressed', QOS),
                           'raw': self.create_publisher(Image, f'{eye}/image_raw', QOS),
                           'info': self.create_publisher(CameraInfo, f'{eye}/camera_info', QOS)}
                     for eye in (('left', 'right') if self.sensor == 'stereo' else (self.sensor,))}
        self._sync = None      # (wall_offset_ns, uncertainty_ns, mono_offset_ns, local wall-minus-monotonic ns)
        self._info_msgs = {}
        self._response = None
        self._inactive = False   # the hub streams another camera; logged once per inactive spell
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='nano-stream', daemon=True)
        self._thread.start()
        self.create_timer(clock_sync_period, self._resync)

    # -- clock --------------------------------------------------------------------------------
    def _synchronize(self, initial=False):
        offset, uncertainty, mono_offset = self.client.synchronize_clock()
        if uncertainty / 1e9 > self.max_sync_uncertainty:
            message = f'Clock sync with the Xavier is too uncertain ({uncertainty / 1e6:.1f} ms)'
            if initial:
                raise RuntimeError(message)
            self.get_logger().warning(message + '; keeping the previous estimate', throttle_duration_sec=60.0)
            return
        self._sync = (offset, uncertainty, mono_offset, time.time_ns() - time.monotonic_ns())

    def _resync(self):
        if self._sync is None or self._response is None:
            return   # the stream thread synchronizes when it (re)connects
        try:
            self._synchronize()
        except Exception as exc:
            self.get_logger().warning(f'Clock resync failed: {exc}', throttle_duration_sec=60.0)

    def _ros_stamp(self, captured):
        """Capture time on the node clock: now minus the frame's age on the local monotonic clock."""
        age_ns = time.monotonic_ns() - int(captured * 1e9)
        return Time(nanoseconds=self.get_clock().now().nanoseconds - age_ns).to_msg()

    # -- streaming ----------------------------------------------------------------------------
    def _run(self):
        while not self._stop.is_set():
            delay = self.reconnect_delay
            try:
                self._stream()
            except CameraInactive as exc:
                delay = self.inactive_poll_period
                if not self._inactive and not self._stop.is_set():
                    self._inactive = True
                    self.get_logger().info(f'{self.client.name} inactive (hub active: {exc.active or "none"}); waiting')
            except Exception as exc:
                if self._stop.is_set() or not self.context.ok():
                    break   # shutting down (Ctrl-C shuts the context down before stop() runs)
                self.get_logger().error(f'Camera stream {self.client.url("")}: {exc}; reconnecting',
                                        throttle_duration_sec=30.0)
            finally:
                response, self._response = self._response, None
                if response is not None:
                    shutdown_response(response)
            self._stop.wait(delay)

    def _stream(self):
        info = self.client.fetch_info(self.sensor)   # raises CameraInactive while the hub streams another camera
        if self._inactive:
            self._inactive = False
            self.get_logger().info(f'{self.client.name} is active again')
        self._synchronize(initial=True)
        conf, calibration = self.client.load_calibration(info)
        rectification = stereo_rectification(conf, capture_size(info), output_size(info))
        self._info_msgs = {eye: camera_info_msg(camera_info_fields(rectification, eye)) for eye in self.pubs}
        width, height = output_size(info)
        self._response = self.client.open_feed(self.sensor)
        self.get_logger().info(f'Streaming {self.sensor} {width}x{height} from {self.client.url("/")} '
                               f'(serial {calibration["serial"]}, baseline {rectification["baseline_mm"]:.2f} mm, '
                               f'calibration from {calibration["source"]})')
        last_id = -1
        for headers, body in iter_mjpeg_parts(self._response):
            wall, mono = time.time_ns(), time.monotonic()
            offset_ns, _, mono_offset_ns, wall_anchor = self._sync
            uses_mono = mono_offset_ns is not None and 'x-capture-mono-ns' in headers
            if not uses_mono and abs(wall - int(mono * 1e9) - wall_anchor) > 50_000_000:
                raise RuntimeError('Local clock changed; reconnecting to resynchronize')
            header = frame_header(headers, width, height)
            if header['frame_id'] <= last_id:
                raise RuntimeError('Camera frame counter reset or moved backwards')
            last_id = header['frame_id']
            stamp = self._ros_stamp(captured_monotonic(header, wall, mono, offset_ns, mono_offset_ns))
            if self.sensor == 'stereo':
                jpegs = zip(('left', 'right'), split_stereo_part(headers, body))
            else:
                jpegs = [(self.sensor, body)]
            for eye, jpeg in jpegs:
                self._publish(eye, jpeg, stamp)
        if not self._stop.is_set():
            self.client.fetch_info(self.sensor)   # the hub ends the feeds of a camera it switches away from
            raise RuntimeError('Camera stream ended')

    def _publish(self, eye, jpeg, stamp):
        header = Header(stamp=stamp, frame_id=self.frame_ids[eye])
        pubs = self.pubs[eye]
        compressed = CompressedImage(header=header, format='bgr8; jpeg compressed bgr8')
        compressed.data = array.array('B', jpeg)
        pubs['compressed'].publish(compressed)
        if pubs['raw'].get_subscription_count():
            image = self.bridge.cv2_to_imgmsg(decode_jpeg(jpeg), encoding='bgr8')
            image.header = header
            pubs['raw'].publish(image)
        info = self._info_msgs[eye]
        info.header = header
        pubs['info'].publish(info)

    def stop(self):
        self._stop.set()
        response, self._response = self._response, None
        if response is not None:
            shutdown_response(response)
        self._thread.join(timeout=max(2.0, self.client.timeout))


def main(args=None):
    rclpy.init(args=args)
    node = NanoCameraNode()
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
