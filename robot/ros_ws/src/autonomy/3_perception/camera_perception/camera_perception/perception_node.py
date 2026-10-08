"""ROS 2 node: AprilTags, YOLO detections and a point cloud from one camera's rectified image and depth.

Subscribes, relative to the node namespace (`zedx` or `zedx_nano` in the launch file), to what the
zedx_nano_depth node publishes:

  left/image_rect_color   bgr8, paired with the depth by exact stamp
  depth/image_rect        16UC1 millimetres, 0 = invalid
  depth/camera_info       the rectified pinhole model of both

Publishes (the launch file remaps them to the pc3.py names, /inspect/... and /servo/...):

  perception/apriltags        MarkerArray: SPHERE per tag, id = tag id (tags with a pose only)
  perception/detections       MarkerArray: CUBE + TEXT per YOLO detection, most confident last
  perception/json             std_msgs/String: the frame's tags and detections with poses and diagnostics
  perception/image            the input image (only while subscribed)
  perception/annotated_image  the image with tags, detections and their axes drawn (only while subscribed)
  perception/points           PointCloud2 x, y, z (+ rgb) from the depth (pointcloud.enable)
  perception/capture_snapshot std_srvs/Trigger: recapture the snapshot cloud (pointcloud.mode snapshot)

Everything is in the camera's optical frame (x right, y down, z forward), stamped with the image
stamp and labelled `output_frame` if set (e.g. top_camera for the ZED X). Frames are processed on a
worker thread, newest first; frames that arrive meanwhile are dropped, not queued.
"""
from collections import defaultdict
from functools import partial
import os
import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Empty, Trigger
from visualization_msgs.msg import MarkerArray

from zedx_nano_depth.sync import PairBuffer

from .geometry import Intrinsics
from .markers import annotate, apriltag_markers, detection_markers, summary_json
from .pointcloud import Snapshot, depth_to_points, to_pointcloud2, voxel_downsample

QOS = QoSProfile(depth=2)
CLOUD_MODES = ('live', 'snapshot')
SIZE_MISMATCH = 0.15   # warn when a tag's size from the depth differs from its configured size by more


class RateLimit:
    """At most one event per `period` s on average, from a stream of opportunities (frames).

    Keeps a due time rather than the time since the last event, and lets an opportunity up to 10% of a
    period early through, so 16 fps input capped at 10 Hz gives ~10 Hz, not every other frame (8 Hz).
    After a pause (or at the start) the schedule restarts from the event, so there is no catch-up burst.
    """

    def __init__(self, period):
        self.period = float(period)
        self._due = 0.0

    def ready(self, now):
        if now < self._due - 0.1 * self.period:
            return False
        self._due = (self._due if now - self._due < self.period else now) + self.period
        return True


class PerceptionNode(Node):
    def __init__(self, **kwargs):
        # kwargs go to rclpy Node (namespace, parameter_overrides, context, ...) for embedding and tests
        super().__init__('camera_perception', **kwargs)
        # dynamic typing: a launch argument "1" arrives as an int where a float is declared; values are converted
        param = lambda name, default: self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value
        self.camera = param('camera', 'zedx')
        self.output_frame = param('output_frame', '')
        self.publish_image = bool(param('publish_image', True))
        self.annotate = bool(param('annotate', True))
        max_rate = float(param('max_rate', 0.0))
        self.min_period = 1.0 / max_rate if max_rate > 0.0 else 0.0

        self.tags = self.yolo = None
        self.detections_key = param('yolo.json_key', 'detections')
        self.marker_ns = param('yolo.marker_ns', 'detections')
        if param('apriltags.enable', True):
            self.tags = self._make_tag_detector(param)
        if param('yolo.enable', True):
            self.yolo = self._make_yolo(param)

        self.cloud_enabled = bool(param('pointcloud.enable', False))
        self.cloud_mode = param('pointcloud.mode', 'live')
        if self.cloud_mode not in CLOUD_MODES:
            raise ValueError(f'pointcloud.mode must be one of {CLOUD_MODES}, got {self.cloud_mode!r}')
        self.cloud_stride = int(param('pointcloud.stride', 4))
        self.cloud_bounds = tuple(tuple(float(v) for v in param(f'pointcloud.bounds_{axis}', default))
                                  for axis, default in (('x', [-0.6, 0.6]), ('y', [-0.6, 0.6]), ('z', [0.2, 1.7])))
        for axis, bound in zip('xyz', self.cloud_bounds):
            if len(bound) != 2 or bound[0] >= bound[1]:
                raise ValueError(f'pointcloud.bounds_{axis} must be [low, high], got {list(bound)}')
        self.cloud_edge_threshold = float(param('pointcloud.edge_threshold', 0.02))
        self.cloud_color = bool(param('pointcloud.color', True))
        cloud_rate = float(param('pointcloud.max_rate', 5.0))
        self.cloud_period = 1.0 / cloud_rate if cloud_rate > 0.0 else 0.0
        self.cloud_voxel = float(param('pointcloud.voxel_size', 0.0))
        snapshot_frames = int(param('pointcloud.snapshot_frames', 10))
        cache_path = param('pointcloud.cache_path', '')
        fresh = bool(param('pointcloud.fresh', False))
        clear_service = param('pointcloud.clear_octomap_service', '/clear_octomap')
        self.clear_max_attempts = int(param('pointcloud.clear_octomap_attempts', 40))

        self.bridge = CvBridge()
        self.pub_tags = self.create_publisher(MarkerArray, 'perception/apriltags', QOS)
        self.pub_detections = self.create_publisher(MarkerArray, 'perception/detections', QOS)
        self.pub_json = self.create_publisher(String, 'perception/json', QOS)
        self.pub_image = self.create_publisher(Image, 'perception/image', QOS)
        self.pub_annotated = self.create_publisher(Image, 'perception/annotated_image', QOS)
        self.pub_points = self.create_publisher(PointCloud2, 'perception/points', QOS) if self.cloud_enabled else None

        # point cloud state; the snapshot is shared between the worker (capture) and the timer (publishing)
        self._cloud_lock = threading.Lock()
        self._cloud_rate = RateLimit(self.cloud_period)
        self.snapshot = self._snapshot_msg = None
        self._new_msg = None        # snapshot: a new capture, published once the octomap is cleared
        self._live_ready = False    # live: a new cloud is waiting for the octomap clear
        self._needs_clear = False   # the next new cloud waits for /clear_octomap
        self._clear_client = self._clear_future = None
        self._clear_sent, self._clear_attempts = 0.0, 0
        if self.cloud_enabled:
            if self.cloud_mode == 'snapshot':
                self.snapshot = Snapshot(snapshot_frames, self.cloud_voxel, cache_path)
                if fresh:
                    self.get_logger().info('pointcloud.fresh: ignoring the snapshot cache, capturing a new cloud')
                elif self.snapshot.load():
                    self.get_logger().info(f'Loaded the snapshot cloud from {self.snapshot.cache_path} '
                                           f'({len(self.snapshot.points)} points)')
                    self._snapshot_msg = to_pointcloud2(Header(), self.snapshot.points, self.snapshot.rgb)
                self.create_service(Trigger, 'perception/capture_snapshot', self._on_capture)
            if clear_service:
                # MoveIt's octomap accumulates: a new capture is only "fresh" if the old map is wiped first
                self._clear_client = self.create_client(Empty, clear_service)
                self._needs_clear = fresh
            if self.cloud_mode == 'snapshot' and self.cloud_period <= 0.0:
                raise ValueError('pointcloud.max_rate must be > 0 in snapshot mode (the snapshot is republished at it)')
            # republishes the snapshot; in live mode it only drives the octomap clearing
            self.create_timer(self.cloud_period or 0.2, self._on_cloud_timer)

        self._info = None
        self._intrinsics_key = self._intrinsics = None
        self._frame_id = f'{self.camera}_left_camera_optical_frame'   # until the first image names it
        self._pairs = PairBuffer()
        self._pending = None
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._frame_rate = RateLimit(self.min_period)
        self._stats = self._new_stats()
        self._idle = False
        self.create_subscription(CameraInfo, 'depth/camera_info', self._on_info, QOS)
        self._inputs = [self.create_subscription(Image, topic, partial(self._on_input, index), QOS).topic_name
                        for index, topic in enumerate(('left/image_rect_color', 'depth/image_rect'))]
        self._thread = threading.Thread(target=self._work, name='perception', daemon=True)
        self._thread.start()
        self.create_timer(30.0, self._report)
        self.get_logger().info(self._describe())

    # -- setup --------------------------------------------------------------------------------
    def _make_tag_detector(self, param):
        from .apriltags import TagDetector, parse_sizes
        ids = param('apriltags.ids', [0, 1, 2, 21, 22])
        ids = [int(i) for i in (ids if isinstance(ids, (list, tuple)) else [ids])]   # -1 as well as [-1]
        try:
            return TagDetector(family=param('apriltags.family', 'tag36h11'),
                               ids=() if any(i < 0 for i in ids) else ids,
                               size=float(param('apriltags.size', 0.0625)),
                               sizes=parse_sizes(param('apriltags.sizes', '')),
                               nthreads=int(param('apriltags.nthreads', 2)),
                               quad_decimate=float(param('apriltags.quad_decimate', 1.0)),
                               max_hamming=int(param('apriltags.max_hamming', 1)),
                               position=param('apriltags.position', 'pnp'))
        except ImportError as exc:
            self.get_logger().error(f'AprilTags disabled: {exc} (pip install pupil-apriltags)')
            return None

    def _make_yolo(self, param):
        from .yolo import YoloDetector
        model = os.path.expanduser(param('yolo.model', ''))
        options = dict(device=param('yolo.device', 'cuda:0'), conf=float(param('yolo.conf', 0.25)),
                       iou=float(param('yolo.iou', 0.7)), imgsz=int(param('yolo.imgsz', 0)),
                       classes=param('yolo.classes', ''), max_detections=int(param('yolo.max_detections', 0)),
                       half=bool(param('yolo.half', False)), clahe_clip=float(param('yolo.clahe_clip', 0.0)),
                       clahe_tiles=int(param('yolo.clahe_tiles', 4)))
        if not model:
            self.get_logger().error('YOLO disabled: set yolo.model to a .pt file')
            return None
        try:
            detector = YoloDetector(model, **options)
            t0 = time.monotonic()
            detector.warmup()
            self.get_logger().info(f'YOLO {os.path.basename(model)} ready in {time.monotonic() - t0:.1f} s: '
                                   f'{detector.describe()}')
            return detector
        except Exception as exc:
            # a missing model or ultralytics only costs the detections; tags and the cloud keep running
            self.get_logger().error(f'YOLO disabled: {exc}')
            return None

    def _describe(self):
        parts = [f'camera {self.camera}, output frame {self.output_frame or "(input frame)"}']
        if self.tags is not None:
            sizes = ', '.join(f'{k}: {v} m' for k, v in sorted(self.tags.sizes.items()))
            parts.append(f'AprilTags ids {sorted(self.tags.ids) or "all"}, {self.tags.size} m'
                         + (f' ({sizes})' if sizes else '') + f', position from {self.tags.position}')
        if self.yolo is not None:
            parts.append(f'YOLO {self.yolo.task}, conf >= {self.yolo.conf}'
                         + (f', best {self.yolo.max_detections}' if self.yolo.max_detections else ''))
        if self.cloud_enabled:
            parts.append(f'{self.cloud_mode} point cloud every {self.cloud_stride} px, '
                         + (f'<= {1.0 / self.cloud_period:.0f} Hz' if self.cloud_period else 'every frame'))
        return 'Perception: ' + '; '.join(parts)

    # -- inputs -------------------------------------------------------------------------------
    def _on_info(self, msg):
        self._info = msg

    def _on_input(self, index, msg):
        self._stats['inputs'] += 1
        pair = self._pairs.add(index, (msg.header.stamp.sec, msg.header.stamp.nanosec), msg)
        if pair is not None:
            self._stats['pairs'] += 1
            with self._cond:        # single slot: the worker always takes the newest frame
                self._pending = pair
                self._cond.notify()

    def _intrinsics_for(self, info, shape):
        key = (info.width, info.height, tuple(info.k))
        if key != self._intrinsics_key:
            if (info.height, info.width) != tuple(shape[:2]):
                raise ValueError(f'image is {shape[1]}x{shape[0]} but camera_info is {info.width}x{info.height}')
            self._intrinsics, self._intrinsics_key = Intrinsics.from_camera_info(info), key
        return self._intrinsics

    # -- processing ---------------------------------------------------------------------------
    def _work(self):
        while not self._stop.is_set():
            with self._cond:
                while self._pending is None and not self._stop.is_set():
                    self._cond.wait(0.5)
                pending, self._pending = self._pending, None
            if pending is None:
                continue
            if not self._frame_rate.ready(time.monotonic()):
                continue
            try:
                self._process(*pending)
            except Exception as exc:
                if self._stop.is_set() or not self.context.ok():
                    return   # Ctrl-C shuts the context down before stop() runs
                self.get_logger().error(f'Skipping frame: {exc!r}', throttle_duration_sec=10.0)

    def _process(self, image_msg, depth_msg):
        info = self._info
        if info is None:
            self.get_logger().warning('Waiting for depth/camera_info', throttle_duration_sec=10.0)
            return
        t0 = time.monotonic()
        bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        if depth.dtype != np.uint16 or depth.shape[:2] != bgr.shape[:2]:
            raise ValueError(f'depth must be 16UC1 like the image {bgr.shape[1]}x{bgr.shape[0]}, got '
                             f'{depth_msg.encoding} {depth.shape[1]}x{depth.shape[0]}')
        intrinsics = self._intrinsics_for(info, bgr.shape)
        self._frame_id = image_msg.header.frame_id or self._frame_id
        header = Header(stamp=image_msg.header.stamp, frame_id=self.output_frame or self._frame_id)
        stats = self._stats

        t1 = time.monotonic()
        tags = self.tags.detect(bgr, depth, intrinsics) if self.tags is not None else []
        t2 = time.monotonic()
        detections = self.yolo.detect(bgr, depth, intrinsics) if self.yolo is not None else []
        t3 = time.monotonic()
        if self.tags is not None:
            self.pub_tags.publish(apriltag_markers(header, tags))
        if self.yolo is not None:
            self.pub_detections.publish(detection_markers(header, detections, self.marker_ns))
        self.pub_json.publish(String(data=summary_json(header, tags, detections, self.detections_key,
                                                       {'camera': self.camera})))
        if self.publish_image and self.pub_image.get_subscription_count():
            image_msg.header.frame_id = header.frame_id
            self.pub_image.publish(image_msg)
        if self.annotate and self.pub_annotated.get_subscription_count():
            annotated = self.bridge.cv2_to_imgmsg(annotate(bgr, tags, detections, intrinsics), encoding='bgr8')
            annotated.header = header
            self.pub_annotated.publish(annotated)
        t4 = time.monotonic()
        points = self._cloud(header, depth, intrinsics, bgr) if self.cloud_enabled else None
        t5 = time.monotonic()

        stats['frames'] += 1
        stats['decode_s'] += t1 - t0
        stats['tags_s'] += t2 - t1
        stats['yolo_s'] += t3 - t2
        stats['publish_s'] += t4 - t3
        if points is not None:
            stats['clouds'] += 1
            stats['cloud_s'] += t5 - t4
            stats['cloud_points'] = points
        for tag in tags:
            stats['tags'][tag.id] += 1
            if tag.measured_size_m is not None:
                stats['tag_sizes'][tag.id].append(tag.measured_size_m)
        for det in detections:
            stats['detections'][det.name] += 1

    def _cloud(self, header, depth, intrinsics, bgr):
        """Build (and in live mode publish) this frame's cloud if one is due; its point count, else None."""
        with self._cloud_lock:
            if self.snapshot is not None and self.snapshot.ready:
                return None
        if not self._cloud_rate.ready(time.monotonic()):
            return None
        points, rgb = depth_to_points(depth, intrinsics, self.cloud_stride, self.cloud_bounds,
                                      self.cloud_edge_threshold, bgr if self.cloud_color else None)
        if self.snapshot is None:
            points, rgb = voxel_downsample(points, rgb, self.cloud_voxel)
            if self._needs_clear:
                self._live_ready = True   # the timer clears the octomap now that there is a cloud to replace it
            else:
                self.pub_points.publish(to_pointcloud2(header, points, rgb))
            return len(points)
        with self._cloud_lock:
            if self.snapshot.add(points, rgb):
                # the previous snapshot stays published until the timer swaps this one in
                self._new_msg = to_pointcloud2(Header(), self.snapshot.points, self.snapshot.rgb)
                saved = self.snapshot.cache_path and self.snapshot.save_error is None
                self.get_logger().info(f'Snapshot cloud captured: {len(self.snapshot.points)} points from '
                                       f'{self.snapshot.frames} frames'
                                       + (f', saved to {self.snapshot.cache_path}' if saved else ''))
                if self.snapshot.save_error is not None:
                    self.get_logger().warning(f'Could not cache the snapshot: {self.snapshot.save_error}')
            elif len(points):
                self.get_logger().info(f'Capturing the snapshot cloud: {self.snapshot.progress}/{self.snapshot.frames}')
        return len(points)

    # -- snapshot cloud and octomap -------------------------------------------------------------
    def _on_cloud_timer(self):
        with self._cloud_lock:
            new_msg = self._new_msg
        # The octomap is cleared only once the new cloud exists; until then the old one stays in MoveIt
        # (and a snapshot keeps being republished). While clearing, nothing is published.
        if (new_msg is not None or self._live_ready) and self._needs_clear and not self._try_clear():
            return
        if self.snapshot is None:
            return   # live clouds are published by the worker
        with self._cloud_lock:
            if new_msg is not None and self._new_msg is new_msg:
                self._snapshot_msg, self._new_msg = new_msg, None
            msg = self._snapshot_msg
        if msg is None:
            return
        # the snapshot is static: republish it (stamped now, like pc3.py) so the octomap keeps it
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.output_frame or self._frame_id)
        self.pub_points.publish(msg)

    def _try_clear(self):
        """One step of wiping move_group's octomap before a new cloud; True once done (or given up)."""
        if self._clear_future is not None:
            if self._clear_future.done():
                self._clear_future = None
                self._needs_clear = self._live_ready = False
                self.get_logger().info('Octomap cleared, publishing the new cloud')
                return True
            if time.monotonic() - self._clear_sent < 5.0:
                return False
            self._clear_future.cancel()
            self._clear_future = None
        self._clear_attempts += 1
        service = self._clear_client.srv_name
        if self._clear_attempts > self.clear_max_attempts:
            self.get_logger().warning(f'{service} did not answer after {self.clear_max_attempts} tries; publishing '
                                      'anyway, so the new cloud is merged into the existing octomap')
            self._needs_clear = self._live_ready = False
            return True
        if not self._clear_client.service_is_ready():
            if self._clear_attempts % 5 == 1:
                self.get_logger().info(f'Waiting for {service} (is the planning stack up?) '
                                       f'[{self._clear_attempts}/{self.clear_max_attempts}]')
            return False
        self._clear_future = self._clear_client.call_async(Empty.Request())
        self._clear_sent = time.monotonic()
        return False

    def _on_capture(self, request, response):
        with self._cloud_lock:
            self.snapshot.reset()   # the current snapshot stays published until the new one is captured
            self._new_msg = None
        if self._clear_client is not None:
            self._needs_clear, self._clear_attempts, self._clear_future = True, 0, None
        response.success = True
        response.message = (f'capturing {self.snapshot.frames} frames'
                            + (f', then clearing {self._clear_client.srv_name}' if self._clear_client else '')
                            + ('; the current cloud stays published until then' if self._snapshot_msg else ''))
        self.get_logger().info(f'Snapshot recapture requested: {response.message}')
        return response

    # -- reporting ----------------------------------------------------------------------------
    @staticmethod
    def _new_stats():
        stats = defaultdict(float)
        stats.update(inputs=0, pairs=0, frames=0, clouds=0, cloud_points=0,
                     tags=defaultdict(int), tag_sizes=defaultdict(list), detections=defaultdict(int))
        return stats

    def _report(self):
        stats, self._stats = self._stats, self._new_stats()
        frames = stats['frames']
        if stats['inputs']:
            self._idle = False
        if frames:
            ms = lambda key, n=frames: f'{1000.0 * stats[key] / max(n, 1):.0f}'
            line = (f'{frames / 30.0:.1f} frames/s (decode {ms("decode_s")}, tags {ms("tags_s")}, yolo {ms("yolo_s")}, '
                    f'publish {ms("publish_s")} ms)')
            if self.tags is not None:
                seen = ', '.join(f'{i}: {n}' for i, n in sorted(stats['tags'].items())) or 'none'
                line += f'; tags seen {{{seen}}}'
            if self.yolo is not None:
                seen = ', '.join(f'{k}: {n}' for k, n in sorted(stats['detections'].items())) or 'none'
                line += f'; detections {{{seen}}}'
            if stats['clouds']:
                line += (f'; {stats["clouds"]} clouds, {ms("cloud_s", stats["clouds"])} ms each, '
                         f'last {int(stats["cloud_points"])} points')
            if self.snapshot is not None and self.snapshot.ready:
                line += f'; publishing the {len(self.snapshot.points)}-point snapshot cloud'
            self.get_logger().info(line)
            self._check_tag_sizes(stats['tag_sizes'])
        elif stats['pairs']:
            self.get_logger().warning('No frame processed in the last 30 s although image and depth arrive')
        elif stats['inputs']:
            self.get_logger().warning(f'{int(stats["inputs"])} messages but no image + depth pair in the last 30 s on '
                                      f'{self._inputs[0]} and {self._inputs[1]}; both must carry equal stamps')
        elif not self._idle:
            # normal while the camera hub streams the other camera: say it once, naming the topic
            self._idle = True
            self.get_logger().info(f'No input on {self._inputs[0]}; idle (camera inactive?)')

    def _check_tag_sizes(self, measured):
        for tag_id, sizes in sorted(measured.items()):
            if len(sizes) < 3:
                continue
            size, configured = float(np.median(sizes)), self.tags.size_of(tag_id)
            if abs(size / configured - 1.0) > SIZE_MISMATCH:
                self.get_logger().warning(
                    f'Tag {tag_id} measures {1000 * size:.0f} mm in the depth but is configured as '
                    f'{1000 * configured:.1f} mm, which scales its PnP distance by the same factor; if the '
                    f'print really is that size set apriltags.sizes "{tag_id}={size:.4f}"')

    def stop(self):
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        self._thread.join(timeout=5)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PerceptionNode()
    except Exception as exc:
        rclpy.try_shutdown()
        raise SystemExit(f'camera_perception: {exc}') from exc
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.5)
            except Exception:
                if rclpy.ok():
                    raise
                break   # Ctrl-C shut the context down during spin_once, not a failure
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
