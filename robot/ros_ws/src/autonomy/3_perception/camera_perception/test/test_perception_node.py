"""The ROS node on a rendered scene published as the depth node would (needs a sourced ROS 2 environment)."""
import json
import os
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('pupil_apriltags')
from cv_bridge import CvBridge  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, PointCloud2  # noqa: E402
from std_msgs.msg import Header, String  # noqa: E402
from std_srvs.srv import Empty, Trigger  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

import camera_perception.yolo as yolo_module  # noqa: E402
from camera_perception.geometry import planar_pose  # noqa: E402
from camera_perception.perception_node import PerceptionNode, RateLimit  # noqa: E402
from camera_perception.pointcloud import pointcloud2_to_numpy  # noqa: E402
from camera_perception.yolo import Detection  # noqa: E402
from scene import INTRINSICS, Scene, rx  # noqa: E402

FRAME = 'zedx_left_camera_optical_frame'


def wait_for(executor, condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    return condition()


def make_scene():
    scene = Scene(rotation=rx(0.4), origin=(0.0, 0.05, 0.7))
    truth = {tag_id: scene.add_tag(tag_id, 0.0625, xy=(x, 0.0))[1] for tag_id, x in ((1, -0.12), (21, 0.12))}
    return scene, truth


class FakeYolo:
    """Stands in for YoloDetector: one 'wellplate' in the middle of the image, located like the real one."""

    def __init__(self, model_path, **options):
        self.options, self.task, self.names = options, 'obb', {0: 'wellplate'}
        self.conf, self.max_detections = options['conf'], options['max_detections']

    def describe(self):
        return {'task': self.task}

    def warmup(self, shape=None):
        pass

    def detect(self, bgr, depth_mm=None, intrinsics=None):
        polygon = np.array([[440, 200], [520, 200], [520, 250], [440, 250]], np.float64)
        det = Detection(0, 'wellplate', 0.9, polygon.mean(axis=0), np.array([80.0, 50.0]), 0.0, polygon)
        det.pose, det.metric_size, det.pose_source = planar_pose(depth_mm, intrinsics, det.center, det.size, 0.0)
        return [det]


class Harness:
    """A PerceptionNode in its own namespace plus an io node feeding it and recording its outputs."""

    def __init__(self, name, overrides, services=()):
        self.namespace = f'/test_perception_{os.getpid()}_{name}'
        self.node = PerceptionNode(namespace=self.namespace,
                                   parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()])
        self.io = rclpy.create_node('io', namespace=self.namespace)
        self.bridge = CvBridge()
        self.received = {}
        for topic, kind in (('perception/apriltags', MarkerArray), ('perception/detections', MarkerArray),
                            ('perception/json', String), ('perception/points', PointCloud2),
                            ('perception/annotated_image', Image), ('perception/image', Image)):
            self.io.create_subscription(kind, topic, self._keep(topic), 10)
        self.pubs = {topic: self.io.create_publisher(kind, topic, 10) for topic, kind in (
            ('left/image_rect_color', Image), ('depth/image_rect', Image), ('depth/camera_info', CameraInfo))}
        self.calls = []
        for service in services:
            self.io.create_service(Empty, service, lambda req, resp: (self.calls.append(time.monotonic()), resp)[1])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.io)
        assert self.wait(lambda: all(p.get_subscription_count() for p in self.pubs.values()))

    def _keep(self, topic):
        return lambda msg: self.received.setdefault(topic, []).append((time.monotonic(), msg))

    def wait(self, condition, timeout=10.0):
        return wait_for(self.executor, condition, timeout)

    def publish(self, scene, stamp_offset=0):
        stamp = self.node.get_clock().now().to_msg()
        stamp.sec += stamp_offset
        header = Header(stamp=stamp, frame_id=FRAME)
        info = CameraInfo(header=header, width=INTRINSICS.width, height=INTRINSICS.height, distortion_model='plumb_bob')
        info.k = [INTRINSICS.fx, 0.0, INTRINSICS.cx, 0.0, INTRINSICS.fy, INTRINSICS.cy, 0.0, 0.0, 1.0]
        self.pubs['depth/camera_info'].publish(info)
        image = self.bridge.cv2_to_imgmsg(scene.image, encoding='bgr8')
        depth = self.bridge.cv2_to_imgmsg(scene.depth_mm(), encoding='16UC1')
        image.header = depth.header = header
        self.pubs['depth/image_rect'].publish(depth)       # either order pairs up
        self.pubs['left/image_rect_color'].publish(image)
        return header

    def last(self, topic):
        return self.received[topic][-1][1]

    def close(self):
        self.node.stop()
        self.executor.shutdown()
        self.node.destroy_node()
        self.io.destroy_node()


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_tags_detections_and_a_live_cloud(ros, monkeypatch):
    monkeypatch.setattr(yolo_module, 'YoloDetector', FakeYolo)
    harness = Harness('live', {'output_frame': 'top_camera', 'yolo.model': __file__, 'yolo.marker_ns': 'wellplates',
                               'yolo.json_key': 'wellplates', 'yolo.conf': 1, 'pointcloud.enable': True,
                               'pointcloud.clear_octomap_service': ''})
    try:
        assert harness.node.yolo.conf == 1.0         # an int for a float parameter is accepted
        scene, truth = make_scene()
        # subscribe to the on-demand images before the frame goes out
        assert harness.wait(lambda: harness.node.pub_annotated.get_subscription_count() and
                            harness.node.pub_image.get_subscription_count())
        harness.info_ready = harness.publish(scene)
        topics = ('perception/apriltags', 'perception/detections', 'perception/json', 'perception/points',
                  'perception/annotated_image', 'perception/image')
        assert harness.wait(lambda: all(harness.received.get(t) for t in topics)), sorted(harness.received)

        tags = harness.last('perception/apriltags')
        assert tags.markers[0].action == Marker.DELETEALL
        spheres = {m.id: m for m in tags.markers[1:]}
        assert sorted(spheres) == [1, 21]
        for tag_id, centre in truth.items():
            p = spheres[tag_id].pose.position
            assert np.linalg.norm([p.x - centre[0], p.y - centre[1], p.z - centre[2]]) < 0.01
            assert spheres[tag_id].header.frame_id == 'top_camera'
            assert spheres[tag_id].header.stamp == harness.info_ready.stamp

        plates = [m for m in harness.last('perception/detections').markers if m.ns == 'wellplates']
        assert len(plates) == 1 and plates[0].type == Marker.CUBE

        payload = json.loads(harness.last('perception/json').data)
        assert [t['id'] for t in payload['apriltags']] == [1, 21]
        assert payload['apriltags'][0]['measured_size_m'] == pytest.approx(0.0625, rel=0.05)
        assert payload['wellplates'][0]['name'] == 'wellplate' and payload['camera'] == 'zedx'

        cloud = harness.last('perception/points')
        assert cloud.header.frame_id == 'top_camera' and [f.name for f in cloud.fields] == ['x', 'y', 'z', 'rgb']
        points, _ = pointcloud2_to_numpy(cloud)
        assert len(points) > 10000
        assert np.abs((points - scene.origin) @ scene.rotation[:, 2]).max() < 0.005
        assert harness.last('perception/image').header.frame_id == 'top_camera'
        assert harness.last('perception/annotated_image').encoding == 'bgr8'
    finally:
        harness.close()


def test_snapshot_cloud_waits_for_the_octomap_clear_and_is_cached(ros, tmp_path):
    cache = tmp_path / 'snapshot.npy'
    namespace_service = 'clear_octomap'
    harness = Harness('snapshot', {'yolo.enable': False, 'apriltags.enable': False, 'pointcloud.enable': True,
                                   'pointcloud.mode': 'snapshot', 'pointcloud.snapshot_frames': 2,
                                   'pointcloud.max_rate': 20.0, 'pointcloud.voxel_size': 0.02,
                                   'pointcloud.cache_path': str(cache), 'pointcloud.fresh': True,
                                   'pointcloud.clear_octomap_service': namespace_service},
                      services=[namespace_service])
    try:
        harness.wait(lambda: False, timeout=0.5)
        assert not harness.calls                         # fresh: no clearing before there is a new cloud
        scene, _ = make_scene()
        for i in range(4):          # the cloud timer runs at 20 Hz: clouds are taken at most every 50 ms
            harness.publish(scene, stamp_offset=i)
            harness.wait(lambda: False, timeout=0.1)
        assert harness.wait(lambda: harness.received.get('perception/points'))
        assert harness.calls and harness.calls[0] <= harness.received['perception/points'][0][0]
        first = harness.last('perception/points')
        assert first.header.frame_id == FRAME
        assert cache.is_file() and np.load(cache).shape == (first.width, 4)
        # republished while no camera frames arrive
        count = len(harness.received['perception/points'])
        assert harness.wait(lambda: len(harness.received['perception/points']) >= count + 3, timeout=2.0)

        client = harness.io.create_client(Trigger, 'perception/capture_snapshot')
        assert client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(Trigger.Request())
        assert harness.wait(future.done) and future.result().success
        assert not harness.node.snapshot.ready
        calls = len(harness.calls)
        # without camera frames the old cloud stays in MoveIt: still republished, the octomap is not cleared
        count = len(harness.received['perception/points'])
        assert harness.wait(lambda: len(harness.received['perception/points']) >= count + 5, timeout=2.0)
        assert len(harness.calls) == calls
        assert all(msg.width == first.width for _, msg in harness.received['perception/points'])
        # a farther scene: once its cloud is captured, the octomap is cleared, then only the new cloud is published
        farther = Scene(rotation=rx(0.4), origin=(0.0, 0.05, 1.2))
        harness.received.pop('perception/points')
        is_new = lambda msg: np.median(pointcloud2_to_numpy(msg)[0][:, 2]) > 1.0   # noqa: E731
        for i in range(4):
            harness.publish(farther, stamp_offset=10 + i)
            harness.wait(lambda: False, timeout=0.1)
        assert harness.wait(lambda: any(is_new(m) for _, m in harness.received.get('perception/points', [])))
        assert len(harness.calls) == calls + 1           # a recapture clears the octomap again
        first_new = next(t for t, m in harness.received['perception/points'] if is_new(m))
        assert harness.calls[-1] <= first_new
        assert all(is_new(m) for t, m in harness.received['perception/points'] if t > harness.calls[-1])
    finally:
        harness.close()


def test_the_snapshot_cache_is_published_without_a_camera(ros, tmp_path):
    cache = tmp_path / 'pc3_accumulated.npy'
    np.save(cache, np.array([[0.0, 0.0, 1.0], [0.1, 0.0, 1.0]], np.float32))
    harness = Harness('cached', {'yolo.enable': False, 'apriltags.enable': False, 'pointcloud.enable': True,
                                 'pointcloud.mode': 'snapshot', 'pointcloud.cache_path': str(cache),
                                 'pointcloud.max_rate': 20.0, 'output_frame': 'top_camera'})
    try:
        assert harness.wait(lambda: len(harness.received.get('perception/points', [])) >= 2)
        cloud = harness.last('perception/points')
        assert cloud.header.frame_id == 'top_camera' and cloud.width == 2 and cloud.point_step == 12
    finally:
        harness.close()


def test_a_missing_yolo_model_only_disables_yolo(ros):
    harness = Harness('nomodel', {'yolo.model': '/nonexistent/best_top.pt'})
    try:
        assert harness.node.yolo is None and harness.node.tags is not None
        scene, _ = make_scene()
        harness.publish(scene)
        assert harness.wait(lambda: harness.received.get('perception/apriltags'))
        assert 'perception/detections' not in harness.received
        assert harness.node.pub_points is None
    finally:
        harness.close()


def test_invalid_parameters_fail_at_startup(ros):
    for overrides in ({'pointcloud.mode': 'stream'}, {'pointcloud.bounds_z': [1.0, 0.5]},
                      {'apriltags.position': 'stereo', 'yolo.enable': False},
                      {'pointcloud.enable': True, 'pointcloud.mode': 'snapshot', 'pointcloud.max_rate': 0.0,
                       'yolo.enable': False}):
        with pytest.raises(ValueError):
            PerceptionNode(namespace=f'/test_perception_{os.getpid()}_invalid', parameter_overrides=[
                Parameter(k, value=v) for k, v in overrides.items()])


def test_rate_limit_keeps_the_nominal_rate_from_faster_frames():
    frames = 1.0 + np.arange(0.0, 10.0, 1 / 16.0) + np.random.default_rng(2).uniform(-0.004, 0.004, 160)
    limit = RateLimit(0.1)
    assert 95 <= sum(limit.ready(t) for t in frames) <= 101      # 10 Hz from 16 fps, not every other frame (80)
    assert sum(limit.ready(t) for t in frames + 60.0) <= 101      # after a pause: no burst of catch-up frames
    fresh = RateLimit(0.2)
    assert [fresh.ready(t) for t in (100.0, 100.0625, 100.125, 100.19)] == [True, False, False, True]
    assert [fresh.ready(t) for t in (200.0, 200.0625)] == [True, False]   # nor two events right after a pause
    everything = RateLimit(0.0)
    assert all(everything.ready(t) for t in frames)
