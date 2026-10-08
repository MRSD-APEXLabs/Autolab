"""Marker arrays in the pc3.py layout that move_to_pose_node reads, annotation and JSON."""
import json

import numpy as np
import pytest

pytest.importorskip('visualization_msgs')
from std_msgs.msg import Header  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

from camera_perception.apriltags import Tag  # noqa: E402
from camera_perception.geometry import Pose  # noqa: E402
from camera_perception.markers import annotate, apriltag_markers, detection_markers, summary_json  # noqa: E402
from camera_perception.yolo import Detection  # noqa: E402
from scene import INTRINSICS, rz  # noqa: E402

HEADER = Header(frame_id='top_camera')


def tag(tag_id, position=None):
    corners = np.array([[510, 310], [490, 310], [490, 290], [510, 290]], np.float64)
    pose = Pose(rz(np.pi), np.asarray(position, np.float64)) if position is not None else None
    return Tag(tag_id, corners.mean(axis=0), corners, 0.0625, pose=pose, pose_source='pnp' if pose else None)


def detection(conf, position=(0.0, 0.0, 1.0), name='wellplate'):
    polygon = np.array([[400, 250], [560, 250], [560, 350], [400, 350]], np.float64)
    pose = Pose(np.diag([1.0, -1.0, -1.0]), np.asarray(position, np.float64)) if position is not None else None
    return Detection(0, name, conf, polygon.mean(axis=0), np.array([160.0, 100.0]), 0.0, polygon, pose,
                     (0.12, 0.08) if pose else None, 'plane' if pose else None)


def test_apriltag_markers_hold_only_tag_spheres():
    array = apriltag_markers(HEADER, [tag(1, (0.1, 0.2, 0.9)), tag(21, (0.0, 0.0, 1.0)), tag(2)])
    first, *spheres = array.markers
    assert first.action == Marker.DELETEALL
    assert [(m.id, m.ns, m.type, m.action) for m in spheres] == [(1, 'apriltags', Marker.SPHERE, Marker.ADD),
                                                                  (21, 'apriltags', Marker.SPHERE, Marker.ADD)]
    assert spheres[0].header.frame_id == 'top_camera'
    assert (spheres[0].pose.position.x, spheres[0].pose.position.y, spheres[0].pose.position.z) == (0.1, 0.2, 0.9)
    q = spheres[0].pose.orientation
    np.testing.assert_allclose([q.x, q.y, q.z, q.w], [0, 0, 1, 0], atol=1e-12)
    assert spheres[0].scale.x == 0.02 and spheres[0].color.r == 1.0


def test_detection_markers_end_with_the_most_confident():
    array = detection_markers(HEADER, [detection(0.95, (0.3, 0, 1)), detection(0.85, (0.1, 0, 1)),
                                       detection(0.99, None)], ns='wellplates')
    first, *markers = array.markers
    assert first.action == Marker.DELETEALL
    cubes = [m for m in markers if m.ns == 'wellplates']
    texts = [m for m in markers if m.ns == 'wellplates_text']
    assert [m.pose.position.x for m in cubes] == [0.1, 0.3]      # ascending confidence; pose-less one dropped
    assert [m.id for m in cubes] == [0, 1] and [m.id for m in texts] == [1000, 1001]
    assert cubes[-1].type == Marker.CUBE and (cubes[-1].scale.x, cubes[-1].scale.y) == (0.12, 0.08)
    assert texts[-1].text == 'wellplate 0.95' and texts[-1].type == Marker.TEXT_VIEW_FACING
    assert len(detection_markers(HEADER, []).markers) == 1


def test_annotate_draws_on_a_copy():
    image = np.full((600, 960, 3), 128, np.uint8)
    out = annotate(image, [tag(21, (0.0, 0.0, 1.0))], [detection(0.9)], INTRINSICS)
    assert out.shape == image.shape and (out != 128).any()
    assert (image == 128).all()
    assert (annotate(image, [tag(3)], [detection(0.5, None)]) != 128).any()   # without poses or intrinsics


def test_summary_json():
    header = Header(frame_id='top_camera')
    header.stamp.sec, header.stamp.nanosec = 12, 34
    payload = json.loads(summary_json(header, [tag(1, (0, 0, 1))], [detection(0.9)], 'wellplates', {'camera': 'zedx'}))
    assert payload['stamp'] == {'sec': 12, 'nanosec': 34} and payload['frame_id'] == 'top_camera'
    assert payload['camera'] == 'zedx'
    assert payload['apriltags'][0]['id'] == 1 and payload['apriltags'][0]['pose']['position'] == [0, 0, 1]
    assert payload['wellplates'][0]['name'] == 'wellplate' and payload['wellplates'][0]['metric_size'] == [0.12, 0.08]
