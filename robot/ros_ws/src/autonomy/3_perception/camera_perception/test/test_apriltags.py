"""AprilTag detection and poses on rendered tags with exact depth."""
import numpy as np
import pytest

pytest.importorskip('pupil_apriltags')
from camera_perception.apriltags import TagDetector, object_points, parse_sizes  # noqa: E402
from scene import INTRINSICS, Scene, angle_between, rx, ry, rz  # noqa: E402

IDS = (0, 1, 2, 21, 22)
# For tags drawn by OpenCV (cv2.aruco DICT_APRILTAG_36h11) the detector's tag frame is the drawing turned 180 deg
# about z; tags printed from the AprilTag images follow the library's frame (x right, y down, z into the tag).
DRAWN_TO_TAG = rz(np.pi)


@pytest.fixture(scope='module')
def detector():
    return TagDetector(ids=IDS)


def test_parse_sizes():
    assert parse_sizes('') == {}
    assert parse_sizes('21=0.075, 22:0.1  5=0.03') == {21: 0.075, 22: 0.1, 5: 0.03}
    for bad in ('21', '21=0', 'x=0.1', '21=abc'):
        with pytest.raises(ValueError):
            parse_sizes(bad)


def test_object_points_are_in_the_ippe_square_order():
    np.testing.assert_allclose(object_points(2.0), [[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]])


def in_image_angle(r1, r2):
    """Angle (deg) between the x axes of two rotations as the camera sees them (projected on its image plane)."""
    a, b = r1[:2, 0], r2[:2, 0]
    return float(np.degrees(np.arccos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1.0, 1.0))))


# A face-on 40 px tag is PnP's worst case: a tenth of a pixel of corner error tilts the pose by several
# degrees (the depth only picks the better of the two mirror solutions). The in-image angle stays sharp.
@pytest.mark.parametrize('rotation, angle, max_tilt', [(np.eye(3), 0.0, 12.0), (rx(0.6), 0.5, 4.0),
                                                       (ry(-0.5) @ rx(0.3), -2.0, 4.0)],
                         ids=['face-on', 'tilted', 'oblique'])
def test_pose_of_a_rendered_tag(detector, rotation, angle, max_tilt):
    scene = Scene(rotation=rotation, origin=(0.04, -0.03, 0.55))
    truth_rotation, centre = scene.add_tag(21, 0.0625, angle=angle)
    tags = detector.detect(scene.image, scene.depth_mm(), INTRINSICS)
    assert [t.id for t in tags] == [21]
    tag = tags[0]
    assert tag.pose_source == 'pnp' and tag.hamming == 0
    assert np.linalg.norm(tag.pose.position - centre) < 0.01
    expected = truth_rotation @ DRAWN_TO_TAG
    assert angle_between(expected, tag.pose.rotation) < max_tilt
    assert in_image_angle(expected, tag.pose.rotation) < 2.0
    assert tag.measured_size_m == pytest.approx(0.0625, rel=0.04)
    assert tag.reprojection_px < 0.5
    np.testing.assert_allclose(tag.center, INTRINSICS.matrix[:2] @ (centre / centre[2]), atol=0.6)
    assert tag.as_dict()['pose']['position'] == [float(v) for v in tag.pose.position]


def test_only_the_configured_ids_are_kept(detector):
    scene = Scene(origin=(0.0, 0.0, 0.6))
    for tag_id, x in ((1, -0.15), (5, 0.0), (21, 0.15)):
        scene.add_tag(tag_id, 0.0625, xy=(x, 0.0))
    image, depth = scene.image, scene.depth_mm()
    assert [t.id for t in detector.detect(image, depth, INTRINSICS)] == [1, 21]
    assert [t.id for t in TagDetector(ids=()).detect(image, depth, INTRINSICS)] == [1, 5, 21]


def test_a_wrong_size_scales_pnp_but_not_the_depth():
    scene = Scene(origin=(0.0, 0.02, 0.6))
    _, centre = scene.add_tag(21, 0.075)                    # printed 75 mm
    image, depth = scene.image, scene.depth_mm()
    pnp = TagDetector(ids=IDS, size=0.0625).detect(image, depth, INTRINSICS)[0]
    assert pnp.pose.position[2] == pytest.approx(0.6 * 0.0625 / 0.075, rel=0.03)
    assert pnp.measured_size_m == pytest.approx(0.075, rel=0.04)
    from_depth = TagDetector(ids=IDS, size=0.0625, position='depth').detect(image, depth, INTRINSICS)[0]
    assert from_depth.pose_source == 'depth'
    assert np.linalg.norm(from_depth.pose.position - centre) < 0.005
    np.testing.assert_allclose(from_depth.pose.rotation, pnp.pose.rotation)
    sized = TagDetector(ids=IDS, size=0.0625, sizes={21: 0.075}).detect(image, depth, INTRINSICS)[0]
    assert sized.size_m == 0.075 and np.linalg.norm(sized.pose.position - centre) < 0.01


def test_without_depth_the_pose_comes_from_the_corners_alone(detector):
    scene = Scene(rotation=rx(0.5), origin=(0.0, 0.0, 0.5))
    truth_rotation, centre = scene.add_tag(1, 0.0625, angle=0.3)
    tag = detector.detect(scene.image, None, INTRINSICS)[0]
    assert tag.measured_size_m is None and tag.pose_source == 'pnp'
    assert np.linalg.norm(tag.pose.position - centre) < 0.01
    assert angle_between(truth_rotation @ DRAWN_TO_TAG, tag.pose.rotation) < 4.0
    bare = detector.detect(scene.image)[0]                  # no intrinsics: image detection only
    assert bare.pose is None and bare.id == 1


def test_invalid_position_mode():
    with pytest.raises(ValueError):
        TagDetector(position='stereo')
