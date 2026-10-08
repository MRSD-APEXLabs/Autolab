"""Depth geometry: plane fits, ray casts, flat-object poses and quaternions."""
import numpy as np
import pytest

from camera_perception.geometry import (Intrinsics, Pose, depth_at, fit_plane, obb_corners, planar_pose,
                                        points_in_polygon, ray_plane, rotation_to_quaternion)
from scene import INTRINSICS, Scene, angle_between, rx, ry, rz


def test_intrinsics_from_camera_info_and_back_projection():
    class Info:
        k = [365.0, 0.0, 479.5, 0.0, 366.0, 299.5, 0.0, 0.0, 1.0]
        width, height = 960, 600
    k = Intrinsics.from_camera_info(Info)
    assert (k.fx, k.fy, k.cx, k.cy, k.width, k.height) == (365.0, 366.0, 479.5, 299.5, 960, 600)
    point = k.backproject(479.5 + 36.5, 299.5 - 36.6, 2.0)
    np.testing.assert_allclose(point, [0.2, -0.2, 2.0])


@pytest.mark.parametrize('rotation', [np.eye(3), rz(np.pi / 2), rx(0.7) @ ry(-0.4) @ rz(2.5), rx(np.pi), ry(np.pi - 1e-3)])
def test_quaternion_matches_the_rotation(rotation):
    x, y, z, w = rotation_to_quaternion(rotation)
    assert w >= 0.0 and np.isclose(x * x + y * y + z * z + w * w, 1.0)
    rebuilt = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    np.testing.assert_allclose(rebuilt, rotation, atol=1e-9)
    np.testing.assert_allclose(rotation_to_quaternion(rz(np.pi / 2)), [0, 0, np.sqrt(0.5), np.sqrt(0.5)])


def test_pose_as_dict_is_the_camera_edge_format():
    d = Pose(np.eye(3), np.array([0.1, 0.2, 0.3])).as_dict()
    assert d == {'position': [0.1, 0.2, 0.3], 'orientation': [0.0, 0.0, 0.0, 1.0]}


def test_depth_at_is_the_median_of_valid_pixels():
    depth = np.zeros((10, 10), np.uint16)
    assert depth_at(depth, 5, 5) is None
    depth[4:7, 4:7] = [[1000, 1000, 1000], [1000, 5000, 1000], [1200, 1200, 1200]]
    assert depth_at(depth, 5, 5, half_window=1) == 1.0
    assert depth_at(depth, 0, 0, half_window=1) is None      # window clipped at the border, all invalid
    depth[:] = 1500
    for u, v in ((-20, 5), (5, -3), (10, 5), (5, 12)):     # outside the image, e.g. a box cut by the border
        assert depth_at(depth, u, v) is None


def test_points_in_polygon_back_projects_the_depth_inside_it():
    scene = Scene(origin=(0.0, 0.0, 0.8))
    depth = scene.depth_mm()
    square = [(400, 200), (560, 200), (560, 400), (400, 400)]
    points = points_in_polygon(depth, INTRINSICS, square, step=4)
    assert 40 * 50 * 0.9 < len(points) <= 41 * 51
    np.testing.assert_allclose(points[:, 2], 0.8)
    u = points[:, 0] / points[:, 2] * INTRINSICS.fx + INTRINSICS.cx
    assert u.min() >= 399.5 and u.max() <= 560.5
    depth[:] = 0
    assert len(points_in_polygon(depth, INTRINSICS, square)) == 0
    assert len(points_in_polygon(depth, INTRINSICS, [(-50, -50), (-10, -50), (-10, -10)])) == 0   # off the image


def test_fit_plane_rejects_outliers_and_faces_the_camera():
    rng = np.random.default_rng(0)
    normal = rx(0.5) @ np.array([0.0, 0.0, -1.0])
    basis = rx(0.5)[:, :2]
    points = np.array([0.0, 0.1, 1.0]) + (rng.uniform(-0.1, 0.1, (400, 2)) @ basis.T)
    points += rng.normal(0.0, 0.001, points.shape)
    points[:20, 2] -= 0.3                                      # a few flying pixels
    centroid, fitted = fit_plane(points)
    assert fitted @ centroid < 0.0                             # towards the camera at the origin
    assert np.degrees(np.arccos(abs(fitted @ normal))) < 1.0
    assert fit_plane(points[:10]) is None


def test_ray_plane():
    centroid, normal = np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, -1.0])
    np.testing.assert_allclose(ray_plane(INTRINSICS, INTRINSICS.cx + 36.5, INTRINSICS.cy, centroid, normal), [0.1, 0, 1])
    assert ray_plane(INTRINSICS, 100, 100, centroid, np.array([1.0, 0.0, 0.0])) is None     # parallel
    assert ray_plane(INTRINSICS, 480, 300, -centroid, normal) is None                      # behind the camera


def test_obb_corners_put_the_width_along_theta():
    corners = obb_corners((100, 50), (40, 20), np.pi / 2)
    np.testing.assert_allclose(corners.mean(axis=0), [100, 50])
    np.testing.assert_allclose(np.linalg.norm(corners[1] - corners[0]), 40)
    np.testing.assert_allclose((corners[1] - corners[0]) / 40, [0, 1], atol=1e-12)   # theta 90 deg: width along +v


@pytest.mark.parametrize('tilt', [0.0, 0.6])
def test_planar_pose_of_a_rectangle_on_a_tilted_plane(tilt):
    scene = Scene(rotation=rx(tilt), origin=(0.05, 0.02, 0.9))
    depth = scene.depth_mm()
    # the image of a 12 x 8 cm rectangle at plane angle 0.4 rad (theta in the image = its projected width direction)
    rotation, centre = scene.tag_pose(angle=0.4)
    corners3d = [centre + rotation @ np.array([sx * 0.06, sy * 0.04, 0.0]) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    image = np.array([[INTRINSICS.fx * p[0] / p[2] + INTRINSICS.cx, INTRINSICS.fy * p[1] / p[2] + INTRINSICS.cy]
                      for p in corners3d])
    x_dir = image[1] - image[0]
    theta = float(np.arctan2(x_dir[1], x_dir[0]))
    size = (float(np.linalg.norm(image[1] - image[0])), float(np.linalg.norm(image[2] - image[1])))
    pose, metric, source = planar_pose(depth, INTRINSICS, image.mean(axis=0), size, theta, polygon=image)
    assert source == 'plane'
    assert np.linalg.norm(pose.position - centre) < 0.01
    assert np.degrees(np.arccos(pose.rotation[:, 2] @ -rotation[:, 2])) < 1.0     # z: the normal, towards the camera
    assert np.degrees(np.arccos(pose.rotation[:, 0] @ rotation[:, 0])) < 3.0      # x: along the width
    np.testing.assert_allclose(np.linalg.det(pose.rotation), 1.0)
    if tilt == 0.0:
        np.testing.assert_allclose(metric, (0.12, 0.08), rtol=0.03)


def test_planar_pose_falls_back_to_the_centre_depth_and_to_none():
    depth = np.zeros((600, 960), np.uint16)
    depth[295:306, 475:486] = 1000                      # only a few pixels at the centre
    pose, metric, source = planar_pose(depth, INTRINSICS, (480, 300), (73, 36.5), 0.0)
    assert source == 'center_depth'
    np.testing.assert_allclose(pose.position, [0.5 / 365.0, 0.5 / 365.0, 1.0], atol=1e-3)
    np.testing.assert_allclose(pose.rotation[:, 2], [0, 0, -1])
    np.testing.assert_allclose(metric, (0.2, 0.1), rtol=0.01)
    assert angle_between(pose.rotation, pose.rotation) == 0.0
    assert planar_pose(np.zeros((600, 960), np.uint16), INTRINSICS, (480, 300), (73, 36.5), 0.0) is None
