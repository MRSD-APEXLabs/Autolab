"""Plate estimates in the base frame, the filter and the servo law."""
import numpy as np
import pytest

from sim import INTRINSICS, PLATE, detect_plate, tool_down
from visual_servoing.control import (Gains, PlateEstimate, PlateFilter, PoseHistory, camera_point, finger_angle_offset,
                                     grasp_yaw, hover_target, plate_in_base, servo_command)
from visual_servoing.mount import Mount, transform, wrap_pi, xarm_pose_matrix

MOUNT = Mount()
CENTER = np.array([0.42, 0.03, 0.02])


def axis_angle_deg(axis):
    return np.degrees(np.arctan2(axis[1], axis[0]))


def axial_error_deg(a, b):
    """Difference of two axis directions (mod 180), degrees."""
    return abs(np.degrees(wrap_pi(2 * np.radians(a - b)) / 2))


def test_pose_history():
    history = PoseHistory(horizon=1.0)
    assert history.at(0.0) is None and history.latest() is None
    for t in range(5):
        history.add(10.0 + 0.1 * t, transform(None, [t * 0.01, 0.0, 0.0]))
    history.add(10.1, transform(None, [9.0, 9.0, 9.0]))                  # out of order: ignored
    assert np.allclose(history.at(10.15)[:3, 3], [0.015, 0, 0])          # interpolated
    assert np.allclose(history.at(10.0)[:3, 3], [0, 0, 0])
    assert np.allclose(history.at(10.5)[:3, 3], [0.04, 0, 0])            # newest, within max_gap
    assert history.at(10.7) is None and history.at(9.5) is None
    history.add(11.35, np.eye(4))                                        # trims older than the horizon
    assert history.at(10.2) is None and history.at(10.4) is not None
    assert np.allclose(history.latest(), np.eye(4))


@pytest.mark.parametrize('yaw_deg, long_deg, width', [(0, 30, 'long'), (40, -20, 'short'), (-60, 95, 'long')])
def test_plate_in_base_with_depth(yaw_deg, long_deg, width):
    base_T_camera = tool_down(0.40, 0.0, 0.35, yaw_deg) @ MOUNT.flange_T_camera
    det = detect_plate(base_T_camera, CENTER, np.radians(long_deg), width=width)
    est = plate_in_base(det, INTRINSICS, base_T_camera, 5.0, PLATE)
    assert est.source == 'plane' and est.stamp == 5.0
    assert np.allclose(est.center, CENTER, atol=1e-6)
    assert axial_error_deg(axis_angle_deg(est.short_axis), long_deg + 90) < 1.5   # perspective skew, averaged
    assert np.allclose(est.size, PLATE, rtol=0.03)


def test_plate_in_base_from_the_box_size_without_depth():
    base_T_camera = tool_down(0.40, 0.0, 0.35) @ MOUNT.flange_T_camera
    det = detect_plate(base_T_camera, CENTER, np.radians(30), with_depth=False)
    assert plate_in_base(det, INTRINSICS, base_T_camera, 0.0) is None     # no depth, no size: can't place it
    est = plate_in_base(det, INTRINSICS, base_T_camera, 0.0, PLATE)
    assert est.source == 'size'
    assert np.linalg.norm(est.center - CENTER) < 0.01                    # the Camera-Edge estimate: roughly right


def estimate(center, short_deg, stamp=0.0):
    a = np.radians(short_deg)
    return PlateEstimate(stamp, np.asarray(center, dtype=float), np.array([np.cos(a), np.sin(a)]), PLATE, 'plane', 0.9)


def test_plate_filter_averages_axes_and_gates_outliers():
    f = PlateFilter(alpha=0.5, gate=0.03, reset_after=3)
    assert f.short_axis is None
    assert f.update(estimate(CENTER, 89.0, 1.0))
    assert f.update(estimate(CENTER + [0.002, 0, 0], -89.0, 2.0))       # the same axis, box flipped by 180
    assert axial_error_deg(axis_angle_deg(f.short_axis), 90.0) < 0.1
    assert np.allclose(f.center, CENTER + [0.001, 0, 0]) and f.count == 2 and f.last_stamp == 2.0
    moved = CENTER + [0.1, 0, 0]
    assert not f.update(estimate(moved, 0.0, 3.0))                      # a jump: dropped...
    assert not f.update(estimate(moved, 0.0, 4.0))
    assert np.allclose(f.center, CENTER + [0.001, 0, 0])
    assert f.update(estimate(moved, 0.0, 5.0))                          # ...until it persists: the plate moved
    assert np.allclose(f.center, moved) and f.count == 3
    assert axial_error_deg(axis_angle_deg(f.short_axis), 0.0) < 0.1


def test_grasp_yaw_lines_the_fingers_up_with_the_short_side():
    for short_deg in (0, 30, 90, 135, -170):
        for current in (-170, -45, 0, 60, 179):
            yaw = grasp_yaw(np.array([np.cos(np.radians(short_deg)), np.sin(np.radians(short_deg))]),
                            np.radians(current))
            fingers = np.degrees(yaw + finger_angle_offset('y'))
            # the flange's y axis (the fingers) in the base frame, tool pointing down
            flange_y = tool_down(0, 0, 0, np.degrees(yaw))[:3, 1]
            assert np.isclose(axis_angle_deg(flange_y[:2]), wrap_pi(np.radians(fingers)) * 180 / np.pi)
            assert axial_error_deg(fingers, short_deg) < 1e-6
            assert abs(np.degrees(wrap_pi(yaw - np.radians(current)))) <= 90.0 + 1e-9   # the nearer of the two
    yaw = grasp_yaw(np.array([1.0, 0.0]), 0.0, finger_axis='x')
    assert np.isclose(yaw, 0.0)


def test_servo_command_moves_the_fingertip_and_saturates():
    mount = Mount()
    flange_T_tcp = xarm_pose_matrix([0, 0, 140, 0, 0, 0])
    pose = tool_down(0.40, 0.0, 0.35, 10.0)
    tip = (pose @ mount.flange_T_tip)[:3, 3]
    gains = Gains()
    cmd = servo_command(pose, mount.flange_T_tip, flange_T_tcp, tip + [0.005, 0.0, -0.001], np.radians(10.5), gains)
    assert np.allclose(cmd.error_xyz, [0.005, 0, -0.001]) and np.isclose(np.degrees(cmd.error_yaw), 0.5)
    assert not cmd.within
    # the TCP velocity with the rotation about the TCP gives the fingertip exactly k * error
    tcp = (pose @ flange_T_tcp)[:3, 3]
    v_tip = cmd.v_tcp + np.cross([0, 0, cmd.wz], tip - tcp)
    assert np.allclose(v_tip, [0.005, 0.0, -0.001]) and np.isclose(cmd.wz, np.radians(0.5))

    far = servo_command(pose, mount.flange_T_tip, flange_T_tcp, tip + [0.3, 0.4, -0.2], np.radians(100), gains)
    v_tip = far.v_tcp + np.cross([0, 0, far.wz], tip - tcp)
    assert np.isclose(np.linalg.norm(v_tip[:2]), gains.max_xy) and np.isclose(v_tip[2], -gains.max_z)
    assert np.allclose(v_tip[:2] / np.linalg.norm(v_tip[:2]), [0.6, 0.8])      # saturated, same direction
    assert np.isclose(far.wz, gains.max_yaw)

    close = servo_command(pose, mount.flange_T_tip, flange_T_tcp, tip + [0.001, 0.001, 0.003], np.radians(11), gains)
    assert close.within


def test_hover_target_and_camera_point():
    assert np.allclose(hover_target(CENTER, 0.08, (0.01, -0.02)), CENTER + [0.01, -0.02, 0.08])
    base_T_camera = tool_down(0.40, 0.0, 0.35) @ MOUNT.flange_T_camera
    p = camera_point(base_T_camera, CENTER)
    assert np.allclose((base_T_camera @ np.append(p, 1))[:3], CENTER)
