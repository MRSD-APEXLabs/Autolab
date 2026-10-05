"""Mount geometry: frames, the CAD numbers and the parameter parsing."""
import numpy as np
import pytest

from sim import INTRINSICS
from visual_servoing.mount import (Mount, inverse, matrix_rpy, matrix_xarm_pose, rpy_matrix, transform, wrap_pi,
                                   xarm_pose_matrix)


def test_rpy_convention_and_round_trips():
    r = rpy_matrix(np.pi, 0.0, 0.0)
    assert np.allclose(r @ [0, 0, 1], [0, 0, -1])            # roll 180: the tool points down
    angles = (0.3, -0.4, 2.0)
    assert np.allclose(matrix_rpy(rpy_matrix(*angles)), angles)
    pose = [400.0, -20.0, 300.0, 180.0, 0.0, 30.0]
    back = matrix_xarm_pose(xarm_pose_matrix(pose))
    assert np.allclose(back[:3], pose[:3])
    assert np.allclose(xarm_pose_matrix(back), xarm_pose_matrix(pose))
    t = transform(rpy_matrix(*angles), [0.1, 0.2, 0.3])
    assert np.allclose(inverse(t) @ t, np.eye(4))
    assert np.isclose(wrap_pi(3 * np.pi / 2), -np.pi / 2)


def test_camera_on_the_20_degree_bracket():
    mount = Mount()
    rotation, origin = mount.flange_T_camera[:3, :3], mount.flange_T_camera[:3, 3]
    assert np.allclose(rotation.T @ rotation, np.eye(3)) and np.isclose(np.linalg.det(rotation), 1.0)
    optical = rotation[:, 2]
    assert np.isclose(np.degrees(np.arccos(optical @ [0, 0, 1])), 20.0)   # 20 deg off the flange axis
    assert optical[0] < 0.0                                                # tipped towards the fingers
    assert np.allclose(rotation[:, 0], [0, 1, 0])                          # image right = flange +y
    # CAD point (90.711, -0.078, 5.732) mm, left lens 9 mm to the image left
    assert np.allclose(origin * 1000, [90.711, -0.078 - 9.0, 5.732])


def test_fingertips_project_to_the_bottom_of_the_wrist_image():
    mount = Mount()
    tip = mount.tip_in_camera()
    assert 0.10 < tip[2] < 0.15                   # ~13 cm in front of the lens
    u, v = mount.project(tip, INTRINSICS)
    assert 200 < u < 290 and 230 < v < 300        # where the finger pads are in the recorded episodes
    assert mount.project([0.0, 0.0, -0.1], INTRINSICS) is None
    assert 'tilted 20.0 deg' in mount.describe()


def test_ee_depth_cancels_and_measured_tip_overrides():
    base = Mount()
    shifted = Mount(ee_below_flange_mm=25.0)
    assert np.allclose(shifted.tip_in_camera(), base.tip_in_camera())
    assert np.allclose(shifted.flange_T_tip[:3, 3] - base.flange_T_tip[:3, 3], [0, 0, 0.025])
    measured = Mount(tip_camera_mm=tuple(base.tip_in_camera() * 1000 + [0, 0, 16]))
    assert np.allclose(measured.tip_in_camera(), base.tip_in_camera() + [0, 0, 0.016])


def test_correction_rotates_about_the_camera_axes():
    base, trimmed = Mount(), Mount(correction_rpy_deg=(0.0, 0.0, 5.0))
    assert np.allclose(trimmed.flange_T_camera[:3, 2], base.flange_T_camera[:3, 2])   # about the optical axis
    angle = np.degrees(np.arccos(trimmed.flange_T_camera[:3, 0] @ base.flange_T_camera[:3, 0]))
    assert np.isclose(angle, 5.0)


def test_from_params():
    mount = Mount.from_params({'tilt_deg': 21.4, 'tip_mm': [0, 0, 120], 'tip_camera_mm': []})
    assert mount.tilt_deg == 21.4 and mount.tip_camera_mm is None
    assert np.allclose(mount.flange_T_tip[:3, 3], [0, 0, 0.12])
    with pytest.raises(ValueError, match='unknown mount parameter'):
        Mount.from_params({'tilt': 20})
    with pytest.raises(ValueError, match='needs 3 values'):
        Mount.from_params({'tip_mm': [0, 109]})
