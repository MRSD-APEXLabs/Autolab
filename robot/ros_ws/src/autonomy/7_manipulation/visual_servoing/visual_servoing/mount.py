"""Wrist geometry: xArm poses, the ZED X Nano mount and the gripper fingertips.

Frames (all right-handed, metres inside this package):

  base     the xArm base frame, z up.
  flange   the xArm tool frame with a zero TCP offset. z points out of the flange towards the fingers.
           The camera sits on the flange's +x side (measured from teleop episodes, see
           tools/estimate_mount_rotation.py), so the fingers close along y. The episodes put the whole
           mount ~7.6 deg further round the flange axis; that cancels (the plate and the fingers are both
           located through the camera, and the finger pads line up with the image x axis within 1.3 deg).
  tcp      what get_position() reports: flange_T_tcp is the controller's TCP offset, read from the arm.
  camera   the rectified LEFT optical frame of the ZED X Nano (x right, y down, z forward), the frame of
           the depth node's depth/image_rect and camera_info.
  tip      the centre between the fingertips, the point the grasp places on the plate.

The CAD measurements (Fusion 360, front view, X towards the camera, Z up) are taken from the "EE" point
on the underside of the mount plate: camera 90.711 mm out and 5.732 mm down, fingertips 109.104 mm down
and 7.838 mm to the side. Fusion X = flange x and Fusion Z = -flange z. The side of the 7.838 mm is taken
from the recorded wrist depth, not the CAD sign: the finger pads' midpoint lies 3.6 mm right of the left
lens (camera x), which fits the fingertips at flange y = -7.8 mm (1.1 mm), not +7.8 mm (16.8 mm).
Where the EE point sits on the flange axis (`ee_below_flange_mm`) moves camera and tip together, so it
changes neither the servo target nor the grasp: it only shifts absolute heights in logs and limits.

`tip_camera_mm` replaces the CAD fingertip with one measured in the camera frame (e.g. from the depth of
the fingers, which the camera sees at the bottom of the image). The recorded depth puts the finger pads
~16 mm further along the flange axis than the CAD fingertip, and the CAD fingertip projects onto the edge
of the gripper body in the image, below the pads. config/visual_servo.yaml uses the measured point until
that is settled (README): the class defaults stay the CAD numbers.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(roll, pitch, yaw):
    """xArm roll/pitch/yaw (radians): R = Rz(yaw) Ry(pitch) Rx(roll)."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_rpy(r):
    """Inverse of rpy_matrix (pitch in [-pi/2, pi/2])."""
    pitch = float(np.arcsin(np.clip(-r[2, 0], -1.0, 1.0)))
    if abs(np.cos(pitch)) > 1e-9:
        return float(np.arctan2(r[2, 1], r[2, 2])), pitch, float(np.arctan2(r[1, 0], r[0, 0]))
    return float(np.arctan2(-r[1, 2], r[1, 1])), pitch, 0.0


def transform(rotation=None, translation=None):
    t = np.eye(4)
    if rotation is not None:
        t[:3, :3] = rotation
    if translation is not None:
        t[:3, 3] = translation
    return t


def inverse(t):
    r, p = t[:3, :3], t[:3, 3]
    return transform(r.T, -r.T @ p)


def xarm_pose_matrix(pose):
    """4x4 (metres) of an xArm pose [x, y, z (mm), roll, pitch, yaw (deg)]."""
    x, y, z, roll, pitch, yaw = (float(v) for v in pose[:6])
    return transform(rpy_matrix(*np.radians([roll, pitch, yaw])), np.array([x, y, z]) / 1000.0)


def matrix_xarm_pose(t):
    """[x, y, z (mm), roll, pitch, yaw (deg)] of a 4x4 (metres)."""
    return [*(t[:3, 3] * 1000.0), *np.degrees(matrix_rpy(t[:3, :3]))]


def wrap_pi(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class Mount:
    """The ZED X Nano on its 20 degree bracket and the gripper, relative to the EE point of the CAD."""
    camera_offset_mm: tuple = (90.711, -0.078, 5.732)   # camera reference point: x towards it, z down (towards the fingers)
    tilt_deg: float = 20.0                             # optical axis tilted from flange z towards the fingers (bracket 110 deg)
    lens_offset_mm: tuple = (-9.0, 0.0, 0.0)           # left optical centre from that point, camera frame (baseline 18 mm)
    correction_rpy_deg: tuple = (0.0, 0.0, 0.0)        # extra camera rotation about its own x, y, z (calibration trim)
    tip_mm: tuple = (-1.022, -7.838, 109.104)          # fingertip centre from the EE point, flange frame
    ee_below_flange_mm: float = 0.0                    # EE point along flange z; cancels out of the servo (see above)
    tip_camera_mm: tuple | None = None                 # measured fingertip centre in the camera frame; overrides tip_mm
    flange_T_camera: np.ndarray = field(init=False, repr=False)
    flange_T_tip: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        t = np.radians(float(self.tilt_deg))
        # optical axis: flange z tipped towards -x (the fingers); image down: towards the fingers; image right: +y
        model = np.column_stack([[0.0, 1.0, 0.0],
                                 [-np.cos(t), 0.0, -np.sin(t)],
                                 [-np.sin(t), 0.0, np.cos(t)]])
        correction = rpy_matrix(*np.radians(np.asarray(self.correction_rpy_deg, dtype=float)))
        rotation = model @ correction
        ee = np.array([0.0, 0.0, float(self.ee_below_flange_mm)]) / 1000.0
        origin = ee + np.asarray(self.camera_offset_mm, dtype=float) / 1000.0 \
            + rotation @ (np.asarray(self.lens_offset_mm, dtype=float) / 1000.0)
        self.flange_T_camera = transform(rotation, origin)
        if self.tip_camera_mm is not None:
            tip = (self.flange_T_camera @ np.append(np.asarray(self.tip_camera_mm, dtype=float) / 1000.0, 1.0))[:3]
        else:
            tip = ee + np.asarray(self.tip_mm, dtype=float) / 1000.0
        self.flange_T_tip = transform(None, tip)

    @classmethod
    def from_params(cls, params):
        """From a dict of the `mount.*` parameters (missing keys keep the CAD values)."""
        known = {'camera_offset_mm', 'tilt_deg', 'lens_offset_mm', 'correction_rpy_deg', 'tip_mm', 'ee_below_flange_mm',
                 'tip_camera_mm'}
        kwargs = {}
        for key, value in params.items():
            if key not in known:
                raise ValueError(f'unknown mount parameter {key!r}; expected one of {sorted(known)}')
            if key == 'tip_camera_mm' and (value is None or len(value) == 0):
                continue   # [] in YAML: use the CAD fingertip
            kwargs[key] = tuple(float(v) for v in value) if isinstance(value, (list, tuple)) else float(value)
        for key in ('camera_offset_mm', 'lens_offset_mm', 'correction_rpy_deg', 'tip_mm', 'tip_camera_mm'):
            if key in kwargs and len(kwargs[key]) != 3:
                raise ValueError(f'mount.{key} needs 3 values, got {list(kwargs[key])}')
        return cls(**kwargs)

    def tip_in_camera(self):
        """The fingertip centre in the camera frame (m)."""
        return (inverse(self.flange_T_camera) @ self.flange_T_tip)[:3, 3]

    def project(self, point_camera, intrinsics):
        """Pixel (u, v) of a camera-frame point, or None behind the camera."""
        x, y, z = (float(v) for v in point_camera)
        if z <= 1e-6:
            return None
        return intrinsics.fx * x / z + intrinsics.cx, intrinsics.fy * y / z + intrinsics.cy

    def describe(self):
        tip = self.tip_in_camera() * 1000.0
        cam = self.flange_T_camera[:3, 3] * 1000.0
        return (f'camera at flange ({cam[0]:.1f}, {cam[1]:.1f}, {cam[2]:.1f}) mm, tilted {self.tilt_deg:.1f} deg; '
                f'fingertips at camera ({tip[0]:.1f}, {tip[1]:.1f}, {tip[2]:.1f}) mm')
