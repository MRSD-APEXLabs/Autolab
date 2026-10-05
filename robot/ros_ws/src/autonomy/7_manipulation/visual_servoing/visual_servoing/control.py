"""Position-based visual servo: plate detections (camera frame, with depth) -> fingertip target -> velocity.

The Camera-Edge servo was image based: it drove the plate's box centre to the image centre and its box
area up to a threshold, then moved by a hand-tuned offset (dx, dy) = (90, -62) mm and a depth estimated
from the box area. Here the stereo depth gives the plate's 3D position, and the mount model (mount.py)
maps it into the base frame through the arm pose at the image's capture time. The plate lies still in the
base frame, so its estimates are averaged there, and the command is the fingertip's error to a hover pose
above it. The offset between camera and fingers is the mount's, not a tuned constant.

Units: metres, radians, seconds. Base frame z is up; the tool points down (roll 180 deg, pitch 0).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import bisect
import threading

import numpy as np

from .mount import inverse, wrap_pi


@dataclass
class PlateEstimate:
    """One plate observation in the base frame."""
    stamp: float                  # image capture time (s)
    center: np.ndarray            # (3,) top-face centre
    short_axis: np.ndarray        # (2,) unit, horizontal direction of the plate's short side (+/- equivalent)
    size: tuple                   # (long, short) metres on the top face
    source: str                   # 'plane' / 'center_depth' (depth) or 'size' (box area)
    conf: float


class PoseHistory:
    """Flange poses (base_T_flange) by time, for the arm pose at an image's capture time."""

    def __init__(self, horizon=2.0):
        self.horizon = float(horizon)
        self._times = deque()
        self._poses = deque()
        self._lock = threading.Lock()   # the control loop writes, the node's annotation reads

    def add(self, stamp, pose):
        with self._lock:
            if self._times and stamp <= self._times[-1]:
                return
            self._times.append(float(stamp))
            self._poses.append(np.asarray(pose, dtype=float))
            while self._times and self._times[0] < stamp - self.horizon:
                self._times.popleft()
                self._poses.popleft()

    def clear(self):
        with self._lock:
            self._times.clear()
            self._poses.clear()

    def latest(self):
        with self._lock:
            return self._poses[-1].copy() if self._poses else None

    def at(self, stamp, max_gap=0.2):
        """Pose at `stamp`: translation interpolated, rotation of the nearer sample; None outside the history
        (older than its start, or more than `max_gap` s after its newest sample)."""
        with self._lock:
            if not self._times or stamp < self._times[0] - 1e-3 or stamp > self._times[-1] + max_gap:
                return None
            times, poses = list(self._times), list(self._poses)
        i = bisect.bisect_left(times, stamp)
        if i == 0:
            return poses[0].copy()
        if i >= len(times):
            return poses[-1].copy()
        t0, t1 = times[i - 1], times[i]
        a = (stamp - t0) / max(t1 - t0, 1e-9)
        pose = (poses[i - 1] if a < 0.5 else poses[i]).copy()
        pose[:3, 3] = (1.0 - a) * poses[i - 1][:3, 3] + a * poses[i][:3, 3]
        return pose


def _horizontal_hit(origin, direction, height):
    """Where the ray origin + s*direction (s > 0) meets the plane z = height, or None."""
    if abs(direction[2]) < 1e-6:
        return None
    s = (height - origin[2]) / direction[2]
    return origin + s * direction if s > 0.0 else None


def plate_in_base(detection, intrinsics, base_T_camera, stamp, plate_size=None):
    """PlateEstimate of a camera_perception Detection, or None if it can't be placed.

    The centre is the detection's depth pose (plane fit, or the median depth at the box centre). Without
    depth it falls back to the Camera-Edge pinhole estimate from the box area and the known plate size
    (`plate_size` = (long, short) m). The orientation assumes the plate lies flat: the box's axes are
    intersected with the horizontal plane through the centre, which is steadier than the fitted normal.
    """
    u, v = (float(c) for c in detection.center)
    rotation, origin = base_T_camera[:3, :3], base_T_camera[:3, 3]
    if detection.pose is not None:
        center = base_T_camera[:3, :3] @ detection.pose.position + origin
        source = detection.pose_source or 'depth'
    elif plate_size is not None:
        area = float(detection.size[0]) * float(detection.size[1])
        if area <= 1.0:
            return None
        z = float(np.sqrt(intrinsics.fx * intrinsics.fy * plate_size[0] * plate_size[1] / area))
        center = rotation @ intrinsics.backproject(u, v, z) + origin
        source = 'size'
    else:
        return None
    theta = float(detection.theta)
    half = 0.5 * np.asarray(detection.size, dtype=float)
    ends = []
    for du, dv in ((half[0] * np.cos(theta), half[0] * np.sin(theta)), (-half[1] * np.sin(theta), half[1] * np.cos(theta))):
        pair = [_horizontal_hit(origin, rotation @ intrinsics.ray(u + sign * du, v + sign * dv), center[2])
                for sign in (-1.0, 1.0)]
        if any(p is None for p in pair):
            return None
        ends.append(pair[1] - pair[0])
    lengths = [float(np.linalg.norm(e[:2])) for e in ends]
    if min(lengths) < 1e-4:
        return None
    # perspective: the box's two image axes land in the plane a little off perpendicular; average them (doubled angles)
    a0 = np.arctan2(ends[0][1], ends[0][0])
    a1 = np.arctan2(ends[1][1], ends[1][0]) - np.pi / 2
    width = 0.5 * np.arctan2(np.sin(2 * a0) + np.sin(2 * a1), np.cos(2 * a0) + np.cos(2 * a1))
    axis = np.array([np.cos(width), np.sin(width)])
    if lengths[1] < lengths[0]:
        axis = np.array([-axis[1], axis[0]])
    return PlateEstimate(stamp, center, axis, (max(lengths), min(lengths)), source, float(detection.conf))


class PlateFilter:
    """Running estimate of the (still) plate in the base frame, with an outlier gate.

    Positions are averaged with weight `alpha`; the axis direction is averaged as a doubled angle, so a
    flip of the box's axis by 180 deg doesn't cancel. An observation more than `gate` m from the estimate
    is dropped; `reset_after` such observations in a row replace the estimate (the plate did move).
    """

    def __init__(self, alpha=0.3, gate=0.03, reset_after=5):
        self.alpha, self.gate, self.reset_after = float(alpha), float(gate), int(reset_after)
        self.reset()

    def reset(self):
        self.center = None
        self._axis2 = None      # (cos 2a, sin 2a)
        self.count = 0
        self.last_stamp = None
        self.size = None
        self._outliers = []

    @property
    def short_axis(self):
        if self._axis2 is None:
            return None
        angle = 0.5 * np.arctan2(self._axis2[1], self._axis2[0])
        return np.array([np.cos(angle), np.sin(angle)])

    def update(self, estimate):
        """Returns True if the estimate was used."""
        angle = 2.0 * np.arctan2(estimate.short_axis[1], estimate.short_axis[0])
        doubled = np.array([np.cos(angle), np.sin(angle)])
        if self.center is not None and np.linalg.norm(estimate.center - self.center) > self.gate:
            self._outliers.append((estimate, doubled))
            if len(self._outliers) < self.reset_after:
                return False
            outliers = self._outliers
            self.reset()                  # the plate moved: restart from the observations that said so
            for est, dbl in outliers:
                self._blend(est, dbl)
            return True
        self._outliers = []
        self._blend(estimate, doubled)
        return True

    def _blend(self, estimate, doubled):
        if self.center is None:
            self.center, self._axis2, self.size = estimate.center.copy(), doubled, estimate.size
        else:
            a = self.alpha
            self.center = (1.0 - a) * self.center + a * estimate.center
            self._axis2 = (1.0 - a) * self._axis2 + a * doubled
            self.size = tuple((1.0 - a) * np.asarray(self.size) + a * np.asarray(estimate.size))
        self.count += 1
        self.last_stamp = estimate.stamp


def finger_angle_offset(finger_axis):
    """Angle of the finger axis in the base xy plane minus the flange yaw, with the tool pointing down
    (roll 180 deg, pitch 0): Rz(yaw) Rx(pi) maps (fx, fy, 0) to angle yaw + atan2(-fy, fx)."""
    fx, fy = {'x': (1.0, 0.0), 'y': (0.0, 1.0)}[finger_axis]
    return float(np.arctan2(-fy, fx))


def grasp_yaw(short_axis, current_yaw, finger_axis='y'):
    """Flange yaw that lines the fingers up with the plate's short side, the closest of the two to
    `current_yaw` (the fingers are symmetric)."""
    want = float(np.arctan2(short_axis[1], short_axis[0])) - finger_angle_offset(finger_axis)
    delta = wrap_pi(want - current_yaw)
    if delta > np.pi / 2:
        delta -= np.pi
    elif delta < -np.pi / 2:
        delta += np.pi
    return current_yaw + delta


@dataclass
class Gains:
    k_xy: float = 1.0             # 1/s
    k_z: float = 1.0              # 1/s
    k_yaw: float = 1.0            # 1/s
    max_xy: float = 0.02          # m/s, fingertip horizontal speed
    max_z: float = 0.02           # m/s
    max_yaw: float = 0.3          # rad/s
    tol_xy: float = 0.002         # m
    tol_z: float = 0.004          # m
    tol_yaw: float = np.radians(1.5)


@dataclass
class Command:
    v_tcp: np.ndarray             # (3,) m/s, base frame, for the controller's TCP point
    wz: float                     # rad/s about base z
    error_xyz: np.ndarray         # (3,) fingertip target - fingertip, base frame
    error_yaw: float
    within: bool                  # every error inside its tolerance


def servo_command(base_T_flange, flange_T_tip, flange_T_tcp, target_tip, target_yaw, gains):
    """Proportional fingertip command towards (target_tip, target_yaw), saturated per axis group.

    The xArm's Cartesian velocity mode moves its TCP point (flange_T_tcp, e.g. 140 mm down the flange);
    the fingertip's velocity is converted to it: v_tcp = v_tip - w x (p_tip - p_tcp).
    """
    tip = (base_T_flange @ flange_T_tip)[:3, 3]
    tcp = (base_T_flange @ flange_T_tcp)[:3, 3]
    yaw = float(np.arctan2(base_T_flange[1, 0], base_T_flange[0, 0]))
    error = np.asarray(target_tip, dtype=float) - tip
    error_yaw = float(wrap_pi(target_yaw - yaw))
    v = np.array([gains.k_xy * error[0], gains.k_xy * error[1], gains.k_z * error[2]])
    speed_xy = float(np.linalg.norm(v[:2]))
    if speed_xy > gains.max_xy:
        v[:2] *= gains.max_xy / speed_xy
    v[2] = float(np.clip(v[2], -gains.max_z, gains.max_z))
    wz = float(np.clip(gains.k_yaw * error_yaw, -gains.max_yaw, gains.max_yaw))
    r = tip - tcp
    v_tcp = v - np.cross([0.0, 0.0, wz], r)
    within = (np.linalg.norm(error[:2]) <= gains.tol_xy and abs(error[2]) <= gains.tol_z
              and abs(error_yaw) <= gains.tol_yaw)
    return Command(v_tcp, wz, error, error_yaw, bool(within))


def hover_target(plate_center, hover_height, offset=(0.0, 0.0)):
    """Fingertip target `hover_height` above the plate's top-face centre, shifted by `offset` (m, base xy)."""
    return np.array([plate_center[0] + offset[0], plate_center[1] + offset[1], plate_center[2] + hover_height])


def camera_point(base_T_camera, point_base):
    """A base-frame point in the camera frame."""
    return (inverse(base_T_camera) @ np.append(np.asarray(point_base, dtype=float), 1.0))[:3]
