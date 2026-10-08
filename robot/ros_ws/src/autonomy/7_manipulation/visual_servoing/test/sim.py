"""A simulated xArm, clock and wrist camera for the servo tests: the arm follows velocity and position commands
in simulated time, and the camera sees a plate lying on the table as the YOLO detector would report it."""
from __future__ import annotations

from collections import deque
import threading

import numpy as np

from camera_perception.geometry import Intrinsics, Pose
from camera_perception.yolo import Detection
from visual_servoing.arm import ArmError, MODE_POSITION, MODE_VELOCITY
from visual_servoing.mount import inverse, rot_z, rpy_matrix, transform, xarm_pose_matrix

INTRINSICS = Intrinsics(243.3, 243.3, 241.8, 152.9, 480, 300)   # the recorded 480x300 wrist stream
PLATE = (0.12776, 0.08548)


class FakeClock:
    """Simulated time: sleep() advances it at once, so a run of minutes takes milliseconds."""

    def __init__(self, t=1000.0):
        self.t = float(t)
        self._lock = threading.Lock()

    def time(self):
        with self._lock:
            return self.t

    def sleep(self, seconds):
        with self._lock:
            self.t += max(0.0, float(seconds))


def tool_down(x, y, z, yaw_deg=0.0):
    """base_T_flange with the tool pointing down (roll 180, pitch 0)."""
    return transform(rpy_matrix(np.pi, 0.0, np.radians(yaw_deg)), np.array([x, y, z]))


class FakeArm:
    """XArm stand-in. Velocity commands are integrated about the TCP point up to their deadman `duration`;
    set_position moves are instant. `fsr(value, arm)` gives the finger readings for a gripper value.
    `on_pose(t, base_T_flange)` runs on every pose read (the camera and test hooks)."""

    def __init__(self, clock, pose, read_only=False, tcp_offset=(0, 0, 140, 0, 0, 0), fsr=None):
        self.clock, self.read_only = clock, read_only
        self.flange_T_tcp = np.eye(4)
        self._offset = xarm_pose_matrix(tcp_offset)
        self.pose = np.asarray(pose, dtype=float).copy()
        self.mode = None
        self.connected = False
        self.fsr = fsr or (lambda value, arm: (0, 0))
        self.on_pose = None
        self.modes, self.moves, self.gripper_values, self.velocities = [], [], [], []
        self.connects = 0
        self._command = (np.zeros(3), 0.0, 0.0)   # v_tcp, wz, expiry
        self._last = clock.time()

    def _integrate(self):
        now = self.clock.time()
        v, wz, expiry = self._command
        dt = max(0.0, min(now, expiry) - self._last)
        self._last = now
        if dt > 0.0 and self.mode == MODE_VELOCITY:
            tcp = self.pose @ self._offset
            tcp[:3, :3] = rot_z(wz * dt) @ tcp[:3, :3]
            tcp[:3, 3] += v * dt
            self.pose = tcp @ inverse(self._offset)

    def _motion(self):
        if self.read_only:
            raise ArmError('dry run: the arm is read-only')
        if not self.connected:
            raise ArmError('xArm not connected')

    def connect(self):
        self.connected = True
        self.connects += 1
        self.flange_T_tcp = self._offset.copy()

    def disconnect(self):
        self.connected = False

    def flange_pose(self):
        if not self.connected:
            raise ArmError('xArm not connected')
        self._integrate()
        pose = self.pose.copy()
        if self.on_pose is not None:
            self.on_pose(self.clock.time(), pose)
        return pose

    def _set_mode(self, mode):
        self._motion()
        self._integrate()
        self._command = (np.zeros(3), 0.0, 0.0)
        self.mode = mode
        self.modes.append(mode)

    def position_mode(self):
        self._set_mode(MODE_POSITION)

    def velocity_mode(self):
        self._set_mode(MODE_VELOCITY)

    def hand_back(self):
        if self.read_only or not self.connected:
            return
        self._set_mode(1)

    def velocity(self, v_tcp, wz, duration):
        self._motion()
        if self.mode != MODE_VELOCITY:
            raise ArmError('velocity command outside velocity mode')
        self._integrate()
        self._command = (np.asarray(v_tcp, dtype=float).copy(), float(wz), self.clock.time() + float(duration))
        self.velocities.append((np.asarray(v_tcp, dtype=float).copy(), float(wz)))

    def stop(self):
        if self.read_only or not self.connected or self.mode != MODE_VELOCITY:
            return
        self._integrate()
        self._command = (np.zeros(3), 0.0, 0.0)

    def move_flange(self, base_T_flange, speed_mm_s, acc_mm_s2=500.0):
        self._motion()
        if self.mode != MODE_POSITION:
            raise ArmError('position command outside position mode')
        self.pose = np.asarray(base_T_flange, dtype=float).copy()
        self.moves.append((self.pose.copy(), float(speed_mm_s)))

    def gripper_setup(self):
        self._motion()

    def gripper_set(self, value, settle=0.5):
        self._motion()
        self.gripper_values.append(int(np.clip(int(value), 0, 1000)))
        return True

    def gripper_open(self):
        return self.gripper_set(0, settle=0.0)

    def gripper_position(self):
        return self.gripper_values[-1] if self.gripper_values else 0

    def gripper_fsr(self):
        self._motion()
        return self.fsr(self.gripper_values[-1] if self.gripper_values else 0, self)


def plate_corners(center, long_angle, size=PLATE):
    """(4, 3) top-face corners of a plate lying flat, centre `center`, long side at `long_angle` (rad)."""
    a = np.array([np.cos(long_angle), np.sin(long_angle), 0.0]) * size[0] / 2
    b = np.array([-np.sin(long_angle), np.cos(long_angle), 0.0]) * size[1] / 2
    c = np.asarray(center, dtype=float)
    return np.array([c + a + b, c - a + b, c - a - b, c + a - b])


def detect_plate(base_T_camera, center, long_angle, size=PLATE, intrinsics=INTRINSICS, with_depth=True, conf=0.9,
                 width='long'):
    """The Detection YOLO + planar_pose would give for the plate, or None if its centre is out of view.
    The box's width runs along the projected long side (`width='long'`) or short side."""
    camera_T_base = inverse(base_T_camera)

    def project(point):
        p = (camera_T_base @ np.append(point, 1.0))[:3]
        return None if p[2] <= 1e-6 else np.array([intrinsics.fx * p[0] / p[2] + intrinsics.cx,
                                                    intrinsics.fy * p[1] / p[2] + intrinsics.cy])

    center = np.asarray(center, dtype=float)
    along = np.array([np.cos(long_angle), np.sin(long_angle), 0.0])
    across = np.array([-along[1], along[0], 0.0])
    if width != 'long':
        along, across, size = across, -along, (size[1], size[0])
    uv = project(center)
    ends = [project(center + s * along * size[0] / 2) for s in (-1, 1)] + \
           [project(center + s * across * size[1] / 2) for s in (-1, 1)]
    corners = [project(p) for p in plate_corners(center, long_angle)]
    if uv is None or any(p is None for p in ends + corners):
        return None
    if not (0 <= uv[0] < intrinsics.width and 0 <= uv[1] < intrinsics.height):
        return None
    w_vec, h_vec = ends[1] - ends[0], ends[3] - ends[2]
    theta = float(np.arctan2(w_vec[1], w_vec[0]))
    det = Detection(0, 'wellplate', conf, uv, np.array([np.linalg.norm(w_vec), np.linalg.norm(h_vec)]), theta,
                    np.array(corners))
    if with_depth:
        rotation = camera_T_base[:3, :3] @ np.column_stack([along, np.cross([0, 0, 1.0], along), [0, 0, 1.0]])
        det.pose = Pose(rotation, (camera_T_base @ np.append(center, 1.0))[:3])
        det.metric_size, det.pose_source = tuple(size), 'plane'
    return det


class Camera:
    """Feeds `runner.observe` with detections of the plate at `rate` Hz, `latency` s after capture, from the
    arm pose at capture time. Install with arm.on_pose = camera."""

    def __init__(self, runner, mount, center, long_angle, rate=15.0, latency=0.06, with_depth=True, visible=True,
                 size=PLATE, conf=0.9):
        self.runner, self.mount, self.size, self.conf = runner, mount, size, conf
        self.center, self.long_angle = np.asarray(center, dtype=float), float(long_angle)
        self.period, self.latency, self.with_depth, self.visible = 1.0 / rate, latency, with_depth, visible
        self._poses = deque(maxlen=200)
        self._next = None
        self.frames = 0

    def __call__(self, t, pose):
        self._poses.append((t, pose))
        if self._next is None:
            self._next = t + self.latency
        if t < self._next or not self.visible:
            return
        self._next = t + self.period
        stamp = t - self.latency
        captured = min(self._poses, key=lambda item: abs(item[0] - stamp))
        base_T_camera = captured[1] @ self.mount.flange_T_camera
        det = detect_plate(base_T_camera, self.center, self.long_angle, self.size, with_depth=self.with_depth,
                           conf=self.conf)
        self.frames += 1
        self.runner.observe(captured[0], [det] if det is not None else [], INTRINSICS)
