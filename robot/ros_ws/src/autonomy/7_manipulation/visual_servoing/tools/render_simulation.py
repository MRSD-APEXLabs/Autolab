#!/usr/bin/env python3
"""Render a simulated grasp run to a video: the tool and the plate in 3D, the wrist camera's view, the errors.

The run is ServoRunner with the deployed parameters (config/visual_servo.yaml, with execute) against the test
simulation (test/sim.py), made physical enough to watch:

- the arm integrates the velocity commands; position moves take distance / speed, gripper steps 0.5 s;
- the camera sees the plate at 15 Hz, 60 ms late, with ~1 mm of noise on its depth position;
- the fingers close at 0.19 mm of opening per gripper step and stop at the plate's edges; each FSR reads the
  squeeze of its pad (Camera-Edge's thresholds are uneven, so FSR1 reads harder); a grasped plate lifts with
  the fingers.

There is no arm model: the video shows the tool from the flange down. It needs no ROS and no hardware.

  python3 tools/render_simulation.py [-o sim.mp4] [--plate X Y ANGLE] [--start X Y Z YAW] [--seed N]
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'test'), str(ROOT.parent / 'camera_perception'), str(ROOT.parent / 'zedx_nano_depth')]
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
from scipy.spatial.transform import Rotation, Slerp  # noqa: E402
import yaml  # noqa: E402

from sim import INTRINSICS, PLATE, FakeArm, FakeClock, detect_plate  # noqa: E402
from visual_servoing.mount import inverse, rot_z, rpy_matrix, transform  # noqa: E402
from visual_servoing.params import build_config, build_mount  # noqa: E402
from visual_servoing.servo import ServoRunner  # noqa: E402

PLATE_HEIGHT = 0.01435                    # SBS plate
WELL_PITCH, WELL_RADIUS = 0.009, 0.0034
OPEN_WIDTH, WIDTH_PER_STEP = 0.110, 0.00019   # finger opening (m) at gripper 0, closing per gripper unit
FINGER_LENGTH, FINGER_WIDTH = 0.040, 0.015
FSR_PER_MM = (80.0, 30.0)                 # FSR reading per mm of pad squeeze
FPS = 25


def yaml_params():
    flat, stack = {}, [('', yaml.safe_load((ROOT / 'config' / 'visual_servo.yaml').read_text())['/**']['ros__parameters'])]
    while stack:
        prefix, values = stack.pop()
        for key, value in values.items():
            if isinstance(value, dict):
                stack.append((f'{prefix}{key}.', value))
            else:
                flat[prefix + key] = value
    return flat


class Scene:
    """The plate (top-face frame: x along the long side) and the fingers' contact with it."""

    def __init__(self, center, long_angle, mount):
        self.plate = transform(rot_z(long_angle), center)
        self.mount = mount
        self.held = None            # tip_T_plate while the fingers hold it

    def tip_frame(self, pose):
        return pose @ self.mount.flange_T_tip

    def plate_frame(self, pose):
        return self.plate if self.held is None else self.tip_frame(pose) @ self.held

    def finger_contact(self, pose, value):
        """(squeeze of finger 1, finger 2 in m, where the plate stops each finger along the finger axis or None)."""
        plate, tip = self.plate_frame(pose), self.tip_frame(pose)[:3, 3]
        finger = pose[:3, 1]                                    # the fingers close along flange y
        half = (OPEN_WIDTH - WIDTH_PER_STEP * value) / 2.0
        top = plate[2, 3]
        if not (tip[2] < top and tip[2] + FINGER_LENGTH > top - PLATE_HEIGHT):
            return (0.0, 0.0), (None, None)                     # the fingers pass above or below the plate
        d = float((plate[:3, 3] - tip) @ finger)
        extent = PLATE[0] / 2 * abs(finger @ plate[:3, 0]) + PLATE[1] / 2 * abs(finger @ plate[:3, 1])
        stops = (d + extent, extent - d)                        # plate edge beyond finger 1 (+y) / finger 2 (-y)
        return tuple(max(0.0, s - half) for s in stops), stops

    def fsr(self, value, arm):
        squeeze, _ = self.finger_contact(arm.pose, value)
        return tuple(int(min(1023, k * s * 1000)) for k, s in zip(FSR_PER_MM, squeeze))

    def finger_half_widths(self, pose, value):
        """Where the two fingers are (m from the tip along flange +y / -y), stopped by the plate."""
        half = (OPEN_WIDTH - WIDTH_PER_STEP * value) / 2.0
        _, stops = self.finger_contact(pose, value)
        return tuple(half if s is None else max(half, s) for s in stops)


class RecordingArm(FakeArm):
    """FakeArm whose moves and gripper steps take their time, logging keyframes for the video."""

    def __init__(self, clock, pose, scene):
        super().__init__(clock, pose, fsr=scene.fsr)
        self.scene, self.runner = scene, None
        self.log = []               # (t, pose, gripper value, plate frame, status)

    def _log(self):
        value = self.gripper_values[-1] if self.gripper_values else 0
        status = self.runner.status() if self.runner is not None else {}
        self.log.append((self.clock.time(), self.pose.copy(), value, self.scene.plate_frame(self.pose), status))

    def flange_pose(self):
        pose = super().flange_pose()
        self._log()
        return pose

    def move_flange(self, base_T_flange, speed_mm_s, acc_mm_s2=500.0):
        start = self.pose.copy()
        self._log()
        squeeze, _ = self.scene.finger_contact(start, self.gripper_values[-1] if self.gripper_values else 0)
        if self.scene.held is None and min(squeeze) > 0.0:     # both fingers press: the plate moves with them
            self.scene.held = inverse(self.scene.tip_frame(start)) @ self.scene.plate
        super().move_flange(base_T_flange, speed_mm_s, acc_mm_s2)
        distance = np.linalg.norm(base_T_flange[:3, 3] - start[:3, 3]) * 1000.0
        turn = np.degrees(Rotation.from_matrix(start[:3, :3].T @ base_T_flange[:3, :3]).magnitude())
        self.clock.sleep(max(distance / speed_mm_s, turn / 30.0) + 0.1)
        self._log()

    def gripper_set(self, value, settle=0.5):
        self._log()
        ok = super().gripper_set(value, settle)
        if value == 0 and self.scene.held is not None:          # opened: the plate stays where it is
            self.scene.plate, self.scene.held = self.scene.plate_frame(self.pose), None
        self.clock.sleep(max(settle, 0.5))
        self._log()
        return ok


class NoisyCamera:
    """The wrist camera: plate detections at `rate` Hz, `latency` s late, with noise on the depth position."""

    def __init__(self, runner, scene, rng, rate=15.0, latency=0.06, noise=(0.001, 0.001, 0.0015)):
        self.runner, self.scene, self.rng = runner, scene, rng
        self.period, self.latency, self.noise = 1.0 / rate, latency, np.asarray(noise)
        self.poses, self.next = [], None

    def __call__(self, t, pose):
        self.poses.append((t, pose))
        self.next = t + self.latency if self.next is None else self.next
        if t < self.next:
            return
        self.next = t + self.period
        stamp, captured = min(self.poses[-40:], key=lambda item: abs(item[0] - (t - self.latency)))
        plate = self.scene.plate_frame(captured)
        det = detect_plate(captured @ self.scene.mount.flange_T_camera, plate[:3, 3],
                           np.arctan2(plate[1, 0], plate[0, 0]))
        if det is not None:
            det.center = det.center + self.rng.normal(0.0, 0.4, 2)
            det.theta += self.rng.normal(0.0, np.radians(0.3))
            det.pose.position = det.pose.position + self.rng.normal(0.0, self.noise)
        self.runner.observe(stamp, [det] if det is not None else [], INTRINSICS)


def simulate(args):
    params = yaml_params()
    params['execute'] = True
    config, mount = build_config(lambda name, default: params.get(name, default)), \
        build_mount(lambda name, default: params.get(name, default))
    clock = FakeClock(0.0)
    scene = Scene(np.array([args.plate[0], args.plate[1], 0.02]), np.radians(args.plate[2]), mount)
    x, y, z, yaw = args.start
    start = transform(rpy_matrix(np.radians(174.0), np.radians(4.0), np.radians(yaw)), [x, y, z])   # not level
    arm = RecordingArm(clock, start, scene)
    runner = ServoRunner(arm, mount, config, clock=clock.time, sleep=clock.sleep)
    arm.runner = runner
    arm.on_pose = NoisyCamera(runner, scene, np.random.default_rng(args.seed))
    runner.start()
    runner._thread.join(120.0)
    arm._log()
    return arm.log, runner.status(), mount, config


class Timeline:
    """The run's keyframes, interpolated at any time."""

    def __init__(self, log):
        keep = {}
        for entry in log:                                       # the last keyframe of each instant
            keep[entry[0]] = entry
        self.entries = [keep[t] for t in sorted(keep)]
        self.times = np.array([e[0] for e in self.entries])
        self.end = float(self.times[-1])

    def _frames(self, i, a, index):
        p0, p1 = self.entries[i][index], self.entries[i + 1][index]
        out = p0.copy()
        out[:3, 3] = (1 - a) * p0[:3, 3] + a * p1[:3, 3]
        rotations = Rotation.from_matrix(np.stack([p0[:3, :3], p1[:3, :3]]))
        out[:3, :3] = Slerp([0.0, 1.0], rotations)(a).as_matrix()
        return out

    def at(self, t):
        i = int(np.clip(np.searchsorted(self.times, t, side='right') - 1, 0, len(self.times) - 2))
        t0, t1 = self.times[i], self.times[i + 1]
        a = float(np.clip((t - t0) / max(t1 - t0, 1e-9), 0.0, 1.0))
        value = (1 - a) * self.entries[i][2] + a * self.entries[i + 1][2]
        status = self.entries[i + 1][4] if a >= 1.0 else self.entries[i][4]
        return self._frames(i, a, 1), value, self._frames(i, a, 3), status


def box_faces(frame, x, y, z):
    """Faces of the box spanning [x0, x1] x [y0, y1] x [z0, z1] in `frame` (4x4), for Poly3DCollection."""
    c = np.array([[x[i], y[j], z[k], 1.0] for i in (0, 1) for j in (0, 1) for k in (0, 1)])
    p = (frame @ c.T).T[:, :3]
    faces = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [p[list(f)] for f in faces]


def wells(plate):
    ix, iy = np.meshgrid((np.arange(12) - 5.5) * WELL_PITCH, (np.arange(8) - 3.5) * WELL_PITCH)
    local = np.column_stack([ix.ravel(), iy.ravel(), np.zeros(96), np.ones(96)])
    return (plate @ local.T).T[:, :3]


def gripper_parts(pose, scene, value):
    """(finger boxes, body box) as face lists, in the base frame."""
    tip = scene.tip_frame(pose)
    h1, h2 = scene.finger_half_widths(pose, value)
    fw, fl = FINGER_WIDTH / 2, FINGER_LENGTH
    fingers = box_faces(tip, (-fw, fw), (h1, h1 + 0.008), (-fl, 0.0)) + \
        box_faces(tip, (-fw, fw), (-h2 - 0.008, -h2), (-fl, 0.0))
    body = box_faces(tip, (-0.022, 0.022), (-0.066, 0.066), (-fl - 0.03, -fl))
    return fingers, body


def draw_3d(ax, t, pose, value, plate, status, scene, mount, trail, table_z):
    ax.cla()
    ax.set_xlim(0.30, 0.56)
    ax.set_ylim(-0.09, 0.17)
    ax.set_zlim(0.0, 0.40)
    ax.set_box_aspect((0.26, 0.26, 0.40), zoom=1.2)
    ax.view_init(elev=18, azim=-58 + 10 * np.sin(t / 12))
    ax.set_xlabel('x (m)', labelpad=-8)
    ax.set_ylabel('y (m)', labelpad=-8)
    ax.set_zlabel('z (m)', labelpad=-8)
    ax.tick_params(labelsize=7, pad=-3)
    table = np.array([[0.30, -0.09], [0.56, -0.09], [0.56, 0.17], [0.30, 0.17]])
    ax.add_collection3d(Poly3DCollection([np.column_stack([table, np.full(4, table_z)])], facecolor='#d8d2c4',
                                         edgecolor='none', alpha=0.6))
    ax.add_collection3d(Poly3DCollection(box_faces(plate, (-PLATE[0] / 2, PLATE[0] / 2), (-PLATE[1] / 2, PLATE[1] / 2),
                                                   (-PLATE_HEIGHT, 0.0)),
                                         facecolor='#9fd3f0', edgecolor='#3b7ea1', linewidth=0.5, alpha=0.55))
    w = wells(plate)
    ax.scatter(w[:, 0], w[:, 1], w[:, 2] + 0.0005, s=1.5, c='#3b7ea1', depthshade=False)
    fingers, body = gripper_parts(pose, scene, value)
    ax.add_collection3d(Poly3DCollection(body, facecolor='#8c8c8c', edgecolor='#444444', linewidth=0.4, alpha=0.9))
    ax.add_collection3d(Poly3DCollection(fingers, facecolor='#303030', edgecolor='#000000', linewidth=0.4))
    tip = scene.tip_frame(pose)
    body_top = (tip @ [0, 0, -FINGER_LENGTH - 0.03, 1])[:3]
    flange = pose[:3, 3]
    ax.plot(*np.column_stack([body_top, flange]), color='#666666', linewidth=6, solid_capstyle='round')
    ax.plot(*np.column_stack([flange, flange + [0, 0, 0.07]]), color='#bbbbbb', linewidth=10, solid_capstyle='round')
    # the camera and its view
    cam = pose @ mount.flange_T_camera
    origin = cam[:3, 3]
    ax.plot(*np.column_stack([flange, origin]), color='#666666', linewidth=3)
    corners = [(0, 0), (INTRINSICS.width, 0), (INTRINSICS.width, INTRINSICS.height), (0, INTRINSICS.height)]
    rays = [cam[:3, :3] @ INTRINSICS.ray(u, v) for u, v in corners]
    far = [origin + r * 0.035 / np.linalg.norm(r) for r in rays]
    for p in far:
        ax.plot(*np.column_stack([origin, p]), color='#e08a00', linewidth=1)
    ax.plot(*np.column_stack(far + far[:1]), color='#e08a00', linewidth=1)
    axis = cam[:3, 2]
    if axis[2] < -1e-3:
        hit = origin + axis * (table_z - origin[2]) / axis[2]
        ax.plot(*np.column_stack([origin, hit]), color='#e08a00', linewidth=0.8, linestyle=':')
    # fingertip trail, plate estimate and servo target
    if len(trail) > 1:
        ax.plot(*np.array(trail).T, color='#d62728', linewidth=1.2)
    if status.get('running') and status.get('phase') == 'servoing' and 'target_mm' in status:
        target, estimate = np.array(status['target_mm']) / 1000, np.array(status['plate_mm']) / 1000
        ax.scatter(*target, marker='x', s=40, c='#c000c0', depthshade=False)
        ax.plot(*np.column_stack([estimate, target]), color='#c000c0', linewidth=0.8, linestyle='--')
        ax.scatter(*estimate, marker='o', s=12, c='#c000c0', depthshade=False)
    ax.scatter(*tip[:3, 3], s=14, c='#d62728', depthshade=False)


def project(points, camera_T_base):
    p = (camera_T_base @ np.column_stack([points, np.ones(len(points))]).T).T[:, :3]
    if (p[:, 2] <= 0.01).any():
        return None
    return np.column_stack([INTRINSICS.fx * p[:, 0] / p[:, 2] + INTRINSICS.cx,
                            INTRINSICS.fy * p[:, 1] / p[:, 2] + INTRINSICS.cy]), p[:, 2]


def camera_view(pose, value, plate, status, scene, mount, table_z):
    """The wrist image as the node would annotate it (bgr, 480x300)."""
    scale = 2                                                # drawn at 2x, for smooth edges
    img = np.full((INTRINSICS.height * scale, INTRINSICS.width * scale, 3), (150, 160, 168), np.uint8)
    camera_T_base = inverse(pose @ mount.flange_T_camera)

    def poly(points, color, thickness=-1):
        out = project(points, camera_T_base)
        if out is not None:
            pts = np.round(out[0] * scale * 8).astype(np.int32)
            if thickness < 0:
                cv2.fillPoly(img, [pts], color, cv2.LINE_AA, shift=3)
            else:
                cv2.polylines(img, [pts], True, color, thickness, cv2.LINE_AA, shift=3)
    for g in np.arange(0.20, 0.70, 0.05):                     # table grid, 5 cm
        for line in (np.array([[g, -0.3, table_z], [g, 0.4, table_z]]), np.array([[0.1, g - 0.35, table_z],
                                                                                  [0.8, g - 0.35, table_z]])):
            pts = np.linspace(line[0], line[1], 40)
            out = project(pts, camera_T_base)
            if out is not None:
                cv2.polylines(img, [np.round(out[0] * scale * 8).astype(np.int32)], False, (135, 145, 152), 1,
                              cv2.LINE_AA, shift=3)
    sides = box_faces(plate, (-PLATE[0] / 2, PLATE[0] / 2), (-PLATE[1] / 2, PLATE[1] / 2), (-PLATE_HEIGHT, 0.0))
    for face in sides[2:]:
        poly(face, (205, 190, 170))
    poly(sides[1], (235, 225, 205))
    w = project(wells(plate), camera_T_base)
    if w is not None:
        for (u, v), z in zip(*w):
            cv2.circle(img, (int(u * scale * 8), int(v * scale * 8)), max(8, int(INTRINSICS.fx * WELL_RADIUS / z * scale * 8)),
                       (190, 170, 150), 1, cv2.LINE_AA, shift=3)
    fingers, body = gripper_parts(pose, scene, value)
    for face in body:
        poly(face, (110, 110, 110))
    for face in fingers:
        poly(face, (45, 45, 45))
    img = cv2.resize(img, (INTRINSICS.width, INTRINSICS.height), interpolation=cv2.INTER_AREA)
    running, phase = status.get('running'), status.get('phase', '')
    if running and phase == 'servoing':
        det = detect_plate(pose @ mount.flange_T_camera, plate[:3, 3], np.arctan2(plate[1, 0], plate[0, 0]))
        if det is not None:
            box = cv2.boxPoints(((float(det.center[0]), float(det.center[1])), (float(det.size[0]), float(det.size[1])),
                                 float(np.degrees(det.theta))))
            cv2.polylines(img, [np.round(box).astype(np.int32)], True, (0, 200, 0), 2, cv2.LINE_AA)
            cv2.circle(img, (int(det.center[0]), int(det.center[1])), 3, (0, 200, 0), -1, cv2.LINE_AA)
        if 'plate_mm' in status:
            est = project(np.array([status['plate_mm']]) / 1000.0, camera_T_base)
            if est is not None:
                u, v = est[0][0]
                cv2.drawMarker(img, (int(u), int(v)), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2, cv2.LINE_AA)
    tip = mount.project(mount.tip_in_camera(), INTRINSICS)
    cv2.drawMarker(img, (int(tip[0]), int(tip[1])), (0, 230, 255), cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)
    lines = [phase or 'idle']
    if running and phase == 'servoing' and 'error_mm' in status:
        e = status['error_mm']
        lines.append(f'err {e[0]:+.0f} {e[1]:+.0f} {e[2]:+.0f} mm {status.get("error_yaw_deg", 0):+.1f} deg '
                     f'settled {status.get("settled", 0)}')
    if status.get('result'):
        lines.append(f'{status["result"]}: {status.get("message", "")}'[:60])
    for i, line in enumerate(lines):
        cv2.putText(img, line, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, line, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def render(log, final, mount, config, out_paths, seed_note):
    timeline = Timeline(log)
    scene = Scene(np.zeros(3), 0.0, mount)                      # geometry helpers only; frames come from the log
    table_z = timeline.entries[0][3][2, 3] - PLATE_HEIGHT
    # time series for the plots
    series = []
    for t, pose, value, plate, status in timeline.entries:
        e = status.get('error_mm') if status.get('phase') == 'servoing' else None
        fsr = status.get('fsr') if status.get('phase') in ('closing', 'lifting', 'returning') else None
        series.append((t, None if e is None else (np.hypot(e[0], e[1]), abs(e[2]), abs(status.get('error_yaw_deg', 0.0))),
                       None if fsr is None or fsr[0] is None else fsr, value))
    phases = []
    for t, _, _, _, status in timeline.entries:
        phase = status.get('phase')
        if phase and (not phases or phases[-1][1] != phase):
            phases.append((t, phase))

    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    grid = fig.add_gridspec(3, 2, width_ratios=[1.25, 1.0], height_ratios=[1.35, 0.8, 0.8], left=0.01, right=0.97,
                            top=0.90, bottom=0.07, wspace=0.08, hspace=0.45)
    ax3d = fig.add_subplot(grid[:, 0], projection='3d')
    axcam, axerr, axfsr = fig.add_subplot(grid[0, 1]), fig.add_subplot(grid[1, 1]), fig.add_subplot(grid[2, 1])
    title = fig.suptitle('', fontsize=12, y=0.985)
    fig.text(0.5, 0.935, f'config/visual_servo.yaml with execute; tool only (no arm model); {seed_note}', ha='center',
             fontsize=7, color='#666666')
    writers = []
    for path in out_paths:
        fourcc = 'avc1' if path.suffix == '.mp4' else 'VP90'
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*fourcc), FPS, (1280, 720))
        if not writer.isOpened():
            raise SystemExit(f'cannot write {path} ({fourcc})')
        writers.append(writer)
    trail, times = [], np.arange(0.0, timeline.end + 1e-9, 1.0 / FPS)
    times = np.concatenate([times, np.full(int(2.0 * FPS), timeline.end)])      # hold the end for 2 s
    for n, t in enumerate(times):
        pose, value, plate, status = timeline.at(t)
        if n == 0 or t > times[n - 1]:
            trail.append(scene.tip_frame(pose)[:3, 3])
        draw_3d(ax3d, t, pose, value, plate, status, scene, mount, trail, table_z)
        axcam.cla()
        axcam.imshow(cv2.cvtColor(camera_view(pose, value, plate, status, scene, mount, table_z), cv2.COLOR_BGR2RGB))
        axcam.set_title('wrist camera (simulated ZED X Nano, as annotated by the node)', fontsize=9)
        axcam.axis('off')
        past = [s for s in series if s[0] <= t]
        axerr.cla()
        pts = [(s[0], *s[1]) for s in past if s[1] is not None]
        if pts:
            a = np.array(pts)
            a[:, 1:] = np.maximum(a[:, 1:], 0.2)
            axerr.plot(a[:, 0], a[:, 1], label='xy error (mm)', color='#1f77b4')
            axerr.plot(a[:, 0], a[:, 2], label='|z error| (mm)', color='#2ca02c')
            axerr.plot(a[:, 0], a[:, 3], label='|yaw error| (deg)', color='#9467bd')
        axerr.set_xlim(0, timeline.end)
        axerr.set_yscale('log')
        axerr.set_ylim(0.2, 300)
        axerr.set_title('servo: fingertip error to the hover pose', fontsize=9)
        if pts:
            axerr.legend(fontsize=7, loc='upper right')
        axerr.tick_params(labelsize=7)
        axfsr.cla()
        pts = [(s[0], *s[2]) for s in past if s[2] is not None]
        if pts:
            a = np.array(pts, dtype=float)
            axfsr.step(a[:, 0], a[:, 1], where='post', label='FSR1', color='#d62728')
            axfsr.step(a[:, 0], a[:, 2], where='post', label='FSR2', color='#ff7f0e')
        grip = [(s[0], s[3]) for s in past if s[3] is not None and s[2] is not None]
        if grip:
            a = np.array(grip, dtype=float)
            axfsr.step(a[:, 0], a[:, 1], where='post', label='gripper command', color='#7f7f7f', linestyle='--')
        for level, color in zip(config.gripper.stop_pressure, ('#d62728', '#ff7f0e')):
            axfsr.axhline(level, color=color, linewidth=0.6, linestyle=':')
        axfsr.set_xlim(0, timeline.end)
        axfsr.set_ylim(0, 700)
        axfsr.set_xlabel('time (s)', fontsize=8)
        axfsr.set_title('grasp: FSRs (dotted: stop pressure) and gripper command', fontsize=9)
        if pts:
            axfsr.legend(fontsize=7, loc='upper left')
        axfsr.tick_params(labelsize=7)
        for ax in (axerr, axfsr):
            for tp, phase in phases:
                if tp <= t and phase not in ('idle', 'preparing'):
                    ax.axvline(tp, color='#cccccc', linewidth=0.6, zorder=0)
        phase = status.get('phase') or 'idle'
        result = status.get('result')
        title.set_text(f'Visual servo grasp, simulated   t = {t:5.1f} s   phase: {phase}'
                       + (f'   result: {result}' if result and not status.get('running') else ''))
        fig.canvas.draw()
        frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        for writer in writers:
            writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        if n % 50 == 0:
            print(f'frame {n}/{len(times)}', flush=True)
    for writer in writers:
        writer.release()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('-o', '--output', default='visual_servo_sim.mp4', help='.mp4 (H.264) or .webm (VP9)')
    parser.add_argument('--webm', action='store_true', help='also write a .webm next to it (plays in VS Code)')
    parser.add_argument('--plate', nargs=3, type=float, default=(0.44, 0.07, 30.0), metavar=('X', 'Y', 'ANGLE'),
                        help='plate top-face centre (m) and long-side angle (deg); the top is at z = 0.02')
    parser.add_argument('--start', nargs=4, type=float, default=(0.40, 0.0, 0.35, 10.0), metavar=('X', 'Y', 'Z', 'YAW'),
                        help='flange start (m) and yaw (deg); roll/pitch start 6/4 deg off level')
    parser.add_argument('--seed', type=int, default=0, help='detection noise seed')
    args = parser.parse_args()
    log, final, mount, config = simulate(args)
    print(f'simulated {log[-1][0]:.1f} s: {final.get("result")} ({final.get("message")})')
    out = Path(args.output)
    paths = [out] + ([out.with_suffix('.webm')] if args.webm else [])
    render(log, final, mount, config, paths, f'plate at ({args.plate[0]:.2f}, {args.plate[1]:.2f}) m, '
           f'{args.plate[2]:.0f} deg; noise seed {args.seed}')
    for path in paths:
        print(f'wrote {path} ({path.stat().st_size / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
