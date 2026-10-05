#!/usr/bin/env python3
"""Check the wrist camera's mount rotation (and the fingertips) against recorded teleop episodes.

Rotation: while the arm translates, the tool's displacement (from tcp_pose, in the tool frame) and the
camera's displacement (visual odometry: LK features lifted with the depth, rigid fit with RANSAC, in the
camera frame) are the same vector in two frames. Kabsch over many such pairs gives the camera axes in the
tool frame, compared with the CAD model of mount.py. Features that don't move are dropped: the held plate and
the fingers ride along with the camera.

The difference is split in two. A turn of the whole mount about the flange axis (camera and gripper together)
changes nothing for the servo: plate and fingers are both measured through the camera, so it cancels. Only
the rest, the camera's rotation relative to the fingers, belongs in `mount.correction_rpy_deg`, and only if
the finger pads confirm it (--pads: the pads should lie along the image x axis).

Fingertips (--pads): the depth at the two finger pads (pixel positions, e.g. read off the annotated image) gives
their midpoint in the camera frame, a measured `mount.tip_camera_mm` to compare with the CAD tip.

  python3 tools/estimate_mount_rotation.py [episodes ...] [--pads 183,225 318,228]

Needs h5py and scipy (both in the repo venv). Episodes: teleop/ HDF5 recordings with wrist_rgb images and depth.
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from visual_servoing.mount import Mount, matrix_rpy, rot_z  # noqa: E402

DEFAULT_EPISODES = str(Path(__file__).resolve().parents[3] / 'data' / 'xarm6' / 'episodes' / '*.hdf5')


def kabsch(a, b):
    """Rotation R with b ~ R a (rows are vectors)."""
    u, _, vt = np.linalg.svd(a.T @ b)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    return vt.T @ np.diag([1.0, 1.0, d]) @ u.T


def rigid(p, q):
    """(R, t) with q ~ R p + t."""
    pc, qc = p.mean(0), q.mean(0)
    r = kabsch(p - pc, q - qc)
    return r, qc - r @ pc


def lift(points, depth_mm, k):
    """Pixels -> camera-frame points (m) with the depth under them (0 where there is none)."""
    fx, fy, cx, cy = k
    h, w = depth_mm.shape
    u, v = np.round(points[:, 0]).astype(int), np.round(points[:, 1]).astype(int)
    ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    z = np.zeros(len(points))
    z[ok] = depth_mm[v[ok], u[ok]] / 1000.0
    return np.column_stack([(points[:, 0] - cx) / fx * z, (points[:, 1] - cy) / fy * z, z])


def camera_displacement(gray0, gray1, depth0, depth1, k, rng, min_flow=2.5):
    """Origin of camera 1 in the camera-0 frame (m), or None if the features don't agree."""
    h = gray0.shape[0]
    mask = np.zeros_like(gray0)
    mask[: int(h * 0.62)] = 255                          # the mount and the fingers fill the bottom of the image
    p0 = cv2.goodFeaturesToTrack(gray0, 400, 0.01, 7, mask=mask)
    if p0 is None or len(p0) < 30:
        return None
    p1, found, _ = cv2.calcOpticalFlowPyrLK(gray0, gray1, p0, None, winSize=(21, 21), maxLevel=3)
    p0, p1 = p0[found[:, 0] == 1, 0], p1[found[:, 0] == 1, 0]
    moving = np.linalg.norm(p1 - p0, axis=1) > min_flow
    p, q = lift(p0[moving], depth0, k), lift(p1[moving], depth1, k)
    ok = (p[:, 2] > 0.1) & (q[:, 2] > 0.1) & (p[:, 2] < 1.5) & (q[:, 2] < 1.5)
    p, q = p[ok], q[ok]
    if len(p) < 20:
        return None
    best = None
    for _ in range(200):
        pick = rng.choice(len(p), 3, replace=False)
        r, t = rigid(p[pick], q[pick])
        inliers = np.linalg.norm(q - (p @ r.T + t), axis=1) < 0.006
        if best is None or inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 15:
        return None
    r, t = rigid(p[best], q[best])                       # camera-0 point -> camera-1 coordinates
    return -r.T @ t


def displacement_pairs(paths, min_move=0.012, max_turn_deg=1.5, step=10):
    """(tool displacements in the tool frame, camera displacements in the camera frame), one per usable pair."""
    import h5py
    from scipy.spatial.transform import Rotation
    rng = np.random.default_rng(0)
    tool, camera = [], []
    for path in paths:
        with h5py.File(path, 'r') as f:
            cal = json.loads(f.attrs['calibration'])['left']
            k = (cal['fx'], cal['fy'], cal['cx'], cal['cy'])
            images, depths = f['observations/images/wrist_rgb'], f['observations/depth/wrist_rgb']
            tcp = f['observations/tcp_pose'][:]              # xyz (m), rotation vector
            for i in range(0, len(tcp) - 15, step):
                far = np.nonzero(np.linalg.norm(tcp[i:, :3] - tcp[i, :3], axis=1) >= min_move)[0]
                if not len(far) or far[0] > 400:             # teleop is slow: pair frames by distance, not time
                    continue
                j = i + int(far[0])
                ri, rj = Rotation.from_rotvec(tcp[i, 3:]), Rotation.from_rotvec(tcp[j, 3:])
                if (ri.inv() * rj).magnitude() > np.radians(max_turn_deg):
                    continue
                gray = [cv2.cvtColor(images[n], cv2.COLOR_RGB2GRAY) for n in (i, j)]
                dc = camera_displacement(*gray, depths[i], depths[j], k, rng)
                if dc is None:
                    continue
                dt = ri.inv().apply(tcp[j, :3] - tcp[i, :3])
                if abs(np.linalg.norm(dc) - np.linalg.norm(dt)) > 0.35 * np.linalg.norm(dt):
                    continue                                 # depth scale off: a bad fit
                tool.append(dt)
                camera.append(dc)
        print(f'{Path(path).name}: {len(tool)} pairs so far', flush=True)
    return np.array(tool), np.array(camera)


def estimate_rotation(paths, mount):
    tool, camera = displacement_pairs(paths)
    if len(tool) < 20:
        sys.exit(f'only {len(tool)} usable displacement pairs: record episodes with more translation')
    rotation = kabsch(camera, tool)                          # tool = R camera: columns are the camera axes
    residual = np.linalg.norm(tool - camera @ rotation.T, axis=1)
    rng = np.random.default_rng(1)
    boot = [kabsch(camera[s], tool[s]) for s in (rng.integers(0, len(tool), len(tool)) for _ in range(300))]
    tilt = lambda r: np.degrees(np.arccos(np.clip(r[2, 2], -1.0, 1.0)))
    model = mount.flange_T_camera[:3, :3]
    np.set_printoptions(precision=3, suppress=True)
    print(f'\n{len(tool)} pairs, median move {np.median(np.linalg.norm(tool, axis=1)) * 1000:.1f} mm, '
          f'median residual {np.median(residual) * 1000:.1f} mm')
    print('camera axes (x, y, z columns) in the tool frame:\n', rotation)
    print(f'optical axis tilt {tilt(rotation):.1f} deg (90%: {np.percentile([tilt(b) for b in boot], 5):.1f}..'
          f'{np.percentile([tilt(b) for b in boot], 95):.1f}); model {mount.tilt_deg:.1f} deg')
    turns = np.radians(np.arange(-30.0, 30.0, 0.05))

    def split(r):
        """(turn of the whole mount about the flange z, deg; the rest as camera rpy, deg)"""
        angle = lambda a: np.arccos(np.clip((np.trace(model.T @ rot_z(-a) @ r) - 1.0) / 2.0, -1.0, 1.0))
        turn = min(turns, key=angle)
        return np.degrees(turn), np.degrees(matrix_rpy(model.T @ rot_z(-turn) @ r))
    turn, rest = split(rotation)
    parts = [split(b) for b in boot]
    print(f'estimate - model: the mount turned {turn:+.1f} deg about the flange axis '
          f'(90%: {np.percentile([p[0] for p in parts], 5):+.1f}..{np.percentile([p[0] for p in parts], 95):+.1f}; '
          'harmless if the gripper turns with it)')
    print('  and the camera rotated about its own x, y, z by', np.round(rest, 1).tolist(), 'deg (90%:',
          np.round(np.percentile([p[1] for p in parts], [5, 95], axis=0), 1).tolist(), ')')
    print('  the whole difference as camera rpy:', np.round(np.degrees(matrix_rpy(model.T @ rotation)), 1).tolist())


def measure_pads(paths, pads, mount, window=4):
    """Finger-pad midpoint (camera frame) from the depth of the first frame of each episode."""
    import h5py
    tips = []
    for path in paths:
        with h5py.File(path, 'r') as f:
            cal = json.loads(f.attrs['calibration'])['left']
            depth = f['observations/depth/wrist_rgb'][0].astype(float)
        points = []
        for u, v in pads:
            patch = depth[v - window:v + window + 1, u - window:u + window + 1]
            z = np.median(patch[patch > 0]) if (patch > 0).any() else np.nan
            points.append([(u - cal['cx']) / cal['fx'] * z, (v - cal['cy']) / cal['fy'] * z, z])
        left, right = np.array(points)
        tips.append((left + right) / 2)
        print(f'{Path(path).name}: pads {np.round(left, 1)} {np.round(right, 1)} mm, '
              f'{np.linalg.norm(left - right):.1f} mm apart')
    tip = np.nanmedian(tips, axis=0)
    (u0, v0), (u1, v1) = pads
    # the fingers close along flange y, which the model puts on the image x axis
    print(f'\nfinger axis in the image: {np.degrees(np.arctan2(v1 - v0, u1 - u0)):+.1f} deg from image x (model 0)')
    print(f'measured pad midpoint (camera, mm): {np.round(tip, 1).tolist()}  -> mount.tip_camera_mm')
    print(f'CAD fingertip       (camera, mm): {np.round(mount.tip_in_camera() * 1000, 1).tolist()}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('episodes', nargs='*', help=f'HDF5 episodes (default {DEFAULT_EPISODES})')
    parser.add_argument('--pads', nargs=2, metavar='U,V', help='finger pad pixels: measure the fingertips instead')
    args = parser.parse_args()
    paths = sorted(p for pattern in (args.episodes or [DEFAULT_EPISODES]) for p in glob.glob(pattern))
    if not paths:
        sys.exit('no episodes found')
    mount = Mount()
    if args.pads:
        measure_pads(paths, [tuple(int(c) for c in pad.split(',')) for pad in args.pads], mount)
    else:
        estimate_rotation(paths, mount)


if __name__ == '__main__':
    main()
