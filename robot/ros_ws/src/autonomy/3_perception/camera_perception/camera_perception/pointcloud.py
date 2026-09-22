"""Point clouds from the rectified depth image, as sensor_msgs/PointCloud2 (x, y, z float32 [+ rgb])."""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np


def depth_to_points(depth_mm, intrinsics, stride=4, bounds=None, edge_threshold=0.0, bgr=None):
    """(points (N, 3) float32 m, rgb (N,) uint32 0x00RRGGBB or None) of every `stride`-th pixel in x and y.

    bounds: ((x_lo, x_hi), (y_lo, y_hi), (z_lo, z_hi)) in the camera frame, None to keep everything valid.
    edge_threshold: drop points whose depth differs from a grid neighbour (stride px away) by more than
    this fraction of their depth per pixel of separation: the "flying pixels" smeared across depth edges.
    """
    stride = max(1, int(stride))
    z = depth_mm[::stride, ::stride].astype(np.float32) / 1000.0
    rows, cols = z.shape
    valid = z > 0.0
    if edge_threshold > 0.0:
        limit = z * (edge_threshold * stride)
        jump = np.zeros_like(z)
        for a, b, before, after in ((z[:-1], z[1:], jump[:-1], jump[1:]),
                                    (z[:, :-1], z[:, 1:], jump[:, :-1], jump[:, 1:])):
            # a missing neighbour (0 depth) is a hole, not a depth edge
            dz = np.where((a > 0.0) & (b > 0.0), np.abs(a - b), 0.0)
            np.maximum(before, dz, out=before)   # views: updates jump in place
            np.maximum(after, dz, out=after)
        valid &= jump <= limit
    us = np.arange(cols, dtype=np.float32) * stride
    vs = np.arange(rows, dtype=np.float32) * stride
    x = (us[None, :] - intrinsics.cx) / intrinsics.fx * z
    y = (vs[:, None] - intrinsics.cy) / intrinsics.fy * z
    if bounds is not None:
        (x_lo, x_hi), (y_lo, y_hi), (z_lo, z_hi) = bounds
        valid &= (x >= x_lo) & (x <= x_hi) & (y >= y_lo) & (y <= y_hi) & (z >= z_lo) & (z <= z_hi)
    points = np.column_stack([x[valid], y[valid], z[valid]]).astype(np.float32)
    rgb = None
    if bgr is not None:
        pixels = bgr[::stride, ::stride][valid].astype(np.uint32)
        rgb = (pixels[:, 2] << 16) | (pixels[:, 1] << 8) | pixels[:, 0]
    return points, rgb


def voxel_downsample(points, rgb=None, voxel=0.01):
    """One point per `voxel`-sized cube (the mean of its points; the colour of its first point)."""
    if voxel <= 0.0 or len(points) == 0:
        return points, rgb
    keys = np.floor(points / voxel).astype(np.int64)
    keys -= keys.min(axis=0)
    span = keys.max(axis=0) + 1
    flat = (keys[:, 0] * span[1] + keys[:, 1]) * span[2] + keys[:, 2]   # one int per voxel: a 1-D unique is fast
    _, first, inverse, counts = np.unique(flat, return_index=True, return_inverse=True, return_counts=True)
    inverse = inverse.ravel()
    sums = np.zeros((len(counts), 3), np.float64)
    np.add.at(sums, inverse, points)
    merged = (sums / counts[:, None]).astype(np.float32)
    return merged, (rgb[first] if rgb is not None else None)


def to_pointcloud2(header, points, rgb=None):
    """sensor_msgs/PointCloud2 (height 1, dense) with x, y, z float32 and, given colours, a packed rgb."""
    from sensor_msgs.msg import PointCloud2, PointField
    msg = PointCloud2()
    msg.header = header
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    columns = [np.asarray(points, np.float32).reshape(-1, 3)]
    if rgb is not None:
        fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))
        columns.append(np.asarray(rgb, np.uint32).reshape(-1, 1).view(np.float32))
    data = np.ascontiguousarray(np.hstack(columns), dtype=np.float32)
    msg.fields = fields
    msg.height, msg.width = 1, len(data)
    msg.is_bigendian = False
    msg.point_step = 4 * data.shape[1]
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


def pointcloud2_to_numpy(msg):
    """(points (N, 3), rgb (N,) uint32 or None) of a cloud written by to_pointcloud2 (tests, tools)."""
    columns = msg.point_step // 4
    data = np.frombuffer(bytes(msg.data), np.float32).reshape(-1, columns)
    rgb = data[:, 3].copy().view(np.uint32) if columns > 3 else None
    return data[:, :3].copy(), rgb


class Snapshot:
    """The pc3.py "snapshot" cloud: `frames` clouds stacked once, then reused (and cached on disk)."""

    def __init__(self, frames=10, voxel=0.0, cache_path=''):
        self.frames = max(1, int(frames))
        self.voxel = float(voxel)
        self.cache_path = os.path.expanduser(cache_path) if cache_path else ''
        self._buffer = []
        self.points = self.rgb = None
        self.save_error = None   # the OSError of the last cache write, if it failed

    @property
    def ready(self):
        return self.points is not None

    @property
    def progress(self):
        return len(self._buffer)

    def load(self):
        """Load the cache (N x 3 xyz like pc3_accumulated.npy, or N x 4 with packed rgb); True if loaded."""
        if not self.cache_path or not os.path.isfile(self.cache_path):
            return False
        data = np.load(self.cache_path)
        if data.ndim != 2 or data.shape[1] not in (3, 4):
            raise ValueError(f'{self.cache_path}: expected N x 3 or N x 4 points, got {data.shape}')
        data = data.astype(np.float32)
        self.points = np.ascontiguousarray(data[:, :3])
        self.rgb = data[:, 3].copy().view(np.uint32) if data.shape[1] == 4 else None
        return True

    def reset(self):
        self._buffer, self.points, self.rgb = [], None, None

    def add(self, points, rgb=None):
        """Buffer one frame's cloud; True once the snapshot is complete (and saved, with a cache path).

        Empty frames (e.g. while the depth is still all invalid) don't count. A failed cache write leaves the
        snapshot usable and is reported in `save_error`.
        """
        if self.ready:
            return True
        if not len(points):
            return False
        self._buffer.append((points, rgb))
        if len(self._buffer) < self.frames:
            return False
        points = np.vstack([p for p, _ in self._buffer])
        rgb = np.concatenate([c for _, c in self._buffer]) if all(c is not None for _, c in self._buffer) else None
        self._buffer = []
        self.points, self.rgb = voxel_downsample(points, rgb, self.voxel)
        self.save_error = None
        if self.cache_path:
            try:
                Path(self.cache_path).parent.mkdir(parents=True, exist_ok=True)
                data = self.points if self.rgb is None else np.column_stack([self.points, self.rgb.view(np.float32)])
                tmp = self.cache_path + '.tmp.npy'
                np.save(tmp, data)
                os.replace(tmp, self.cache_path)
            except OSError as exc:
                self.save_error = exc
        return True
