"""Pixel and depth geometry shared by the detectors.

Conventions: images are rectified (no distortion), depth images are uint16 millimetres with 0 = invalid,
3D points are metres in the camera's optical frame (x right, y down, z forward). Rotations are 3x3
matrices whose columns are the object's axes in the camera frame; quaternions are [x, y, z, w].
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole model of a rectified image."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_camera_info(cls, msg):
        k = msg.k
        return cls(float(k[0]), float(k[4]), float(k[2]), float(k[5]), int(msg.width), int(msg.height))

    @property
    def matrix(self):
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def ray(self, u, v):
        """Direction through pixel (u, v), scaled to z = 1."""
        return np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])

    def backproject(self, u, v, z):
        return self.ray(u, v) * z


@dataclass
class Pose:
    rotation: np.ndarray      # 3x3
    position: np.ndarray      # (3,) metres

    def as_dict(self):
        """{"position": [x, y, z], "orientation": [x, y, z, w]}: the Camera-Edge payload format."""
        return {'position': [float(v) for v in self.position],
                'orientation': [float(v) for v in rotation_to_quaternion(self.rotation)]}


def rotation_to_quaternion(r):
    """[x, y, z, w] of a rotation matrix (w >= 0)."""
    r = np.asarray(r, dtype=np.float64)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        q = [0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        q = [(r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        q = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s]
    q = np.asarray(q, dtype=np.float64)
    q /= max(np.linalg.norm(q), 1e-12)
    return q if q[3] >= 0.0 else -q


def depth_at(depth_mm, u, v, half_window=2):
    """Median valid depth (m) in a (2*half_window+1)^2 window around (u, v), or None (also outside the image)."""
    h, w = depth_mm.shape[:2]
    x, y = int(round(u)), int(round(v))
    if not (0 <= x < w and 0 <= y < h):
        return None   # e.g. the centre of a box cut by the image border
    patch = depth_mm[max(0, y - half_window):min(h, y + half_window + 1),
                     max(0, x - half_window):min(w, x + half_window + 1)]
    patch = patch[patch > 0]
    return float(np.median(patch)) / 1000.0 if patch.size else None


def points_in_polygon(depth_mm, intrinsics, polygon, step=4):
    """(N, 3) points (m) of the valid depth pixels inside `polygon` (image points), sampled every `step` px."""
    polygon = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
    h, w = depth_mm.shape[:2]
    x0, y0 = np.floor(polygon.min(axis=0)).astype(int)
    x1, y1 = np.ceil(polygon.max(axis=0)).astype(int)
    x0, y0, x1, y1 = max(0, x0), max(0, y0), min(w - 1, x1), min(h - 1, y1)
    if x1 < x0 or y1 < y0:
        return np.empty((0, 3))
    mask = np.zeros((y1 - y0 + 1, x1 - x0 + 1), np.uint8)
    cv2.fillPoly(mask, [np.round(polygon - (x0, y0)).astype(np.int32)], 1)
    ys, xs = np.nonzero(mask[::step, ::step])
    ys, xs = ys * step + y0, xs * step + x0
    z = depth_mm[ys, xs].astype(np.float64) / 1000.0
    valid = z > 0
    xs, ys, z = xs[valid], ys[valid], z[valid]
    return np.column_stack([(xs - intrinsics.cx) / intrinsics.fx * z, (ys - intrinsics.cy) / intrinsics.fy * z, z])


def _least_squares_plane(points):
    centroid = points.mean(axis=0)
    return centroid, np.linalg.svd(points - centroid, full_matrices=False)[2][-1]


def fit_plane(points, min_points=20, trials=64, tolerance=None):
    """(centroid, unit normal facing the camera) of the plane most points lie on, or None.

    RANSAC over point triples (seeded, so repeatable), then a least-squares fit to the points within
    `tolerance` (default 4 mm or 1% of the median depth, whichever is larger) of it: flying pixels at the
    object's edges and whatever else the outline caught do not tilt the plane.
    """
    points = np.asarray(points, dtype=np.float64)
    if len(points) < min_points:
        return None
    if tolerance is None:
        tolerance = max(0.004, 0.01 * float(np.median(points[:, 2])))
    rng = np.random.default_rng(0)
    triples = rng.integers(0, len(points), (trials, 3))
    a, b, c = points[triples[:, 0]], points[triples[:, 1]], points[triples[:, 2]]
    normals = np.cross(b - a, c - a)
    lengths = np.linalg.norm(normals, axis=1)
    good = lengths > 1e-12
    if good.any():
        normals, a = normals[good] / lengths[good, None], a[good]
        support = (np.abs((points[None, :, :] - a[:, None, :]) @ normals[:, :, None])[..., 0] <= tolerance).sum(axis=1)
        best = int(np.argmax(support))
        inliers = np.abs((points - a[best]) @ normals[best]) <= tolerance
        if inliers.sum() >= min_points:
            points = points[inliers]
    centroid, normal = _least_squares_plane(points)
    inliers = np.abs((points - centroid) @ normal) <= tolerance
    if min_points <= inliers.sum() < len(points):
        centroid, normal = _least_squares_plane(points[inliers])
    if float(normal @ centroid) > 0.0:   # the camera is at the origin: face it
        normal = -normal
    return centroid, normal


def ray_plane(intrinsics, u, v, centroid, normal):
    """Where the ray through pixel (u, v) meets the plane, or None if it runs (nearly) parallel or behind."""
    ray = intrinsics.ray(u, v)
    denominator = float(ray @ normal)
    if abs(denominator) < 1e-6:
        return None
    scale = float(centroid @ normal) / denominator
    return ray * scale if scale > 0.0 else None


def _unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else None


def obb_corners(center, size, theta):
    """The 4 image corners of an oriented box (width along theta, image y down), in drawing order."""
    c, s = np.cos(theta), np.sin(theta)
    local = np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]) * np.asarray(size, dtype=np.float64)
    return local @ np.array([[c, s], [-s, c]]) + np.asarray(center, dtype=np.float64)


def planar_pose(depth_mm, intrinsics, center, size, theta, polygon=None, step=4, min_points=20):
    """Pose and metric size of a flat object seen as an oriented box, from the depth inside it.

    Returns (Pose, (width_m, height_m), source) or None. With enough depth points the pose sits where the
    box centre's ray meets a plane fitted to them; z is the plane normal (towards the camera), x runs along
    the box width (angle `theta` in the image) projected into the plane. With too few points it falls back
    to the median depth at the centre and a plane facing the camera (source 'center_depth').
    """
    u, v = float(center[0]), float(center[1])
    corners = obb_corners(center, size, theta)
    plane = fit_plane(points_in_polygon(depth_mm, intrinsics, corners if polygon is None else polygon, step),
                      min_points)
    if plane is not None:
        centroid, normal = plane
        position = ray_plane(intrinsics, u, v, centroid, normal)
        du = 0.25 * max(float(size[0]), float(size[1]), 32.0)
        ends = [ray_plane(intrinsics, u + sign * du * np.cos(theta), v + sign * du * np.sin(theta), centroid, normal)
                for sign in (-1.0, 1.0)]
        on_plane = [ray_plane(intrinsics, cu, cv, centroid, normal) for cu, cv in corners]
        if all(p is not None for p in [position, *ends, *on_plane]):
            x_axis = ends[1] - ends[0]
            x_axis = _unit(x_axis - (x_axis @ normal) * normal)
            if x_axis is not None:
                y_axis = np.cross(normal, x_axis)
                rotation = np.column_stack([x_axis, y_axis, normal])
                metric = (float(np.linalg.norm(on_plane[1] - on_plane[0])),
                          float(np.linalg.norm(on_plane[2] - on_plane[1])))
                return Pose(rotation, position), metric, 'plane'
    z = depth_at(depth_mm, u, v, half_window=3)
    if z is None:
        return None
    normal = np.array([0.0, 0.0, -1.0])
    x_axis = np.array([np.cos(theta), np.sin(theta), 0.0])
    rotation = np.column_stack([x_axis, np.cross(normal, x_axis), normal])
    metric = (float(size[0]) * z / intrinsics.fx, float(size[1]) * z / intrinsics.fy)
    return Pose(rotation, intrinsics.backproject(u, v, z)), metric, 'center_depth'
