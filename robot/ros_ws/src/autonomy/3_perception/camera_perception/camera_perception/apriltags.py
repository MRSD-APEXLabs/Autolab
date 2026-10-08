"""AprilTag detection with 6-DoF poses from the tag corners, checked against the stereo depth.

Tag frame (the AprilTag / OpenCV IPPE_SQUARE convention): origin at the tag centre, x to the right and
y down as the tag is printed, z into the tag, i.e. away from a camera that sees its face.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from .geometry import Pose, depth_at, fit_plane, points_in_polygon, ray_plane

POSITION_MODES = ('pnp', 'depth')


def parse_sizes(text):
    """{id: size_m} from 'id=size, id=size' (also accepts ':' and whitespace separators)."""
    sizes = {}
    for item in text.replace(',', ' ').split():
        key, sep, value = item.replace(':', '=').partition('=')
        if not sep:
            raise ValueError(f'tag size {item!r} is not id=size_m')
        size = float(value)
        if not size > 0.0:
            raise ValueError(f'tag {key} size must be > 0, got {value}')
        sizes[int(key)] = size
    return sizes


def object_points(size):
    """Tag corners in the tag frame, in the order the detector reports them (and IPPE_SQUARE expects)."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


@dataclass
class Tag:
    id: int
    center: np.ndarray              # (2,) px
    corners: np.ndarray             # (4, 2) px, detector order
    size_m: float                   # configured edge length
    hamming: int = 0
    decision_margin: float = 0.0
    pose: Pose | None = None
    pose_source: str | None = None  # 'pnp', 'depth' (PnP rotation, depth position) or 'center_depth'
    reprojection_px: float | None = None
    measured_size_m: float | None = None   # edge length from the stereo depth, if the tag had enough of it
    extra: dict = field(default_factory=dict)

    def as_dict(self):
        """Camera-Edge `apriltags` entry (id, center, corners, pose) plus diagnostics."""
        return {
            'id': self.id,
            'center': [float(v) for v in self.center],
            'corners': [[float(x), float(y)] for x, y in self.corners],
            'pose': self.pose.as_dict() if self.pose is not None else None,
            'pose_source': self.pose_source,
            'size_m': self.size_m,
            'measured_size_m': self.measured_size_m,
            'reprojection_px': self.reprojection_px,
            'hamming': self.hamming,
            'decision_margin': self.decision_margin,
        }


class TagDetector:
    """pupil_apriltags detection, keeping `ids` (all if empty), with poses from IPPE_SQUARE.

    Of the two IPPE solutions (a small tag's face-on pose is ambiguous), the one whose z axis agrees
    with the plane fitted to the depth inside the tag wins; without depth the lower reprojection error
    does. `position='depth'` replaces the PnP translation with where the centre ray meets that plane.
    """

    def __init__(self, family='tag36h11', ids=(), size=0.0625, sizes=None, nthreads=2, quad_decimate=1.0,
                 max_hamming=1, position='pnp', depth_step=2):
        if position not in POSITION_MODES:
            raise ValueError(f'position must be one of {POSITION_MODES}, got {position!r}')
        from pupil_apriltags import Detector   # native library: import where it is needed
        self.detector = Detector(families=family, nthreads=int(nthreads), quad_decimate=float(quad_decimate))
        self.ids = frozenset(int(i) for i in ids)
        self.size = float(size)
        self.sizes = dict(sizes or {})
        self.max_hamming = int(max_hamming)
        self.position = position
        self.depth_step = int(depth_step)

    def size_of(self, tag_id):
        return self.sizes.get(tag_id, self.size)

    def detect(self, bgr, depth_mm=None, intrinsics=None):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        tags = []
        for r in self.detector.detect(np.ascontiguousarray(gray)):
            if (self.ids and r.tag_id not in self.ids) or r.hamming > self.max_hamming:
                continue
            tag = Tag(int(r.tag_id), np.asarray(r.center, np.float64), np.asarray(r.corners, np.float64),
                      self.size_of(int(r.tag_id)), int(r.hamming), float(r.decision_margin))
            if intrinsics is not None:
                self._locate(tag, depth_mm, intrinsics)
            tags.append(tag)
        tags.sort(key=lambda t: t.id)
        return tags

    def _locate(self, tag, depth_mm, intrinsics):
        plane = None
        if depth_mm is not None:
            plane = fit_plane(points_in_polygon(depth_mm, intrinsics, tag.corners, self.depth_step), min_points=12)
        if plane is not None:
            centroid, normal = plane
            on_plane = [ray_plane(intrinsics, u, v, centroid, normal) for u, v in tag.corners]
            if all(p is not None for p in on_plane):
                tag.measured_size_m = float(np.mean([np.linalg.norm(on_plane[i] - on_plane[i - 1]) for i in range(4)]))

        solution = self._solve_pnp(tag, intrinsics, None if plane is None else plane[1])
        if solution is not None:
            rotation, translation, tag.reprojection_px = solution
            tag.pose, tag.pose_source = Pose(rotation, translation), 'pnp'
            if self.position == 'depth' and plane is not None:
                hit = ray_plane(intrinsics, tag.center[0], tag.center[1], *plane)
                if hit is not None:
                    tag.pose, tag.pose_source = Pose(rotation, hit), 'depth'
            return
        # the Camera-Edge fallback: depth at the centre, identity orientation
        z = depth_at(depth_mm, *tag.center, half_window=3) if depth_mm is not None else None
        if z is not None:
            tag.pose, tag.pose_source = Pose(np.eye(3), intrinsics.backproject(*tag.center, z)), 'center_depth'

    @staticmethod
    def _solve_pnp(tag, intrinsics, plane_normal):
        try:
            count, rvecs, tvecs, errors = cv2.solvePnPGeneric(
                object_points(tag.size_m), tag.corners.reshape(4, 1, 2), intrinsics.matrix, None,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error:
            return None
        errors = np.ravel(errors) if errors is not None else np.zeros(count)
        candidates = [(float(errors[i]), cv2.Rodrigues(rvecs[i])[0], tvecs[i].reshape(3)) for i in range(count)
                      if tvecs[i].reshape(3)[2] > 0.0]
        if not candidates:
            return None
        best = min(c[0] for c in candidates)
        if plane_normal is not None:
            # both IPPE solutions fit a small tag about equally well: let the depth plane decide between them
            # (tag z points away from the camera, the plane normal towards it)
            plausible = [c for c in candidates if c[0] <= 1.5 * best + 0.5]
            return _with_error(max(plausible, key=lambda c: float(-c[1][:, 2] @ plane_normal)))
        return _with_error(min(candidates, key=lambda c: c[0]))


def _with_error(candidate):
    error, rotation, translation = candidate
    return rotation, translation, error
