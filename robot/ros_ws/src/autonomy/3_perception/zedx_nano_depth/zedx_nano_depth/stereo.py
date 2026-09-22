"""Depth from a calibrated stereo pair: rectification, disparity backend, depth in millimetres.

`StereoRectifier` turns per-eye K, D, R, P (a ROS CameraInfo pair, or
zedx_nano_camera.calibration.stereo_rectification() directly) into OpenCV rectification maps.
`StereoMatcher` runs a disparity backend on a downscaled rectified pair and returns a uint16
depth map in millimetres aligned to the rectified left image; 0 means invalid.
"""
from __future__ import annotations

import cv2
import numpy as np


class StereoRectifier:
    """Rectification maps and rectified pinhole model of a stereo pair.

    `left` and `right` map 'K' (3x3), 'D', 'R' (3x3) and 'P' (3x4, only its 3x3 part is used)
    to arrays; `baseline_mm` is the rectified baseline.
    """

    def __init__(self, size, left, right, baseline_mm):
        width, height = (int(v) for v in size)
        p1, p2 = np.asarray(left['P'], dtype=np.float64), np.asarray(right['P'], dtype=np.float64)
        self.size = (width, height)
        self.maps_left = cv2.initUndistortRectifyMap(np.asarray(left['K'], dtype=np.float64), np.asarray(left['D'], dtype=np.float64),
                                                     np.asarray(left['R'], dtype=np.float64), p1, (width, height), cv2.CV_16SC2)
        self.maps_right = cv2.initUndistortRectifyMap(np.asarray(right['K'], dtype=np.float64), np.asarray(right['D'], dtype=np.float64),
                                                      np.asarray(right['R'], dtype=np.float64), p2, (width, height), cv2.CV_16SC2)
        self.fx, self.fy = float(p1[0, 0]), float(p1[1, 1])
        self.cx, self.cy = float(p1[0, 2]), float(p1[1, 2])
        self.baseline_mm = float(baseline_mm)
        if not self.baseline_mm > 0:
            raise ValueError(f'Stereo baseline must be positive, got {self.baseline_mm} mm')

    @classmethod
    def from_camera_info(cls, left, right):
        """Rectifier from a sensor_msgs/CameraInfo pair (right P[0,3] = -fx * baseline in metres)."""
        size = (int(left.width), int(left.height))
        if (int(right.width), int(right.height)) != size or 0 in size:
            raise ValueError('Left and right CameraInfo must have the same non-zero size')

        def eye(info):
            return {'K': np.array(info.k, dtype=np.float64).reshape(3, 3), 'D': np.array(info.d, dtype=np.float64),
                    'R': np.array(info.r, dtype=np.float64).reshape(3, 3), 'P': np.array(info.p, dtype=np.float64).reshape(3, 4)}
        left_eye, right_eye = eye(left), eye(right)
        projection = right_eye['P']
        if projection[0, 0] <= 0:
            raise ValueError('Right CameraInfo has no projection matrix; the camera must publish a stereo calibration')
        return cls(size, left_eye, right_eye, -projection[0, 3] / projection[0, 0] * 1000.0)

    def rectify(self, left_bgr, right_bgr):
        return (cv2.remap(left_bgr, *self.maps_left, cv2.INTER_LINEAR),
                cv2.remap(right_bgr, *self.maps_right, cv2.INTER_LINEAR))

    def rectified_calibration(self, base, method='opencv_sgbm'):
        """Calibration dict for the rectified left image (pinhole, no distortion), derived from `base`."""
        cal = dict(base)
        cal['left'] = {'fx': self.fx, 'fy': self.fy, 'cx': self.cx, 'cy': self.cy,
                       'k1': 0.0, 'k2': 0.0, 'p1': 0.0, 'p2': 0.0, 'k3': 0.0}
        cal['right'] = dict(cal['left'])
        cal['resolution'] = [self.size[0], self.size[1]]
        cal['rectified'] = True
        cal['stereo'] = dict(base.get('stereo', {}), baseline_m=self.baseline_mm / 1000.0, rectified=True)
        cal['depth'] = {'unit': 'mm', 'dtype': 'uint16', 'invalid': 0, 'aligned_to': 'left', 'available': True,
                        'method': method, 'source': 'workstation'}
        return cal

    def rectified_camera_info_fields(self):
        """sensor_msgs/CameraInfo fields of the rectified left image (and of the depth aligned to it)."""
        width, height = self.size
        return {'width': width, 'height': height, 'distortion_model': 'plumb_bob', 'd': [0.0] * 5,
                'k': [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0],
                'r': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                'p': [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]}


class SGBMBackend:
    """Classical semi-global block matching (OpenCV). Input: rectified BGR pair; output: float32 disparity."""

    name = 'opencv_sgbm'

    def __init__(self, num_disparities=64, block_size=5, **_ignored):
        self.num_disparities = int(np.ceil(num_disparities / 16.0) * 16)
        channels = 1
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=self.num_disparities, blockSize=block_size,
            P1=8 * channels * block_size * block_size, P2=32 * channels * block_size * block_size,
            disp12MaxDiff=1, uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    def disparity(self, left_bgr, right_bgr):
        gray_l = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
        return self.sgbm.compute(gray_l, gray_r).astype(np.float32) / 16.0


def make_backend(name, **options):
    """Depth backend by name: 'sgbm' (CPU, always available) or 'neural' (GPU stereo network)."""
    if name == 'sgbm':
        return SGBMBackend(**{k: v for k, v in options.items() if k in ('num_disparities', 'block_size')})
    if name == 'neural':
        from .neural_stereo import NeuralStereoBackend
        return NeuralStereoBackend(**options)
    raise ValueError(f"Unknown depth backend {name!r}; expected 'sgbm' or 'neural'")


class StereoMatcher:
    """Disparity from a backend on a downscaled rectified pair, converted to a uint16 depth map in mm."""

    def __init__(self, calibration, match_width=640, num_disparities=64, block_size=5, max_depth_mm=4000, backend=None):
        self.calibration = calibration
        width, height = calibration.size
        self.match_width = min(int(match_width) or width, width)
        self.match_size = (self.match_width, int(round(height * self.match_width / width)))
        self.scale = self.match_width / width
        self.max_depth_mm = float(max_depth_mm)
        self.fx_match = calibration.fx * self.scale
        self.backend = backend if backend is not None else SGBMBackend(num_disparities, block_size)
        self.num_disparities = getattr(self.backend, 'num_disparities', num_disparities)
        self.min_depth_mm = self.fx_match * calibration.baseline_mm / max(self.num_disparities, 1)
        warmup = getattr(self.backend, 'warmup', None)
        if warmup is not None:
            warmup(self.match_size)   # first GPU inference is several times slower (allocations, autotuning)

    def depth_from_rectified(self, left_rect_bgr, right_rect_bgr):
        """Depth (uint16 mm, 0 = invalid) at the rectified image size."""
        if self.scale != 1.0:
            left_rect_bgr = cv2.resize(left_rect_bgr, self.match_size, interpolation=cv2.INTER_AREA)
            right_rect_bgr = cv2.resize(right_rect_bgr, self.match_size, interpolation=cv2.INTER_AREA)
        disparity = np.asarray(self.backend.disparity(left_rect_bgr, right_rect_bgr), dtype=np.float32)
        if disparity.shape != left_rect_bgr.shape[:2]:
            raise ValueError('Depth backend returned a disparity map of the wrong size')
        valid = np.isfinite(disparity) & (disparity > 0.5)
        depth = np.zeros(disparity.shape, dtype=np.float32)
        depth[valid] = self.fx_match * self.calibration.baseline_mm / disparity[valid]
        depth[depth > self.max_depth_mm] = 0
        depth16 = np.clip(depth, 0, 65535).astype(np.uint16)
        if self.scale != 1.0:
            depth16 = cv2.resize(depth16, self.calibration.size, interpolation=cv2.INTER_NEAREST)
        return depth16

    def compute(self, left_bgr, right_bgr):
        """(rectified left BGR, depth uint16 mm) for an unrectified pair."""
        left_rect, right_rect = self.calibration.rectify(left_bgr, right_bgr)
        return left_rect, self.depth_from_rectified(left_rect, right_rect)

    def close(self):
        close = getattr(self.backend, 'close', None)
        if close is not None:
            close()


def colorize_depth(depth_mm, near_mm=100.0, far_mm=1000.0):
    """Turbo-coloured BGR visualisation; invalid pixels are black, near is red, far is blue."""
    valid = depth_mm > 0
    scaled = np.zeros(depth_mm.shape, dtype=np.uint8)
    if valid.any():
        norm = 1.0 - (depth_mm[valid].astype(np.float32) - near_mm) / max(far_mm - near_mm, 1.0)
        scaled[valid] = np.clip(255 * norm, 1, 255).astype(np.uint8)
    color = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    color[~valid] = 0
    return color


def depth_stats(depth_mm):
    valid = depth_mm > 0
    if not valid.any():
        return {'valid_fraction': 0.0, 'median_mm': None, 'min_mm': None}
    values = depth_mm[valid]
    return {'valid_fraction': round(float(valid.mean()), 3), 'median_mm': int(np.median(values)),
            'min_mm': int(values.min())}
