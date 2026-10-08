"""Synthetic stereo rig: a textured fronto-parallel plane seen by an ideal, already rectified pair."""
from types import SimpleNamespace

import cv2
import numpy as np

W, H, FX, BASELINE_MM, SHIFT = 960, 600, 480.0, 18.0, 24
EXPECTED_MM = FX * BASELINE_MM / SHIFT   # 360 mm


def synthetic_pair(shift=SHIFT, seed=0):
    """The right view is the left shifted `shift` px to the left (disparity `shift` everywhere)."""
    rng = np.random.RandomState(seed)
    texture = cv2.GaussianBlur((rng.rand(H, W + shift, 3) * 255).astype(np.uint8), (0, 0), 1.2)
    return texture[:, :W].copy(), texture[:, shift:shift + W].copy()


def ideal_eyes():
    k = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1]])
    left = {'K': k, 'D': np.zeros(5), 'R': np.eye(3), 'P': np.hstack([k, np.zeros((3, 1))])}
    right = dict(left, P=np.hstack([k, [[-FX * BASELINE_MM], [0], [0]]]))
    return left, right


def camera_info(eye):
    """A duck-typed sensor_msgs/CameraInfo of the ideal rig (P translation in metres)."""
    p = eye['P'].copy()
    p[:, 3] /= 1000.0
    return SimpleNamespace(width=W, height=H, k=eye['K'].reshape(-1).tolist(), d=eye['D'].tolist(),
                           r=eye['R'].reshape(-1).tolist(), p=p.reshape(-1).tolist())


def valid_center(depth):
    center = depth[60:-60, 100:-100]
    return center[center > 0]
