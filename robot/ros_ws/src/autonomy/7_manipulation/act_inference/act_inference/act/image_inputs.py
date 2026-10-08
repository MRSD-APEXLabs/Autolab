"""Shared ACT image encoding for datasets, replay and live inference."""
import cv2
import numpy as np

DEPTH_MAX_MM = 2000.0
DEPTH_ENCODING = 'metric_mm_clip_2000_plus_valid_v1'


def encode_image(rgb, depth=None, use_depth=False):
    """HWC float32: RGB [0,1], optional clipped metric depth [0,1] and validity."""
    rgb = np.asarray(rgb)
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('Expected uint8 RGB')
    image = rgb.astype(np.float32) / 255.0
    if not use_depth:
        return image
    if depth is None or depth.dtype != np.uint16 or depth.shape != rgb.shape[:2]:
        raise ValueError('RGB-depth policy requires aligned uint16 depth in millimeters')
    valid = depth > 0
    if not valid.any():
        raise ValueError('RGB-depth policy received an entirely invalid depth frame')
    metric = np.minimum(depth.astype(np.float32), DEPTH_MAX_MM) / DEPTH_MAX_MM
    return np.concatenate([image, metric[..., None], valid.astype(np.float32)[..., None]], axis=-1)


def resize_pair(rgb, depth, size):
    rgb = cv2.resize(rgb, size, interpolation=cv2.INTER_AREA)
    if depth is not None:
        depth = cv2.resize(depth, size, interpolation=cv2.INTER_NEAREST)
    return rgb, depth
