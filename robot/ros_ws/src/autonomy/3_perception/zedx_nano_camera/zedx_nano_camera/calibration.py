"""Stereolabs factory calibration (SN<serial>.conf) for the ZED X Nano and ZED X at the streamed resolution.

Conventions (Stereolabs' zed-opencv-calibration / staff answers, verified on the lab Nano
2026-09-19): K per eye from the LEFT_CAM_*/RIGHT_CAM_* section of the capture mode, the
8-coefficient rational distortion from LEFT_DISTO/RIGHT_DISTO when present (else the
per-resolution k1,k2,p1,p2,k3), R = Rodrigues((RX, CV, RZ)) in radians, T = (-Baseline, TY,
TZ) in mm (the file stores |Tx|), cv2.stereoRectify(..., CALIB_ZERO_DISPARITY, alpha=0).
Feature matches on the rectified pair then agree vertically to under a pixel and
disparities are positive. The ZED X's file has the same layout; the hub streams its SVGA mode (960x600).
"""
from __future__ import annotations

import configparser

import cv2
import numpy as np

SECTION_BY_CAPTURE = {(1920, 1200): 'FHD1200', (1920, 1080): 'FHD', (960, 600): 'SVGA'}
_DISTORTION = ('k1', 'k2', 'p1', 'p2', 'k3')
_RATIONAL = ('k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6')


def parse_calibration_conf(text):
    """Parse a Stereolabs SN<serial>.conf into {section: {key: float}}."""
    parser = configparser.ConfigParser(interpolation=None)
    parser.optionxform = str
    parser.read_string(text)
    return {section: {key: float(value) for key, value in parser[section].items()}
            for section in parser.sections()}


def parse_calibration_text(text, source):
    if '[STEREO]' not in text:
        raise ValueError(f'{source} did not return a Stereolabs calibration file')
    try:
        return parse_calibration_conf(text)
    except configparser.Error as exc:
        raise ValueError(f'{source} returned an unreadable calibration file: {exc}') from None


def _section(capture_size):
    capture = tuple(int(v) for v in capture_size)
    section = SECTION_BY_CAPTURE.get(capture)
    if section is None:
        raise ValueError(f'No calibration section for capture size {capture}')
    return capture, section


def build_calibration(conf, capture_size, output_size, serial=0, source='calib.stereolabs.com', model='ZED X Nano'):
    """Scale the factory intrinsics of the capture mode to the streamed image size (unrectified)."""
    capture, section = _section(capture_size)
    try:
        left, right, stereo = conf['LEFT_CAM_' + section], conf['RIGHT_CAM_' + section], conf['STEREO']
    except KeyError as exc:
        raise ValueError(f'Calibration file lacks section {exc}') from None
    width, height = (int(v) for v in output_size)
    sx, sy = width / capture[0], height / capture[1]

    def eye(values):
        result = {'fx': values['fx'] * sx, 'fy': values['fy'] * sy, 'cx': values['cx'] * sx, 'cy': values['cy'] * sy}
        result.update({key: values.get(key, 0.0) for key in _DISTORTION})
        return result

    return {
        'resolution': [width, height],
        'capture_resolution': list(capture),
        'left': eye(left),
        'right': eye(right),
        'stereo': {'baseline_m': stereo['Baseline'] / 1000.0, 'ty_m': stereo.get('TY', 0.0) / 1000.0,
                   'tz_m': stereo.get('TZ', 0.0) / 1000.0, 'rx': stereo.get('RX_' + section, 0.0),
                   'cv': stereo.get('CV_' + section, 0.0), 'rz': stereo.get('RZ_' + section, 0.0)},
        'depth': {'unit': 'mm', 'dtype': 'uint16', 'invalid': 0, 'aligned_to': 'left', 'available': False},
        'camera_model': model, 'serial': int(serial), 'rectified': False,
        'distortion_model': 'opencv_k1k2p1p2k3', 'source': source,
    }


def stereo_rectification(conf, capture_size, image_size):
    """Rectification of a stereo pair at the streamed resolution.

    `conf` is the parsed calibration file, `capture_size` the sensor mode (the section the
    intrinsics belong to) and `image_size` the size of the streamed images. Returns
    {'size', 'baseline_mm', 'left': {K, D, R, P}, 'right': {K, D, R, P}} with the float64
    matrices of cv2.stereoRectify; P's translation column is in millimetres.
    """
    capture, section = _section(capture_size)
    width, height = (int(v) for v in image_size)
    sx, sy = width / capture[0], height / capture[1]

    def intrinsics(name, disto_name):
        try:
            c = conf[name]
        except KeyError:
            raise ValueError(f'Calibration file lacks section {name}') from None
        k = np.array([[c['fx'] * sx, 0, c['cx'] * sx], [0, c['fy'] * sy, c['cy'] * sy], [0, 0, 1]], dtype=np.float64)
        disto = conf.get(disto_name)
        if disto and all(key in disto for key in _RATIONAL):
            # Resolution-independent rational model (SDK >= 3.5 prefers it over the per-resolution set).
            d = np.array([disto[key] for key in _RATIONAL], dtype=np.float64)
        else:
            d = np.array([c.get(key, 0.0) for key in _DISTORTION], dtype=np.float64)
        return k, d

    k1, d1 = intrinsics('LEFT_CAM_' + section, 'LEFT_DISTO')
    k2, d2 = intrinsics('RIGHT_CAM_' + section, 'RIGHT_DISTO')
    try:
        stereo = conf['STEREO']
    except KeyError:
        raise ValueError('Calibration file lacks section STEREO') from None
    rvec = np.array([stereo.get('RX_' + section, 0.0), stereo.get('CV_' + section, 0.0),
                     stereo.get('RZ_' + section, 0.0)], dtype=np.float64)
    rotation, _ = cv2.Rodrigues(rvec)
    translation = np.array([-stereo['Baseline'], stereo.get('TY', 0.0), stereo.get('TZ', 0.0)], dtype=np.float64)
    r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(k1, d1, k2, d2, (width, height), rotation, translation,
                                                flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    return {'size': (width, height), 'baseline_mm': float(-p2[0, 3] / p2[0, 0]),
            'left': {'K': k1, 'D': d1, 'R': r1, 'P': p1},
            'right': {'K': k2, 'D': d2, 'R': r2, 'P': p2}}


def camera_info_fields(rectification, eye):
    """sensor_msgs/CameraInfo fields for 'left' or 'right' of `stereo_rectification()`.

    Follows the ROS convention: K and D describe the raw image, R and P the rectification,
    and the right camera's P[0,3] = -fx' * baseline with the baseline in metres.
    """
    matrices = rectification[eye]
    projection = np.array(matrices['P'], dtype=np.float64)
    projection[:, 3] /= 1000.0
    distortion = np.asarray(matrices['D'], dtype=np.float64)
    width, height = rectification['size']
    return {'width': int(width), 'height': int(height),
            'distortion_model': 'rational_polynomial' if len(distortion) == 8 else 'plumb_bob',
            'd': distortion.tolist(), 'k': np.asarray(matrices['K'], dtype=np.float64).reshape(-1).tolist(),
            'r': np.asarray(matrices['R'], dtype=np.float64).reshape(-1).tolist(),
            'p': projection.reshape(-1).tolist()}
