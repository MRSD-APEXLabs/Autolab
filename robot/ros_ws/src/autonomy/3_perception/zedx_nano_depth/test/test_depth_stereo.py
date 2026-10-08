import os
from pathlib import Path

import numpy as np
import pytest

from depth_fakes import BASELINE_MM, EXPECTED_MM, FX, H, W, camera_info, ideal_eyes, synthetic_pair, valid_center
from zedx_nano_depth.stereo import (SGBMBackend, StereoMatcher, StereoRectifier, colorize_depth, depth_stats,
                                    make_backend)


def rectifier():
    left, right = ideal_eyes()
    return StereoRectifier((W, H), left, right, BASELINE_MM)


def test_synthetic_plane_depth_matches_geometry():
    calibration = rectifier()
    assert calibration.fx == FX and calibration.baseline_mm == BASELINE_MM
    left, right = synthetic_pair()
    for match_width in (W, 480):
        matcher = StereoMatcher(calibration, match_width=match_width, num_disparities=64)
        left_rect, depth = matcher.compute(left, right)
        assert left_rect.shape == left.shape and depth.shape == (H, W) and depth.dtype == np.uint16
        valid = valid_center(depth)
        assert len(valid) > 0.5 * (H - 120) * (W - 200)
        assert np.median(valid) == pytest.approx(EXPECTED_MM, rel=0.06), (match_width, np.median(valid))
    stats = depth_stats(depth)
    assert stats['valid_fraction'] > 0.5 and abs(stats['median_mm'] - EXPECTED_MM) < 0.06 * EXPECTED_MM
    assert depth_stats(np.zeros((4, 4), np.uint16)) == {'valid_fraction': 0.0, 'median_mm': None, 'min_mm': None}


def test_max_depth_is_invalidated():
    left, right = synthetic_pair()
    _, depth = StereoMatcher(rectifier(), match_width=480, max_depth_mm=EXPECTED_MM * 0.8).compute(left, right)
    assert not valid_center(depth).size or np.median(valid_center(depth)) < EXPECTED_MM * 0.8


def test_rectifier_from_camera_info_matches_direct_construction():
    left, right = ideal_eyes()
    direct = rectifier()
    from_info = StereoRectifier.from_camera_info(camera_info(left), camera_info(right))
    assert from_info.size == direct.size and from_info.baseline_mm == pytest.approx(BASELINE_MM)
    assert (from_info.fx, from_info.fy, from_info.cx, from_info.cy) == (direct.fx, direct.fy, direct.cx, direct.cy)
    for a, b in zip(direct.maps_left + direct.maps_right, from_info.maps_left + from_info.maps_right):
        np.testing.assert_array_equal(a, b)
    fields = from_info.rectified_camera_info_fields()
    assert fields['width'] == W and fields['k'] == [FX, 0.0, W / 2, 0.0, FX, H / 2, 0.0, 0.0, 1.0]
    assert fields['p'][3] == 0.0 and fields['d'] == [0.0] * 5 and fields['distortion_model'] == 'plumb_bob'


def test_rectifier_rejects_bad_camera_info():
    left, right = ideal_eyes()
    small = camera_info(right)
    small.width = 640
    with pytest.raises(ValueError, match='same non-zero size'):
        StereoRectifier.from_camera_info(camera_info(left), small)
    no_projection = camera_info(right)
    no_projection.p = [0.0] * 12
    with pytest.raises(ValueError, match='projection'):
        StereoRectifier.from_camera_info(camera_info(left), no_projection)
    swapped = camera_info(left)          # right eye with Tx = 0: zero baseline
    with pytest.raises(ValueError, match='baseline'):
        StereoRectifier.from_camera_info(camera_info(left), swapped)


def test_rectified_calibration_and_colorize():
    base = {'resolution': [W, H], 'left': {}, 'right': {}, 'stereo': {'baseline_m': 0.018}, 'serial': 1,
            'depth': {'available': False}, 'rectified': False}
    cal = rectifier().rectified_calibration(base)
    assert cal['rectified'] and cal['depth']['available'] and cal['depth']['method'] == 'opencv_sgbm'
    assert cal['left']['fx'] == FX and cal['left']['k1'] == 0 and cal['resolution'] == [W, H]
    assert cal['stereo']['baseline_m'] == pytest.approx(0.018) and base['depth']['available'] is False
    depth = np.zeros((10, 10), np.uint16)
    depth[2:5, 2:5] = 150
    depth[6:8, 6:8] = 900
    color = colorize_depth(depth, near_mm=100, far_mm=1000)
    assert color.shape == (10, 10, 3) and not color[0, 0].any() and color[3, 3].any() and color[7, 7].any()
    assert not np.array_equal(color[3, 3], color[7, 7])


def cuda_available():
    try:
        import torch
        return torch.cuda.is_available()
    except ImportError:
        return False


def test_backend_registry_and_errors():
    assert isinstance(make_backend('sgbm', model='ignored', models_dir=None), SGBMBackend)
    with pytest.raises(ValueError, match='Unknown depth backend'):
        make_backend('magic')
    if cuda_available():
        with pytest.raises(ValueError, match='Unknown depth model'):
            make_backend('neural', model='no-such-model')


def models_dir():
    from zedx_nano_depth.neural_stereo import MODELS_DIR
    return Path(os.environ.get('ZEDX_NANO_DEPTH_TEST_MODELS') or MODELS_DIR)


@pytest.mark.skipif(not cuda_available(), reason='needs a CUDA GPU')
def test_raft_realtime_backend_recovers_the_synthetic_plane():
    if not (models_dir() / 'raftstereo-realtime.pth').exists():
        pytest.skip(f'RAFT-Stereo weights not in {models_dir()} (set ZEDX_NANO_DEPTH_TEST_MODELS)')
    backend = make_backend('neural', model='raft-realtime', models_dir=models_dir())
    try:
        assert backend.name == 'raft_stereo_realtime' and backend.describe()['model'] == 'raft-realtime'
        matcher = StereoMatcher(rectifier(), match_width=480, backend=backend)
        _, depth = matcher.compute(*synthetic_pair())
        valid = valid_center(depth)
        assert len(valid) > 0.9 * (H - 120) * (W - 200)
        assert np.median(valid) == pytest.approx(EXPECTED_MM, rel=0.06)
    finally:
        backend.close()
