import cv2
import numpy as np
import pytest

from camera_fakes import CONF, ZEDX_CONF
from zedx_nano_camera.calibration import (build_calibration, camera_info_fields, parse_calibration_conf,
                                          parse_calibration_text, stereo_rectification)


def conf():
    return parse_calibration_conf(CONF)


def test_calibration_is_scaled_to_stream_size():
    cal = build_calibration(conf(), (1920, 1200), (960, 600), serial=99292912)
    assert cal['left']['fx'] == pytest.approx(475) and cal['left']['cx'] == pytest.approx(480)
    assert cal['left']['cy'] == pytest.approx(300) and cal['left']['k1'] == pytest.approx(0.04)
    assert cal['right']['fx'] == pytest.approx(475.5) and cal['stereo']['baseline_m'] == pytest.approx(0.01801)
    assert cal['stereo']['ty_m'] == pytest.approx(-0.00004) and cal['stereo']['tz_m'] == pytest.approx(0.00019)
    assert cal['resolution'] == [960, 600] and cal['capture_resolution'] == [1920, 1200]
    assert cal['depth']['available'] is False and cal['serial'] == 99292912 and cal['rectified'] is False
    assert cal['camera_model'] == 'ZED X Nano'
    assert build_calibration(conf(), (1920, 1200), (960, 600), model='ZED X')['camera_model'] == 'ZED X'
    with pytest.raises(ValueError):
        build_calibration(conf(), (640, 480), (640, 480))
    with pytest.raises(ValueError):
        build_calibration(parse_calibration_conf('[STEREO]\nBaseline=1\n'), (1920, 1200), (960, 600))


def test_zedx_svga_calibration_is_used_unscaled():
    cal = build_calibration(parse_calibration_conf(ZEDX_CONF), (960, 600), (960, 600), serial=42757821, model='ZED X')
    assert cal['left']['fx'] == pytest.approx(367.5) and cal['capture_resolution'] == [960, 600]
    rect = stereo_rectification(parse_calibration_conf(ZEDX_CONF), (960, 600), (960, 600))
    assert rect['size'] == (960, 600) and rect['baseline_mm'] == pytest.approx(119.87, rel=0.01)


def test_parse_calibration_text_rejects_non_calibration_files():
    assert parse_calibration_text(CONF, 'test')['MISC']['Sensor_ID'] == 1
    with pytest.raises(ValueError, match='did not return'):
        parse_calibration_text('<html>404</html>', 'test')
    with pytest.raises(ValueError, match='unreadable'):
        parse_calibration_text('[STEREO]\nBaseline\n', 'test')


def test_rectification_aligns_rows_and_gives_positive_disparity():
    rect = stereo_rectification(conf(), (1920, 1200), (960, 600))
    assert rect['size'] == (960, 600) and rect['baseline_mm'] == pytest.approx(18.01, rel=0.01)
    left, right = rect['left'], rect['right']
    for eye in (left, right):
        assert np.allclose(eye['R'] @ eye['R'].T, np.eye(3), atol=1e-9) and eye['D'].shape == (5,)
    assert left['P'][0, 0] == right['P'][0, 0] and left['P'][1, 2] == right['P'][1, 2] and left['P'][0, 3] == 0
    # Project 3D points (left camera frame, mm) into both raw images with the same extrinsics
    # stereo_rectification uses, then rectify them: rows must agree and disparity = fx * B / Z.
    stereo = conf()['STEREO']
    rotation, _ = cv2.Rodrigues(np.array([stereo['RX_FHD1200'], stereo['CV_FHD1200'], stereo['RZ_FHD1200']]))
    translation = np.array([-stereo['Baseline'], stereo['TY'], stereo['TZ']])
    points = np.array([[x, y, z] for x in (-80.0, 0.0, 90.0) for y in (-50.0, 40.0) for z in (250.0, 600.0)])
    raw_left = cv2.projectPoints(points, np.zeros(3), np.zeros(3), left['K'], left['D'])[0]
    raw_right = cv2.projectPoints(points, cv2.Rodrigues(rotation)[0], translation, right['K'], right['D'])[0]
    rect_left = cv2.undistortPoints(raw_left, left['K'], left['D'], R=left['R'], P=left['P']).reshape(-1, 2)
    rect_right = cv2.undistortPoints(raw_right, right['K'], right['D'], R=right['R'], P=right['P']).reshape(-1, 2)
    assert np.abs(rect_left[:, 1] - rect_right[:, 1]).max() < 0.05
    disparity = rect_left[:, 0] - rect_right[:, 0]
    depth = (left['R'] @ points.T).T[:, 2]
    np.testing.assert_allclose(disparity, left['P'][0, 0] * rect['baseline_mm'] / depth, rtol=2e-3)


def test_rational_distortion_is_used_when_complete():
    text = CONF + '\n[LEFT_DISTO]\nk1=1\nk2=2\np1=3\np2=4\nk3=5\nk4=6\nk5=7\nk6=8\n'
    rect = stereo_rectification(parse_calibration_conf(text), (1920, 1200), (960, 600))
    assert rect['left']['D'].tolist() == [1, 2, 3, 4, 5, 6, 7, 8] and rect['right']['D'].shape == (5,)
    fields = camera_info_fields(rect, 'left')
    assert fields['distortion_model'] == 'rational_polynomial' and len(fields['d']) == 8
    partial = CONF + '\n[LEFT_DISTO]\nk1=1\n'
    assert stereo_rectification(parse_calibration_conf(partial), (1920, 1200), (960, 600))['left']['D'].shape == (5,)


def test_camera_info_fields_follow_ros_conventions():
    rect = stereo_rectification(conf(), (1920, 1200), (960, 600))
    left, right = camera_info_fields(rect, 'left'), camera_info_fields(rect, 'right')
    for fields, eye in ((left, 'left'), (right, 'right')):
        assert fields['width'] == 960 and fields['height'] == 600 and fields['distortion_model'] == 'plumb_bob'
        assert len(fields['k']) == 9 and len(fields['r']) == 9 and len(fields['p']) == 12 and len(fields['d']) == 5
        np.testing.assert_array_equal(np.reshape(fields['k'], (3, 3)), rect[eye]['K'])
    assert left['p'][3] == 0
    # Tx = -fx' * baseline in metres
    assert -right['p'][3] / right['p'][0] == pytest.approx(rect['baseline_mm'] / 1000.0)
    assert right['p'][:3] == rect['right']['P'][0, :3].tolist()
    with pytest.raises(ValueError):
        stereo_rectification(conf(), (1280, 720), (640, 360))
    with pytest.raises(ValueError, match='STEREO'):
        stereo_rectification({k: v for k, v in conf().items() if k != 'STEREO'}, (1920, 1200), (960, 600))
