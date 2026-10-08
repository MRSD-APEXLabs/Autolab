from dataclasses import replace
import io
import time
from types import SimpleNamespace

import numpy as np
import pytest

from act_inference.cameras import NanoStreamCamera, frame_from_image, make_camera
from act_inference.config import InferenceConfig
from inference_fakes import EXPECTED_MM, FX, H, W, fake_stereo_server, jpeg, part, synthetic_pair


def test_stereo_camera_delivers_rectified_left_and_depth(monkeypatch):
    fake_stereo_server(monkeypatch)
    cam = NanoStreamCamera('xavier', port=1, depth_width=480, depth_backend='sgbm')
    cam.start()
    try:
        frame = cam.latest()
        assert frame.image.shape == (H, W, 3) and frame.depth.shape == (H, W) and frame.depth.dtype == np.uint16
        assert (frame.depth > 0).mean() > 0.4 and frame.header['right_frame_id'] == frame.header['frame_id'] + 2
        assert np.median(frame.depth[frame.depth > 0]) == pytest.approx(EXPECTED_MM, rel=0.08)
        assert cam.calibration['rectified'] and cam.calibration['depth']['available']
        assert cam.calibration['depth']['method'] == 'opencv_sgbm' and cam.calibration['left']['fx'] == pytest.approx(FX, rel=1e-2)
        # image is RGB: channel order reversed relative to the (BGR) JPEG
        left_bgr = synthetic_pair()[0]
        assert np.abs(frame.image[100:-100, 100:-100, 0].astype(int) - left_bgr[100:-100, 100:-100, 2]).mean() < 6
        cam.check_mode()
    finally:
        cam.stop()
    assert cam._error is None and not cam._depth_thread.is_alive() and not cam._thread.is_alive()


def test_receive_rejects_backwards_frame_ids_and_reports_stream_end():
    left, right = (jpeg(np.zeros((6, 8, 3), np.uint8)),) * 2
    cam = NanoStreamCamera('test')
    cam._wall_anchor = time.time_ns() - time.monotonic_ns()
    cam._response = io.BytesIO(part(5, left, right) + part(5, left, right))
    cam._receive()
    assert 'backwards' in str(cam._error)
    cam = NanoStreamCamera('test')
    cam._wall_anchor = time.time_ns() - time.monotonic_ns()
    cam._response = io.BytesIO(part(1, left, right) + part(2, left, right))
    cam._receive()
    assert cam.frames_received == 2 and 'ended' in str(cam._error) and cam._pending[0]['x-frame-id'] == '2'
    with pytest.raises(RuntimeError, match='disconnected'):
        cam.check_mode()


def test_stale_and_fresh_frames():
    cam = NanoStreamCamera('test')
    now = time.time_ns()
    headers = {'x-frame-id': '1', 'x-capture-ns': str(now), 'x-ready-ns': str(now)}
    fresh = frame_from_image(headers, np.zeros((6, 8, 3), np.uint8), np.ones((6, 8), np.uint16), now, time.monotonic(), 0)
    cam._latest = fresh
    assert cam.latest() is fresh
    cam._latest = replace(fresh, captured_monotonic=time.monotonic() - 1)
    with pytest.raises(RuntimeError, match='stale'):
        cam.latest()
    cam.depth_ms = 400.0     # slow depth widens the budget by two compute periods
    assert cam.latest() is cam._latest
    cam._latest = None
    with pytest.raises(RuntimeError, match='No camera frame'):
        cam.latest()
    with pytest.raises(ValueError, match='Depth map'):
        frame_from_image(headers, np.zeros((6, 8, 3), np.uint8), np.zeros((4, 4), np.uint16), now, 0.0, 0)


def test_make_camera_sources():
    cam = make_camera(InferenceConfig(camera_host='xavier', camera_port=9000, depth_models_dir='/m', max_frame_age=.3))
    assert isinstance(cam, NanoStreamCamera) and cam.client.url('/info') == 'http://xavier:9000/info'
    assert cam.depth_models_dir == '/m' and cam.max_age == .3
    ros = make_camera(InferenceConfig(camera_source='ros', ros_namespace='wrist/'))
    assert type(ros).__name__ == 'RosDepthCamera' and ros._topic('depth/image_rect') == '/wrist/depth/image_rect'
    with pytest.raises(ValueError):
        make_camera(SimpleNamespace(camera_source='camera-edge'))
