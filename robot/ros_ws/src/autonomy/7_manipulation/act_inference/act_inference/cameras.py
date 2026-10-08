"""Camera sources for live inference.

Every source has start(), check_mode(), latest(), stop() and a `calibration` dict, and
latest() returns a CameraFrame: the rectified left image as RGB plus a uint16 depth map in
millimetres aligned to it (0 = invalid), which is what the policies were trained on.

`NanoStreamCamera` reads the synchronized stereo pairs straight from the Xavier's MJPEG server
and computes depth on this machine. `ros_camera.RosDepthCamera` (--camera-source ros) instead
reads the zedx_nano_depth node's topics, so one depth network serves every consumer.
"""
from dataclasses import dataclass
import threading
import time

import cv2
import numpy as np

from zedx_nano_camera.calibration import stereo_rectification
from zedx_nano_camera.stream import (NanoStreamClient, capture_size, captured_monotonic, decode_jpeg, frame_header,
                                     iter_mjpeg_parts, output_size, shutdown_response, split_stereo_part)
from zedx_nano_depth.stereo import StereoMatcher, StereoRectifier, make_backend

from .config import CAMERA_SOURCES


@dataclass(frozen=True)
class CameraFrame:
    image: np.ndarray
    depth: np.ndarray
    header: dict
    received_ns: int
    received_monotonic: float
    captured_monotonic: float


def frame_from_image(headers, bgr, depth, received_ns, received_monotonic, offset_ns, mono_offset_ns=None):
    """CameraFrame from a decoded BGR image, a depth map (None: all invalid) and the part's metadata headers."""
    height, width = bgr.shape[:2]
    header = frame_header(headers, width, height)
    if depth is None:
        depth = np.zeros((height, width), dtype=np.uint16)
    elif depth.shape != (height, width) or depth.dtype != np.uint16:
        raise ValueError('Depth map does not match the image')
    return CameraFrame(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), depth, header, received_ns, received_monotonic,
                       captured_monotonic(header, received_ns, received_monotonic, offset_ns, mono_offset_ns))


class NanoStreamCamera:
    """Rectified left RGB and stereo depth from the Xavier's /video_feed/stereo, computed on this machine."""

    def __init__(self, host, max_age=0.25, port=8090, timeout=3.0, depth_width=640, depth_backend='neural',
                 depth_model='raft-realtime', depth_repo='', depth_models_dir=''):
        self.client = NanoStreamClient(host, port, timeout)
        self.max_age, self.timeout = max_age, timeout
        self.depth_width = depth_width
        self.depth_backend, self.depth_model = depth_backend, depth_model
        self.depth_repo, self.depth_models_dir = depth_repo, depth_models_dir
        self.calibration = None
        self.conf = None
        self.info = None
        self._matcher = None
        self._pending = None
        self._pending_cond = threading.Condition()
        self._depth_thread = None
        self.depth_ms = 0.0
        self.depth_info = {}
        self.offset_ns = self.uncertainty_ns = 0
        self.mono_offset_ns = None
        self._wall_anchor = 0
        self._response = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest = None
        self._error = None
        self.frames_received = 0

    def start(self):
        info = self.client.fetch_info('stereo')
        self.info = info
        self.offset_ns, self.uncertainty_ns, self.mono_offset_ns = self.client.synchronize_clock()
        if self.uncertainty_ns / 1e9 > self.max_age / 2:
            raise RuntimeError('Camera clock synchronization is too uncertain')
        self._wall_anchor = time.time_ns() - time.monotonic_ns()
        self.conf, calibration = self.client.load_calibration(info)
        rectification = stereo_rectification(self.conf, capture_size(info), output_size(info))
        rectifier = StereoRectifier(rectification['size'], rectification['left'], rectification['right'],
                                    rectification['baseline_mm'])
        backend = make_backend(self.depth_backend, model=self.depth_model, repo=self.depth_repo or None,
                               models_dir=self.depth_models_dir or None)
        self._matcher = StereoMatcher(rectifier, match_width=self.depth_width, backend=backend)
        self.depth_info = getattr(backend, 'describe', lambda: {'backend': self.depth_backend})()
        self.calibration = rectifier.rectified_calibration(calibration, method=getattr(backend, 'name', self.depth_backend))
        self._depth_thread = threading.Thread(target=self._depth_worker, name='nano-depth', daemon=True)
        self._depth_thread.start()
        self._response = self.client.open_feed('stereo')
        self._thread = threading.Thread(target=self._receive, name='nano-stream', daemon=True)
        self._thread.start()
        deadline = time.monotonic() + 5
        while self._latest is None:
            if self._error is not None:
                raise RuntimeError(f'Camera receive failed: {self._error}') from self._error
            if time.monotonic() > deadline:
                raise TimeoutError('No Nano frames received')
            time.sleep(0.01)
        self.latest()

    def _receive(self):
        try:
            last_id = -1
            for headers, body in iter_mjpeg_parts(self._response):
                wall, mono = time.time_ns(), time.monotonic()
                uses_mono = self.mono_offset_ns is not None and 'x-capture-mono-ns' in headers
                if not uses_mono and abs(wall - int(mono * 1e9) - self._wall_anchor) > 50_000_000:
                    raise RuntimeError('Local clock changed; restart to synchronize cameras')
                frame_id = int(headers.get('x-frame-id', -1))
                if frame_id <= last_id:
                    raise RuntimeError('Camera frame counter reset or moved backwards')
                last_id = frame_id
                left_jpeg, right_jpeg = split_stereo_part(headers, body)
                with self._lock:
                    self.frames_received += 1
                with self._pending_cond:   # single slot: the depth worker always takes the newest pair
                    self._pending = (headers, left_jpeg, right_jpeg, wall, mono)
                    self._pending_cond.notify()
            if not self._stop.is_set():
                raise RuntimeError('Camera stream ended')
        except Exception as exc:
            if not self._stop.is_set():
                self._error = exc

    def _depth_worker(self):
        """Rectifies and matches the newest stereo pair; publishes the frame with its depth map."""
        try:
            while not self._stop.is_set():
                with self._pending_cond:
                    while self._pending is None and not self._stop.is_set():
                        self._pending_cond.wait(0.5)
                    pending, self._pending = self._pending, None
                if pending is None:
                    continue
                headers, left_jpeg, right_jpeg, wall, mono = pending
                t0 = time.monotonic()
                left_rect, depth = self._matcher.compute(decode_jpeg(left_jpeg), decode_jpeg(right_jpeg))
                frame = frame_from_image(headers, left_rect, depth, wall, mono, self.offset_ns, self.mono_offset_ns)
                with self._lock:
                    self._latest = frame
                    self.depth_ms = round(1000 * (time.monotonic() - t0), 1)
        except Exception as exc:
            if not self._stop.is_set():
                self._error = exc

    def check_mode(self, force=False):
        """Health check: the receiver and the depth worker must be alive."""
        if self._error is not None:
            raise RuntimeError(f'Camera disconnected or invalid stream: {self._error}') from self._error
        if self._thread is not None and not self._thread.is_alive():
            raise RuntimeError('Camera receiver stopped')
        if self._depth_thread is not None and not self._depth_thread.is_alive():
            raise RuntimeError('Depth worker stopped')

    def latest(self):
        if self._error is not None:
            raise RuntimeError(f'Camera disconnected or invalid stream: {self._error}') from self._error
        with self._lock:
            frame = self._latest
        if frame is None:
            raise RuntimeError('No camera frame')
        age = time.monotonic() - frame.captured_monotonic
        uncertainty = self.uncertainty_ns / 1e9
        # Depth is computed after capture and the frame stays current until the next one finishes,
        # so a frame can be up to two compute periods old on top of the transport budget.
        allowance = 2 * self.depth_ms / 1000.0 + 0.05
        if age < -uncertainty - 0.01 or age + uncertainty > self.max_age + allowance:
            raise RuntimeError(f'Camera frame stale or clock mismatch: age={age:.3f}s (depth {self.depth_ms:.0f} ms)')
        return frame

    def stop(self):
        self._stop.set()
        response, self._response = self._response, None
        if response is not None:
            shutdown_response(response)
        with self._pending_cond:
            self._pending_cond.notify_all()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=max(2.0, self.timeout))
        if self._depth_thread is not None and self._depth_thread.is_alive():
            self._depth_thread.join(timeout=5)
        if self._matcher is not None:
            self._matcher.close()


def make_camera(config):
    if config.camera_source == 'nano-stream':
        return NanoStreamCamera(config.camera_host, config.max_frame_age, port=config.camera_port,
                                depth_width=config.depth_width, depth_backend=config.depth_backend,
                                depth_model=config.depth_model, depth_repo=config.depth_repo,
                                depth_models_dir=config.depth_models_dir)
    if config.camera_source == 'ros':
        from .ros_camera import RosDepthCamera
        return RosDepthCamera(config.ros_namespace, config.max_frame_age)
    raise ValueError(f'Unknown camera source {config.camera_source!r}; expected one of {CAMERA_SOURCES}')
