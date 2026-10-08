"""Synthetic frames, a camera_hub status and HTTP helpers for the camera UI tests."""
from contextlib import contextmanager
import http.client
import threading

import cv2
import numpy as np

from camera_ui.web import make_server

W, H, FX = 960, 600, 480.0
INTRINSICS = {'fx': FX, 'fy': FX, 'cx': W / 2, 'cy': H / 2, 'width': W, 'height': H}


class FakeClock:
    def __init__(self, now=100.0):
        self.now = now

    def __call__(self):
        return self.now


def rgb_image():
    """BGR test card: horizontal colour ramp, so resizing or channel swaps show."""
    image = np.zeros((H, W, 3), np.uint8)
    image[:, :, 2] = np.linspace(0, 255, W, dtype=np.uint8)[None, :]
    image[:, :, 1] = np.linspace(0, 255, H, dtype=np.uint8)[:, None]
    image[:, :, 0] = 80
    return image


def depth_ramp(near=400, far=1600):
    """uint16 mm growing from `near` (left) to `far` (right), with an invalid (0) band on the top 60 rows."""
    depth = np.tile(np.linspace(near, far, W).astype(np.uint16), (H, 1))
    depth[:60] = 0
    return depth


def jpeg(bgr, quality=90):
    return cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()


def decode(data, flags=cv2.IMREAD_COLOR):
    image = cv2.imdecode(np.frombuffer(data, np.uint8), flags)
    assert image is not None
    return image


def hub_status(active='zedx_nano', state='streaming', error=None, reachable=True):
    """What the zedx_nano_camera camera_hub node publishes on /camera_hub/status."""
    cameras = {'zedx': {'model': 'ZED X', 'serial': 42757821, 'role': 'base', 'backend': 'zed_sdk'},
               'zedx_nano': {'model': 'ZED X Nano', 'serial': 99292912, 'role': 'wrist', 'backend': 'v4l2'}}
    for name, camera in cameras.items():
        camera.update(active=name == active, fps=29.9 if name == active else 0.0, frames=100, restarts=0, last_error='')
    return {'server': 'camera_hub', 'hub_version': 1, 'active': active, 'state': state, 'error': error,
            'state_since_s': 12.5, 'switch_count': 3, 'cameras': cameras, 't_ns': 0, 'monotonic_ns': 0,
            'reachable': reachable, 'poll_error': None if reachable else 'connection refused',
            'hub_url': 'http://192.168.1.101:8090'}


@contextmanager
def serving(app):
    """The app's HTTP server on an ephemeral localhost port; yields the port."""
    server = make_server(app, '127.0.0.1', 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server.server_port
    finally:
        app.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def request(port, method, path, body=None, timeout=10.0, headers=None):
    """(status, headers, body bytes); a body is sent as JSON unless `headers` say otherwise."""
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=timeout)
    try:
        if headers is None:
            headers = {'Content-Type': 'application/json'} if body is not None else {}
        conn.request(method, path, body=body, headers=headers)
        response = conn.getresponse()
        return response.status, dict(response.getheaders()), response.read()
    finally:
        conn.close()


def read_parts(response, count):
    """The first `count` JPEG bodies of an open multipart/x-mixed-replace response."""
    parts = []
    while len(parts) < count:
        line = response.fp.readline()
        if not line:
            raise EOFError('stream ended')
        if line.strip() != b'--frame':
            continue
        headers = {}
        while True:
            line = response.fp.readline().strip()
            if not line:
                break
            name, _, value = line.decode().partition(':')
            headers[name.strip().lower()] = value.strip()
        assert headers['content-type'] == 'image/jpeg'
        parts.append(response.fp.read(int(headers['content-length'])))
    return parts


@contextmanager
def open_stream(port, path, timeout=10.0):
    """An open MJPEG response; leaving the block closes the connection like a browser tab does."""
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=timeout)
    response = None
    try:
        conn.request('GET', path)
        response = conn.getresponse()
        assert response.status == 200, response.read()
        assert response.getheader('Content-Type') == 'multipart/x-mixed-replace; boundary=frame'
        yield response
    finally:
        if response is not None:
            response.close()   # the socket stays open while the response holds it
        conn.close()
