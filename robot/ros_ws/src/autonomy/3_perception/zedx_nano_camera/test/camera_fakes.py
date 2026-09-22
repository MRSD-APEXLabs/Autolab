"""Stand-ins for the Xavier servers: the Nano-only stream server (scripts/zedx_nano_v4l2_stream.py),
in memory, and the camera hub (2_manipulation/xavier_camera_hub), over HTTP on localhost."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import threading
import time
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np

from zedx_nano_camera import stream

CONF = """[LEFT_CAM_FHD1200]
fx=950
fy=950
cx=960
cy=600
k1=0.04
k2=-0.03
p1=0.001
p2=-0.001
k3=-0.03

[RIGHT_CAM_FHD1200]
fx=951
fy=951
cx=948
cy=601
k1=0.02
k2=-0.03
p1=0
p2=0
k3=0

[STEREO]
Baseline=18.01
TY=-0.04
TZ=0.19
CV_FHD1200=0.0005
RX_FHD1200=-0.0002
RZ_FHD1200=-0.00006

[MISC]
Sensor_ID=1
"""
INFO = {'server_version': 4, 'model': 'ZED X Nano', 'serial': 99292912,
        'capture': {'width': 1920, 'height': 1200, 'fps': 30}, 'output': {'width': 960, 'height': 600},
        'sensors': {'left': '/dev/video3', 'right': '/dev/video2'}, 'stereo': {'available': True}}
# The ZED X streams its SVGA mode unscaled, so its file needs the SVGA sections.
ZEDX_CONF = """[LEFT_CAM_SVGA]
fx=367.5
fy=367.4
cx=482.1
cy=301.3
k1=0
k2=0
p1=0
p2=0
k3=0

[RIGHT_CAM_SVGA]
fx=367.9
fy=367.8
cx=479.6
cy=299.8
k1=0
k2=0
p1=0
p2=0
k3=0

[STEREO]
Baseline=119.87
TY=0.02
TZ=-0.31
CV_SVGA=0.001
RX_SVGA=0.0004
RZ_SVGA=-0.0002
"""
ZEDX_INFO = {'server_version': 4, 'model': 'ZED X', 'serial': 42757821,
             'capture': {'width': 960, 'height': 600, 'fps': 60, 'pixel_format': 'zed_sdk'},
             'output': {'width': 960, 'height': 600, 'format': 'jpeg'},
             'sensors': {'left': 'zed_sdk', 'right': 'zed_sdk'}, 'stereo': {'available': True, 'tolerance_us': 0}}
HUB_CAMERAS = {'zedx': (ZEDX_INFO, ZEDX_CONF), 'zedx_nano': (INFO, CONF)}


def jpeg(bgr=None):
    if bgr is None:
        bgr = np.zeros((6, 8, 3), np.uint8)
        bgr[:, :, 2] = 255
    return cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes()


def part(frame_id=1, capture_ns=1_050_000_000, ready_ns=1_060_000_000, body=None, capture_mono_ns=None):
    body = jpeg() if body is None else body
    mono = b'' if capture_mono_ns is None else b'X-Capture-Mono-Ns: %d\r\n' % capture_mono_ns
    head = (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\nX-Frame-Id: %d\r\n'
            b'X-Capture-Ns: %d\r\nX-Ready-Ns: %d\r\n%sX-Sensor: left\r\n\r\n' % (len(body), frame_id, capture_ns, ready_ns, mono))
    return head + body + b'\r\n'


def stereo_part(frame_id, left_jpeg, right_jpeg, capture_ns=None, capture_mono_ns=None):
    capture_ns = time.time_ns() if capture_ns is None else capture_ns
    capture_mono_ns = time.monotonic_ns() if capture_mono_ns is None else capture_mono_ns
    body = left_jpeg + right_jpeg
    head = (b'--frame\r\nContent-Type: application/octet-stream\r\nContent-Length: %d\r\nX-Frame-Id: %d\r\n'
            b'X-Capture-Ns: %d\r\nX-Ready-Ns: %d\r\nX-Capture-Mono-Ns: %d\r\nX-Sensor: stereo\r\nX-Left-Length: %d\r\n'
            b'X-Right-Frame-Id: %d\r\nX-Sync-Us: 120\r\n\r\n'
            % (len(body), frame_id, capture_ns, capture_ns + 1000, capture_mono_ns, len(left_jpeg), frame_id + 2))
    return head + body + b'\r\n'


class BlockingStream:
    """Serves bytes, then blocks like a live socket until close() instead of reporting EOF."""

    def __init__(self, data, content_type='multipart/x-mixed-replace; boundary=frame'):
        self.buffer = io.BytesIO(data)
        self.closed = threading.Event()
        self.headers = {'Content-Type': content_type}

    def _wait_eof(self):
        self.closed.wait(5)
        return b''

    def readline(self):
        line = self.buffer.readline()
        return line if line else self._wait_eof()

    def read(self, n):
        data = self.buffer.read(n)
        return data if data else self._wait_eof()

    def close(self):
        self.closed.set()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def fake_server(monkeypatch, pairs=((b'L', b'R'),), info=None, conf=CONF, calibration_status=200, gate=None):
    """Route the stream module's HTTP opener to an in-memory server; returns the list of requested URLs.

    `pairs` are (left_jpeg, right_jpeg) served once on /video_feed/stereo, stamped when the feed opens,
    which waits for the threading.Event `gate` if one is given.
    """
    info = dict(INFO, **(info or {}))
    requests = []

    def urlopen(url, timeout=None):
        requests.append(url)
        path = url.split('//', 1)[1].split('/', 1)[1]
        if path == 'info':
            return io.BytesIO(json.dumps(info).encode())
        if path == 'time':
            return io.BytesIO(json.dumps({'t_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns()}).encode())
        if path == 'calibration.conf':
            if calibration_status != 200:
                raise stream.urllib.error.HTTPError(url, calibration_status, 'missing', {}, None)
            return io.BytesIO(conf.encode())
        if path == 'video_feed/stereo':
            if gate is not None:
                gate.wait(10)
            return BlockingStream(b''.join(stereo_part(i + 1, lj, rj) for i, (lj, rj) in enumerate(pairs)))
        if path == 'video_feed/html':
            return BlockingStream(b'<html>', content_type='text/html')
        raise AssertionError(url)
    monkeypatch.setattr(stream, '_open', urlopen)
    return requests


class FakeHub:
    """The camera hub's HTTP API on 127.0.0.1 with synthetic cameras; one camera streams at a time.

    Routes: /status, POST /select, /time, /cameras/<name>/{info,calibration.conf,video_feed/stereo}
    and the legacy root routes, which serve the Nano. An inactive camera's feed (and the legacy
    routes while the Nano is inactive) answer 503; a camera's open feeds end when another one is
    selected. Selecting a camera in `failing` answers 502 and leaves it selected in state error.
    A select waits `select_delay` s before answering, and 409 if another select came in meanwhile.
    `pairs` holds each camera's (left_jpeg, right_jpeg). `requests` records (method, path) of every
    request, `selects` the query of every POST /select.
    """

    def __init__(self, active='zedx_nano', period=0.05, select_delay=0.0, failing=()):
        self.active, self.period, self.select_delay, self.failing = active, period, select_delay, set(failing)
        self.error = None
        self.frame_ids = {name: 0 for name in HUB_CAMERAS}
        self.switch_count = self.generation = 0
        self.requests = []
        self.selects = []   # query of each POST /select
        self.closed = threading.Event()
        self.pairs = {name: (jpeg(np.full((6, 8, 3), color, np.uint8)), jpeg(np.full((6, 8, 3), color[::-1], np.uint8)))
                      for name, color in (('zedx', (255, 0, 0)), ('zedx_nano', (0, 0, 255)))}
        hub = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                hub._get(self)

            def do_POST(self):
                hub._post(self)

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, args=(0.05,), name='fake-hub', daemon=True).start()

    def close(self):
        self.closed.set()
        self.server.shutdown()
        self.server.server_close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def paths(self, method='GET'):
        return [path for m, path in list(self.requests) if m == method]

    def select(self, camera):
        """Switch cameras like POST /select; returns (http status, reply)."""
        if camera not in HUB_CAMERAS and camera is not None:
            return 400, {'ok': False, 'error': f'unknown camera {camera!r}', 'status': self.status()}
        if camera != self.active or self.error:
            self.switch_count += 1
        self.active, self.error = camera, None
        self.generation += 1
        generation = self.generation
        time.sleep(self.select_delay)
        if self.generation != generation:
            return 409, {'ok': False, 'error': f'selection changed to {self.active} while waiting', 'status': self.status()}
        if camera in self.failing:
            self.error = f'{camera} failed to start'
            return 502, {'ok': False, 'error': self.error, 'status': self.status()}
        return 200, {'ok': True, 'status': self.status()}

    def status(self):
        state = 'idle' if self.active is None else 'error' if self.error else 'streaming'
        return {'server': 'camera_hub', 'hub_version': 1, 'active': self.active, 'state': state, 'error': self.error,
                'state_since_s': 1.0, 'switch_count': self.switch_count,
                'cameras': {name: {'model': info['model'], 'serial': info['serial'], 'active': name == self.active,
                                   'frames': self.frame_ids[name]} for name, (info, _) in HUB_CAMERAS.items()},
                't_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns()}

    # -- HTTP ---------------------------------------------------------------------------------
    @staticmethod
    def _reply(handler, code, body, content_type='application/json'):
        data = json.dumps(body).encode() if content_type == 'application/json' else body.encode()
        handler.send_response(code)
        handler.send_header('Content-Type', content_type)
        handler.send_header('Content-Length', str(len(data)))
        handler.end_headers()
        handler.wfile.write(data)

    def _post(self, handler):
        url = urlsplit(handler.path)
        self.requests.append(('POST', url.path))
        if url.path != '/select':
            return self._reply(handler, 404, {'error': 'not found'})
        body = json.loads(handler.rfile.read(int(handler.headers.get('Content-Length', 0))) or b'{}')
        camera = body.get('camera')
        self.selects.append(parse_qs(url.query))
        self._reply(handler, *self.select(None if camera == 'none' else camera))

    def _get(self, handler):
        path = urlsplit(handler.path).path
        self.requests.append(('GET', path))
        if path == '/time':
            return self._reply(handler, 200, {'t_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns()})
        if path == '/status':
            return self._reply(handler, 200, self.status())
        if path.startswith('/cameras/'):
            name, _, route = path[len('/cameras/'):].partition('/')
            if name not in HUB_CAMERAS:
                return self._reply(handler, 404, {'error': f'unknown camera {name!r}'})
            inactive = {'error': f'camera {name} is not active', 'camera': name, 'active': self.active}
        else:
            name, route = 'zedx_nano', path.lstrip('/')
            inactive = {'error': f'ZED X Nano is not the active camera (active: {self.active}); '
                                 f'select it at http://127.0.0.1:{self.port}/', 'camera': name, 'active': self.active}
        info, conf = HUB_CAMERAS[name]
        if route == 'calibration.conf':
            return self._reply(handler, 200, conf, 'text/plain')
        if route == 'info':
            if path == '/info' and self.active != name:
                return self._reply(handler, 503, inactive)
            return self._reply(handler, 200, dict(info, camera=name, active=self.active == name, hub_active=self.active,
                                                  calibration_available=True))
        if route == 'video_feed/stereo':
            if self.active != name:
                return self._reply(handler, 503, inactive)
            return self._stream(handler, name)
        self._reply(handler, 404, {'error': 'not found'})

    def _stream(self, handler, name):
        handler.send_response(200)
        handler.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        handler.end_headers()
        extra = b'X-Camera: %s\r\nX-Serial: %d\r\n' % (name.encode(), HUB_CAMERAS[name][0]['serial'])
        try:
            while self.active == name and not self.closed.is_set():   # deselected: end the feed
                self.frame_ids[name] += 1
                part = stereo_part(self.frame_ids[name], *self.pairs[name])
                head, _, body = part.partition(b'\r\n\r\n')
                handler.wfile.write(head + b'\r\n' + extra + b'\r\n' + body)
                handler.wfile.flush()
                time.sleep(self.period)
        except OSError:
            pass   # client went away
