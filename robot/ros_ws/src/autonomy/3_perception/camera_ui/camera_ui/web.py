"""HTTP side of the camera UI: the page, a status and camera-switch API, MJPEG streams and snapshots.

Standard library only (ThreadingHTTPServer, one thread per request) and independent of ROS:
`CameraUIApp` reads frames through a Renderer and switches cameras through a `select(camera)`
callable, which the node binds to the camera_hub services.

  GET  /                                   the page (web/index.html)
  GET  /api/status                         hub status, per-camera stream rates, depth statistics, intrinsics
  POST /api/select {"camera": name|null}   switch the hub to a camera; null releases it (application/json only)
  GET  /api/depth_at?camera=&u=&v=         depth (mm) and optical-frame XYZ (mm) at normalized coordinates
  GET  /stream/<cam>/rgb.mjpg              rectified left image, or the raw JPEG while no depth node runs
  GET  /stream/<cam>/depth.mjpg?near=&far=&auto=1   TURBO depth: near red, far blue, invalid black
  GET  /snapshot/<cam>/rgb.png, /snapshot/<cam>/depth.png (16-bit, mm)
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import itertools
import json
from pathlib import Path
import select
import threading
import time
from urllib.parse import parse_qs, urlsplit

PAGE = (Path(__file__).resolve().parent / 'web' / 'index.html').read_bytes()
DEFAULT_RANGE_MM = (100.0, 1000.0)   # depth stream range with auto=0 and no near/far


class CameraUIApp:
    """State behind the HTTP API: frames (via `renderer`), the last hub status and camera switches.

    `select(camera)` switches the hub (camera None releases it) and returns (HTTP status,
    {'ok': bool, 'message': str}); None means no switching is possible.
    """

    def __init__(self, renderer, labels, select=None, max_fps=15.0):
        self.renderer = renderer
        self.store = renderer.store
        self.cameras = self.store.cameras
        self.labels = {camera: labels.get(camera, camera) for camera in self.cameras}
        self.max_fps = float(max_fps)
        self.closed = threading.Event()   # set on shutdown; ends the MJPEG streams
        self._select = select
        self._lock = threading.Lock()
        self._hub = self._hub_received = None
        self._switch_ids = itertools.count()
        self._switching = None   # (id, camera, start) of the newest switch still waiting for its reply

    def set_hub_status(self, text):
        """Keep a camera_hub status message (JSON object text); raises ValueError for anything else."""
        status = json.loads(text)
        if not isinstance(status, dict):
            raise ValueError('camera hub status is not a JSON object')
        with self._lock:
            self._hub, self._hub_received = status, self.store.clock()

    def status(self):
        now = self.store.clock()
        with self._lock:
            hub, received, switching = self._hub, self._hub_received, self._switching
        return {
            'hub': hub,
            'hub_age_s': None if received is None else round(now - received, 3),
            'switching': None if switching is None else {'camera': switching[1], 'elapsed_s': round(now - switching[2], 1)},
            'stale_after': self.store.stale_after,
            'cameras': {camera: {'label': self.labels[camera], **self.renderer.camera_status(camera)}
                        for camera in self.cameras},
        }

    def select(self, camera):
        """(HTTP status, {'ok', 'message'}) of switching the hub to `camera`; None releases every camera.

        Switches are not queued here: a newer selection or a release goes straight to the hub,
        which supersedes the switch still waiting for its camera (that one then fails with
        "selection changed"). So a camera that never delivers cannot lock the others out, and
        `switching` in the status is the newest switch still waiting.
        """
        if camera is not None and camera not in self.cameras:
            return 400, {'ok': False, 'message': f'unknown camera {camera!r}; expected one of {list(self.cameras)}'}
        if self._select is None:
            return 503, {'ok': False, 'message': 'camera switching is not available'}
        with self._lock:
            switch = self._switching = (next(self._switch_ids), camera, self.store.clock())
        try:
            return self._select(camera)
        finally:
            with self._lock:
                if self._switching is switch:   # a newer one keeps showing until it ends
                    self._switching = None

    def close(self):
        self.closed.set()


def depth_range_query(query):
    """(near, far) mm from ?near=&far=&auto= (None, None = auto range); raises ValueError on bad values.

    Without near/far, auto is on unless auto=0 is given.
    """
    def value(name, default):
        return query.get(name, [default])[0]
    manual = 'near' in query or 'far' in query
    if value('auto', '0' if manual else '1').lower() in ('1', 'true', 'on', 'yes'):
        return None, None
    near, far = float(value('near', DEFAULT_RANGE_MM[0])), float(value('far', DEFAULT_RANGE_MM[1]))
    if not 0.0 <= near < far <= 65535.0:
        raise ValueError(f'need 0 <= near < far <= 65535 mm, got near={near:g} far={far:g}')
    return near, far


class UIHandler(BaseHTTPRequestHandler):
    app: CameraUIApp = None   # set on a per-server subclass by make_server()
    server_version = 'camera-ui/1'
    timeout = 30   # socket timeout: a stalled client cannot pin a handler thread forever

    def log_message(self, format, *args):   # keep the node's log for the node
        pass

    def _send(self, body, content_type, status=200, headers=()):
        try:
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for name, value in headers:
                self.send_header(name, value)
            self.end_headers()
            self.wfile.write(body)
        except OSError:
            return   # the client went away

    def _send_json(self, payload, status=200):
        self._send(json.dumps(payload).encode(), 'application/json', status)

    def do_GET(self):
        url = urlsplit(self.path)
        query = parse_qs(url.query)
        parts = url.path.strip('/').split('/')
        if url.path == '/':
            self._send(PAGE, 'text/html; charset=utf-8')
        elif url.path == '/api/status':
            self._send_json(self.app.status())
        elif url.path == '/api/depth_at':
            self._depth_at(query)
        elif len(parts) == 3 and parts[0] in ('stream', 'snapshot'):
            self._camera_route(*parts, query)
        else:
            self._send_json({'error': f'not found: {url.path}'}, 404)

    def do_POST(self):
        if urlsplit(self.path).path != '/api/select':
            self._send_json({'error': 'not found'}, 404)
            return
        # Same-origin JSON only. Any web page can make a browser POST text/plain or a form here without a
        # CORS preflight; application/json needs one, which this server never grants, and the browser
        # names the requesting page in Origin.
        origin = self.headers.get('Origin')
        if origin is not None and urlsplit(origin).netloc != self.headers.get('Host'):
            self._send_json({'ok': False, 'message': f'cross-origin request from {origin} refused'}, 403)
            return
        if self.headers.get_content_type() != 'application/json':
            self._send_json({'ok': False, 'message': 'send the body as Content-Type: application/json'}, 415)
            return
        try:
            length = int(self.headers.get('Content-Length') or 0)
            if not 0 < length <= 65536:
                raise ValueError('expected a JSON body {"camera": name|null}')
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict) or 'camera' not in body:
                raise ValueError('expected a JSON body {"camera": name|null}')
        except ValueError as exc:   # json.JSONDecodeError is a ValueError
            self._send_json({'ok': False, 'message': str(exc)}, 400)
            return
        camera = body['camera']
        status, payload = self.app.select(None if camera in (None, 'none') else camera)
        self._send_json(payload, status)

    def _depth_at(self, query):
        camera = query.get('camera', [''])[0]
        try:
            if camera not in self.app.cameras:
                raise ValueError(f'unknown camera {camera!r}')
            u, v = (float(query.get(name, ['nan'])[0]) for name in ('u', 'v'))
            if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
                raise ValueError('u and v must be normalized image coordinates in [0, 1]')
        except ValueError as exc:
            self._send_json({'error': str(exc)}, 400)
            return
        self._send_json(self.app.renderer.depth_at(camera, u, v))

    def _camera_route(self, kind, camera, name, query):
        renderer = self.app.renderer
        if camera not in self.app.cameras:
            self._send_json({'error': f'unknown camera {camera!r}'}, 404)
        elif (kind, name) == ('stream', 'rgb.mjpg'):
            self._stream(lambda: renderer.rgb_jpeg(camera)[:2])
        elif (kind, name) == ('stream', 'depth.mjpg'):
            try:
                near, far = depth_range_query(query)
            except ValueError as exc:
                self._send_json({'error': str(exc)}, 400)
                return
            self._stream(lambda: renderer.depth_jpeg(camera, near, far))
        elif kind == 'snapshot' and name in ('rgb.png', 'depth.png'):
            stream = name[:-len('.png')]
            frame, png = renderer.rgb_png(camera) if stream == 'rgb' else renderer.depth_png(camera)
            if frame is None:
                self._send_json({'error': f'no {stream} frame from {camera} yet'}, 503)
                return
            filename = f'{camera}_{stream}_{frame.stamp_ns}.png'
            self._send(png, 'image/png', headers=[('Content-Disposition', f'attachment; filename="{filename}"')])
        else:
            self._send_json({'error': f'not found: {self.path}'}, 404)

    def _client_gone(self, timeout):
        # An MJPEG client never sends more bytes, so a readable socket means EOF/RST (or garbage).
        readable, _, _ = select.select([self.connection], [], [], timeout)
        return bool(readable)

    def _stream(self, latest):
        """MJPEG of latest() -> (seq, jpeg): each new frame once, at most max_fps, until the client leaves."""
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        period = 1.0 / self.app.max_fps if self.app.max_fps > 0 else 0.0
        last_seq, next_send = None, 0.0
        try:
            while not self.app.closed.is_set():
                wait = next_send - time.monotonic()
                if wait > 0:   # rate limit before rendering, so skipped frames are never encoded
                    if self._client_gone(min(wait, 0.1)):
                        return
                    continue
                seq, jpeg = latest()
                if jpeg is None or seq == last_seq:
                    if self._client_gone(0.02):
                        return
                    continue
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(jpeg)
                                 + jpeg + b'\r\n')
                self.wfile.flush()
                last_seq, next_send = seq, time.monotonic() + period
        except OSError:   # broken pipe, reset, socket timeout
            return


def make_server(app, host='0.0.0.0', port=8001):
    handler = type('BoundUIHandler', (UIHandler,), {'app': app})
    server = ThreadingHTTPServer((host, port), handler)
    server.daemon_threads = True
    return server
