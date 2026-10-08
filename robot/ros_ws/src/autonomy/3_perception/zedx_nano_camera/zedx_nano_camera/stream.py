"""Client for the Xavier stereo MJPEG servers.

Two servers speak this protocol: the camera hub (2_manipulation/xavier_camera_hub), which serves
the ZED X or the ZED X Nano, one at a time, under /cameras/<name>/, and the older Nano-only
server (scripts/zedx_nano_v4l2_stream.py in the teleop repo), whose root API the hub keeps for
the Nano. Both serve unrectified JPEGs, one stream per sensor or synchronized left+right pairs on
video_feed/stereo. Every part carries the Xavier capture stamps; `NanoStreamClient.synchronize_clock()`
estimates the Xavier clocks relative to the local ones so capture times can be mapped to this
machine. The Stereolabs factory calibration is fetched from the server (or calib.stereolabs.com).
A camera the hub is not streaming raises `CameraInactive`; `CameraHubClient` switches cameras.
"""
from __future__ import annotations

import json
import socket
import time
import urllib.error
import urllib.request

import cv2
import numpy as np

from .calibration import build_calibration, parse_calibration_text

CALIBRATION_URL = 'https://calib.stereolabs.com/?SN={}'
DEFAULT_CAMERA = 'zedx_nano'   # what the root API (camera '') serves
DEFAULT_MODEL = 'ZED X Nano'
MAX_PART_BYTES = 32_000_000
MIN_SERVER_VERSION = 2
# Requests to the Xavier bypass any http_proxy from the environment (the old Camera-Edge
# client did the same with http_no_proxy); the calibration server download stays proxy-aware.
_DIRECT_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def _open(url, timeout):
    return _DIRECT_OPENER.open(url, timeout=timeout)


class CameraInactive(RuntimeError):
    """The hub is streaming another camera or none; `active` is the hub's active camera (None: no camera)."""

    def __init__(self, message, active=None):
        super().__init__(message)
        self.active = active


def _error_reply(exc):
    """JSON body of an HTTPError from the hub ({} when it has none)."""
    try:
        reply = json.loads(exc.read())
    except Exception:
        return {}
    return reply if isinstance(reply, dict) else {}


def _inactive(exc, camera):
    """CameraInactive from the hub's 503 reply ({"error": ..., "active": <hub's active camera>})."""
    reply = _error_reply(exc)
    return CameraInactive(reply.get('error') or f'{camera} is not the active camera', reply.get('active'))


def shutdown_response(response):
    """Wake a reader blocked in recv() (close() alone waits for the buffer lock, i.e. the socket timeout)."""
    try:
        response.fp.raw._sock.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass
    try:
        response.close()
    except Exception:
        pass


def iter_mjpeg_parts(stream, max_part=MAX_PART_BYTES):
    """Yield (headers, body) for every part of a multipart/x-mixed-replace stream.

    `stream` needs readline() and read(n) (an http.client.HTTPResponse works). The
    generator ends when the stream ends. Parts must carry Content-Length: this client
    never scans bodies for boundaries, so a missing or invalid length is an error.
    """
    while True:
        line = stream.readline()
        if not line:
            return
        if not line.startswith(b'--'):
            continue
        headers = {}
        while True:
            line = stream.readline()
            if not line:
                return
            if line in (b'\r\n', b'\n'):
                break
            key, sep, value = line.decode('latin-1').partition(':')
            if sep:
                headers[key.strip().lower()] = value.strip()
        try:
            length = int(headers.get('content-length', ''))
        except ValueError:
            raise ValueError('MJPEG part without Content-Length') from None
        if not 0 < length <= max_part:
            raise ValueError('MJPEG part has an invalid Content-Length')
        body = stream.read(length)
        if len(body) < length:
            return
        yield headers, body


def decode_jpeg(body):
    bgr = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError('Invalid JPEG frame')
    return bgr


def split_stereo_part(headers, body):
    """(left_jpeg, right_jpeg) of a /video_feed/stereo part (left JPEG followed by right JPEG)."""
    try:
        left_length = int(headers['x-left-length'])
    except (KeyError, ValueError):
        raise ValueError('Stereo part is missing X-Left-Length') from None
    if not 0 < left_length < len(body):
        raise ValueError('Invalid X-Left-Length')
    return body[:left_length], body[left_length:]


def frame_header(headers, width, height):
    """Metadata of one stream part: frame id, Xavier capture stamps and, for pairs, the right frame id."""
    try:
        frame_id = int(headers['x-frame-id'])
        t_image_ns = int(headers['x-capture-ns'])
        t_ready_ns = int(headers['x-ready-ns'])
        capture_mono_ns = int(headers['x-capture-mono-ns']) if 'x-capture-mono-ns' in headers else None
    except (KeyError, ValueError):
        raise ValueError('MJPEG part is missing frame metadata headers') from None
    if min(frame_id, t_image_ns, t_ready_ns) < 0 or (capture_mono_ns is not None and capture_mono_ns < 0):
        raise ValueError('Invalid frame metadata')
    header = {'type': 'nano_stream_frame', 'frame_id': frame_id, 't_image_ns': t_image_ns,
              't_grab_done_ns': t_ready_ns, 't_pub_ns': t_ready_ns, 'image_size': [width, height],
              'sensor': headers.get('x-sensor', ''), 'capture_mono_ns': capture_mono_ns}
    if 'x-right-frame-id' in headers:
        header['right_frame_id'] = int(headers['x-right-frame-id'])
        header['stereo_sync_us'] = int(headers.get('x-sync-us', 0))
    return header


def captured_monotonic(header, received_ns, received_monotonic, offset_ns, mono_offset_ns=None):
    """Capture time of a part on the local monotonic clock (seconds).

    Uses the Xavier's CLOCK_MONOTONIC stamp when the server provides one (immune to wall-clock
    steps on either side), else the wall-clock stamp and `offset_ns` like the Camera-Edge client.
    """
    if header['capture_mono_ns'] is not None and mono_offset_ns is not None:
        # mono_offset is Xavier CLOCK_MONOTONIC minus local monotonic (ns).
        return (header['capture_mono_ns'] - mono_offset_ns) / 1e9
    # offset is Xavier wall-clock minus local wall-clock. Anchor to local monotonic.
    return received_monotonic - (received_ns + offset_ns - header['t_image_ns']) / 1e9


def fetch_calibration_text(get, serial, timeout=15):
    """Calibration .conf text from the Xavier (`get(path)` -> bytes) or from calib.stereolabs.com."""
    try:
        return get('/calibration.conf').decode('utf-8', 'replace'), 'xavier:/calibration.conf'
    except urllib.error.HTTPError as exc:
        if exc.code != 404:
            raise
    if serial:
        with urllib.request.urlopen(CALIBRATION_URL.format(serial), timeout=timeout) as response:
            return response.read().decode('utf-8', 'replace'), 'calib.stereolabs.com'
    raise RuntimeError('No factory calibration for the camera; start the Xavier server with --serial')


class NanoStreamClient:
    """HTTP access to one Xavier camera: /info, clock sync, calibration and MJPEG feeds.

    `camera` names a camera of the hub ('zedx', 'zedx_nano'); '' uses the root API, which is the
    Nano on both the hub and the old Nano-only server.
    """

    def __init__(self, host, port=8090, timeout=3.0, camera=''):
        self.host, self.port, self.timeout, self.camera = host, int(port), timeout, camera
        self.name = camera or DEFAULT_CAMERA

    def path(self, path):
        """Server path of `path` for this camera (/time is shared by all cameras)."""
        return f'/cameras/{self.camera}{path}' if self.camera and path != '/time' else path

    def url(self, path):
        return f'http://{self.host}:{self.port}{self.path(path)}'

    def get(self, path):
        with _open(self.url(path), self.timeout) as response:
            return response.read()

    def get_json(self, path):
        return json.loads(self.get(path))

    def fetch_info(self, sensor='stereo'):
        """Server /info, checked for a compatible version, for the requested feed and that the camera is active."""
        try:
            info = self.get_json('/info')
        except urllib.error.HTTPError as exc:
            if exc.code == 503:
                raise _inactive(exc, self.name) from None
            if exc.code == 404 and self.camera:
                raise RuntimeError(f'The Xavier server has no camera {self.camera!r}; is the camera hub running '
                                   '(the old Nano-only server has none)?') from None
            raise
        if int(info.get('server_version', 0)) < MIN_SERVER_VERSION:
            raise RuntimeError('Xavier Nano stream server is too old; redeploy scripts/zedx_nano_v4l2_stream.py')
        if info.get('active') is False:
            active = info.get('hub_active')
            raise CameraInactive(f'{self.name} is not the active camera (hub active: {active or "none"})', active)
        if sensor == 'stereo':
            if not (info.get('stereo') or {}).get('available'):
                raise RuntimeError('Xavier Nano stream server has no stereo feed; redeploy scripts/zedx_nano_v4l2_stream.py '
                                   'with both sensors (--device /dev/video3,/dev/video2)')
        elif sensor not in info.get('sensors', {}):
            raise RuntimeError(f'{self.name} stream has no sensor {sensor!r}: {sorted(info.get("sensors", {}))}')
        return info

    def synchronize_clock(self, samples=7):
        """Minimum round-trip estimate of the Xavier clocks relative to the local ones.

        Returns (wall_offset_ns, uncertainty_ns, mono_offset_ns); mono_offset_ns is None when
        the server does not report its monotonic clock.
        """
        results = []
        for _ in range(samples):
            t0, m0 = time.time_ns(), time.monotonic_ns()
            reply = self.get_json('/time')
            elapsed = time.monotonic_ns() - m0
            midpoint_wall, midpoint_mono = t0 + elapsed // 2, m0 + elapsed // 2
            remote_mono = reply.get('monotonic_ns')
            results.append((elapsed, int(reply['t_ns']) - midpoint_wall,
                            None if remote_mono is None else int(remote_mono) - midpoint_mono))
        rtt, offset, mono_offset = min(results, key=lambda r: r[0])
        return offset, rtt // 2, mono_offset

    def load_calibration(self, info):
        """(parsed .conf, unrectified calibration scaled to the streamed size) for the server's camera."""
        serial = int(info.get('serial') or 0)
        text, source = fetch_calibration_text(self.get, serial)
        conf = parse_calibration_text(text, source)
        return conf, build_calibration(conf, capture_size(info), output_size(info), serial, source,
                                       model=info.get('model') or DEFAULT_MODEL)

    def open_feed(self, sensor='stereo'):
        """Open video_feed/<sensor>; read it with iter_mjpeg_parts(), stop it with shutdown_response()."""
        try:
            response = _open(self.url(f'/video_feed/{sensor}'), self.timeout)
        except urllib.error.HTTPError as exc:
            if exc.code == 503:
                raise _inactive(exc, self.name) from None
            raise
        content_type = response.headers.get('Content-Type', '')
        if 'multipart' not in content_type:
            shutdown_response(response)
            raise RuntimeError(f'Unexpected stream content type {content_type!r}')
        return response


class CameraHubClient:
    """Control API of the Xavier camera hub: GET /status and POST /select (which camera streams)."""

    def __init__(self, host, port=8090, timeout=3.0):
        self.host, self.port, self.timeout = host, int(port), timeout

    def url(self, path):
        return f'http://{self.host}:{self.port}{path}'

    def _not_a_hub(self, exc):
        if exc.code in (404, 405):
            return RuntimeError(f'{self.url("/")} has no camera hub API ({exc}); is the old Nano-only server running '
                                'instead of the camera hub?')
        return exc

    def status(self):
        try:
            with _open(self.url('/status'), self.timeout) as response:
                return json.loads(response.read())
        except urllib.error.HTTPError as exc:
            raise self._not_a_hub(exc) from None

    def select(self, camera, timeout=30.0):
        """Make `camera` the streaming camera (None releases all) and wait up to `timeout` s for its first frame.

        Returns the hub's reply {"ok": bool, "error": str (when not ok), "status": {...}}. A start
        failure or timeout is a reply with ok false (the hub keeps retrying the camera), not an exception.
        """
        request = urllib.request.Request(self.url(f'/select?timeout={timeout:g}'), method='POST',
                                         data=json.dumps({'camera': camera}).encode(),
                                         headers={'Content-Type': 'application/json'})
        try:
            # the hub answers after the start completes or `timeout` expires, so allow for both
            with _open(request, timeout + self.timeout) as response:
                return json.loads(response.read())
        except urllib.error.HTTPError as exc:
            reply = _error_reply(exc)
            if 'ok' not in reply:
                raise self._not_a_hub(exc) from None
            return reply


def capture_size(info):
    return info['capture']['width'], info['capture']['height']


def output_size(info):
    return info['output']['width'], info['output']['height']
