"""HTTP side of the image stream server: MJPEG streams and a health report (stdlib only, no ROS).

  GET /stream/<kind>/<camera>.mjpg   the stream's newest frames as multipart/x-mixed-replace JPEG
  GET /health                        per stream: topic, subscribed, viewers, last_frame_age_s, error

A stream counts as watched (StreamHub.acquire) for as long as its HTTP response is open.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import select
import threading
import time
from urllib.parse import urlsplit


class StreamHandler(BaseHTTPRequestHandler):
    hub = None          # set on a per-server subclass by make_server()
    max_fps = 15.0
    closed = None       # threading.Event; set on shutdown, ends the streams
    server_version = 'image-stream-server/1'
    timeout = 30        # socket timeout: a stalled client cannot pin a handler thread forever

    def log_message(self, format, *args):   # keep the node's log for the node
        pass

    def _send_json(self, payload, status=200):
        body = json.dumps(payload).encode()
        try:
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Access-Control-Allow-Origin', '*')   # read-only; lets the GCS page poll it
            self.end_headers()
            self.wfile.write(body)
        except OSError:
            return   # the client went away

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/health':
            self._send_json(self.hub.health())
            return
        parts = path.strip('/').split('/')
        if len(parts) == 3 and parts[0] == 'stream' and parts[2].endswith('.mjpg'):
            key = (parts[1], parts[2][:-len('.mjpg')])
            if key in self.hub.streams:
                self._stream(key)
                return
        self._send_json({'error': f'not found: {path}'}, 404)

    def _client_gone(self, timeout):
        # An MJPEG client never sends more bytes, so a readable socket means EOF/RST (or garbage).
        readable, _, _ = select.select([self.connection], [], [], timeout)
        return bool(readable)

    def _stream(self, key):
        """Each new frame once, at most max_fps, until the client leaves or the server closes."""
        self.hub.acquire(key)
        try:
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            period = 1.0 / self.max_fps if self.max_fps > 0 else 0.0
            last_seq, next_send = None, 0.0
            while not self.closed.is_set():
                wait = next_send - time.monotonic()
                if wait > 0:   # rate limit before encoding, so skipped frames are never encoded
                    if self._client_gone(min(wait, 0.1)):
                        return
                    continue
                seq, jpeg = self.hub.latest_jpeg(key)
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
        finally:
            self.hub.release(key)


def make_server(hub, host='0.0.0.0', port=8002, max_fps=15.0):
    closed = threading.Event()
    handler = type('BoundStreamHandler', (StreamHandler,), {'hub': hub, 'max_fps': float(max_fps), 'closed': closed})
    server = ThreadingHTTPServer((host, port), handler)
    server.daemon_threads = True
    server.closed = closed
    return server
