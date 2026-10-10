"""HTTP helpers for the server tests: serve a StreamHub, open MJPEG streams and read their parts."""
from contextlib import contextmanager
import http.client
import threading

from image_stream_server.web import make_server


@contextmanager
def serving(hub, max_fps=100.0):
    server = make_server(hub, '127.0.0.1', 0, max_fps=max_fps)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server.server_port
    finally:
        server.closed.set()
        server.shutdown()
        server.server_close()
        thread.join(2.0)


def request(port, path, timeout=5.0):
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=timeout)
    conn.request('GET', path)
    response = conn.getresponse()
    body = response.read()
    conn.close()
    return response.status, dict(response.getheaders()), body


@contextmanager
def open_stream(port, path, timeout=5.0):
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=timeout)
    conn.request('GET', path)
    response = conn.getresponse()
    try:
        yield response
    finally:
        response.close()   # the response holds the socket; closing only conn would leave it open
        conn.close()


def read_parts(response, count):
    """The first `count` JPEG payloads of a multipart/x-mixed-replace response."""
    parts = []
    while len(parts) < count:
        line = response.fp.readline()
        if not line:
            raise EOFError('stream ended')
        if line.lower().startswith(b'content-length:'):
            length = int(line.split(b':', 1)[1])
            response.fp.readline()   # blank line after the part headers
            parts.append(response.fp.read(length))
    return parts
