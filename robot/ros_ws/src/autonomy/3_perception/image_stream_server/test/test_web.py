"""HTTP routes and MJPEG streams of the image stream server against a StreamHub with a fake encoder (no ROS)."""
import json
import threading
import time

from http_fakes import open_stream, read_parts, request, serving
from image_stream_server.streams import StreamHub

KEY = ('annotated', 'zedx')
STREAMS = {KEY: '/inspect/annotated_image', ('raw', 'zedx'): '/zedx/left/image_rect_color'}


def make_hub():
    return StreamHub(STREAMS, lambda message: b'JPEG' + message.encode())


def wait_until(condition, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.01)
    return condition()


def test_health_lists_every_stream_with_cors():
    hub = make_hub()
    with serving(hub) as port:
        status, headers, body = request(port, '/health')
    assert status == 200
    assert headers['Content-Type'] == 'application/json'
    assert headers['Access-Control-Allow-Origin'] == '*'
    assert set(json.loads(body)) == {'annotated/zedx', 'raw/zedx'}


def test_unknown_paths_are_404():
    with serving(make_hub()) as port:
        for path in ('/', '/stream/annotated/zedx_nano.mjpg', '/stream/annotated/zedx', '/stream/x/zedx.mjpg',
                     '/stream/annotated/zedx.mjpg/extra'):
            assert request(port, path)[0] == 404, path


def test_stream_acquires_while_open_and_sends_new_frames():
    hub = make_hub()
    with serving(hub) as port:
        with open_stream(port, '/stream/annotated/zedx.mjpg') as response:
            assert response.status == 200
            assert response.getheader('Content-Type') == 'multipart/x-mixed-replace; boundary=frame'
            assert wait_until(lambda: hub.wanted() == {KEY})
            hub.update(KEY, 'one')
            assert read_parts(response, 1) == [b'JPEGone']
            hub.update(KEY, 'two')
            assert read_parts(response, 1) == [b'JPEGtwo']
        assert wait_until(lambda: hub.wanted() == set())   # client left -> released


def test_two_viewers_share_the_stream_and_release_independently():
    hub = make_hub()
    with serving(hub) as port:
        with open_stream(port, '/stream/annotated/zedx.mjpg') as first:
            with open_stream(port, '/stream/annotated/zedx.mjpg') as second:
                assert wait_until(lambda: hub.health()['annotated/zedx']['viewers'] == 2)
                hub.update(KEY, 'a')
                assert read_parts(first, 1) == [b'JPEGa'] and read_parts(second, 1) == [b'JPEGa']
            assert wait_until(lambda: hub.health()['annotated/zedx']['viewers'] == 1)
            assert hub.wanted() == {KEY}
        assert wait_until(lambda: hub.wanted() == set())


def test_failed_encodes_are_skipped_not_fatal():
    calls = []

    def encode(message):
        calls.append(message)
        if message == 'bad':
            raise ValueError('nope')
        return b'ok'
    hub = StreamHub(STREAMS, encode)
    with serving(hub) as port:
        with open_stream(port, '/stream/annotated/zedx.mjpg') as response:
            assert wait_until(lambda: hub.wanted() == {KEY})
            hub.update(KEY, 'bad')
            assert wait_until(lambda: 'bad' in calls)
            hub.update(KEY, 'good')
            assert read_parts(response, 1) == [b'ok']


def test_server_close_ends_open_streams():
    hub = make_hub()
    done = threading.Event()
    with serving(hub) as port:
        def watch():
            with open_stream(port, '/stream/annotated/zedx.mjpg') as response:
                response.read()   # returns when the server ends the stream
            done.set()
        threading.Thread(target=watch, daemon=True).start()
        assert wait_until(lambda: hub.wanted() == {KEY})
    assert done.wait(3.0)
