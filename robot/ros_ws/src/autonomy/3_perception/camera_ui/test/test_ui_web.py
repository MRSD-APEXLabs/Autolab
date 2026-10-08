"""The HTTP API and MJPEG streams of the camera UI against synthetic frames (no ROS)."""
import json
import threading
import time

import cv2
import numpy as np
import pytest

from camera_ui.frames import FrameStore, Renderer
from camera_ui.web import CameraUIApp, depth_range_query
from ui_fakes import H, INTRINSICS, W, decode, depth_ramp, hub_status, jpeg, open_stream, read_parts, request, rgb_image, serving

LABELS = {'zedx': 'ZED X (base)', 'zedx_nano': 'ZED X Nano (wrist)'}


def make_app(select=None, max_fps=15.0):
    store = FrameStore(('zedx', 'zedx_nano'), stale_after=1.0)
    return CameraUIApp(Renderer(store), LABELS, select=select, max_fps=max_fps)


def feed(app, camera='zedx_nano', rect=True):
    store = app.store
    if rect:
        store.update(camera, 'rect', rgb_image(), stamp_ns=7)
        store.update(camera, 'depth', depth_ramp(400, 1600), stamp_ns=7)
        store.update(camera, 'info', INTRINSICS, stamp_ns=7)
    store.update(camera, 'raw', jpeg(rgb_image()), stamp_ns=7)


def get_json(port, path):
    status, headers, body = request(port, 'GET', path)
    assert headers['Content-Type'] == 'application/json'
    return status, json.loads(body)


def test_page_and_status():
    app = make_app()
    feed(app)
    app.set_hub_status(json.dumps(hub_status(active='zedx_nano')))
    with serving(app) as port:
        status, headers, page = request(port, 'GET', '/')
        assert status == 200 and headers['Content-Type'].startswith('text/html')
        assert b'<title>Camera viewer</title>' in page and b'/api/status' in page
        assert b'http://' not in page and b'https://' not in page   # self-contained, no external resources

        status, body = get_json(port, '/api/status')
    assert status == 200
    assert body['hub']['active'] == 'zedx_nano' and body['hub']['reachable'] is True
    assert 0 <= body['hub_age_s'] < 5 and body['switching'] is None and body['stale_after'] == 1.0
    nano, zedx = body['cameras']['zedx_nano'], body['cameras']['zedx']
    assert nano['label'] == 'ZED X Nano (wrist)' and zedx['label'] == 'ZED X (base)'
    assert nano['rgb']['source'] == 'rectified' and (nano['rgb']['width'], nano['rgb']['height']) == (W, H)
    assert nano['depth']['median_mm'] == pytest.approx(1000, abs=2) and nano['depth']['valid_fraction'] == pytest.approx(0.9)
    assert nano['intrinsics'] == {'fx': 480.0, 'fy': 480.0, 'cx': 480.0, 'cy': 300.0}
    assert zedx['rgb']['source'] is None and zedx['depth']['age_s'] is None and zedx['intrinsics'] is None


def test_status_before_any_hub_message_and_invalid_hub_messages():
    app = make_app()
    assert app.status()['hub'] is None and app.status()['hub_age_s'] is None
    with pytest.raises(ValueError):
        app.set_hub_status('not json')
    with pytest.raises(ValueError):
        app.set_hub_status('[1, 2]')
    assert app.status()['hub'] is None


def test_rgb_and_depth_streams_send_the_newest_frames():
    app = make_app()
    feed(app)
    with serving(app) as port:
        with open_stream(port, '/stream/zedx_nano/rgb.mjpg') as response:
            rgb = decode(read_parts(response, 1)[0])
        assert rgb.shape == (H, W, 3) and np.abs(rgb.astype(int) - rgb_image()).mean() < 3
        with open_stream(port, '/stream/zedx_nano/depth.mjpg?near=400&far=1600') as response:
            depth = decode(read_parts(response, 1)[0])
        assert depth.shape == (H, W, 3) and depth[300, 5, 2] > 90 and depth[20, 480].max() < 25
        with open_stream(port, '/stream/zedx_nano/depth.mjpg?auto=1') as response:
            assert decode(read_parts(response, 1)[0]).shape == (H, W, 3)

        feed(app, 'zedx', rect=False)   # no depth node for the ZED X: its raw JPEG, unchanged
        with open_stream(port, '/stream/zedx/rgb.mjpg') as response:
            assert read_parts(response, 1)[0] == app.store.get('zedx', 'raw').data


def test_streams_are_rate_limited_and_skip_repeated_frames():
    app = make_app(max_fps=5.0)
    feed(app)
    stop = threading.Event()

    def publisher():   # a 50 Hz camera
        while not stop.is_set():
            app.store.update('zedx_nano', 'rect', rgb_image())
            time.sleep(0.02)
    thread = threading.Thread(target=publisher, daemon=True)
    thread.start()
    try:
        with serving(app) as port, open_stream(port, '/stream/zedx_nano/rgb.mjpg') as response:
            start = time.monotonic()
            read_parts(response, 6)
            elapsed = time.monotonic() - start
    finally:
        stop.set()
        thread.join()
    assert elapsed >= 0.9   # 6 parts at <= 5 fps: >= 5 intervals of 0.2 s

    app = make_app()
    feed(app)
    with serving(app) as port, open_stream(port, '/stream/zedx_nano/rgb.mjpg', timeout=0.5) as response:
        read_parts(response, 1)
        with pytest.raises(TimeoutError):   # no new frame, no new part
            read_parts(response, 1)


def stream_threads():
    return [t for t in threading.enumerate() if 'process_request' in t.name]


def test_streams_end_when_the_client_disconnects_or_the_app_closes():
    app = make_app()
    feed(app)
    with serving(app) as port:
        before = len(stream_threads())
        with open_stream(port, '/stream/zedx_nano/rgb.mjpg') as response:
            read_parts(response, 1)
            assert len(stream_threads()) == before + 1
        deadline = time.monotonic() + 2.0
        while len(stream_threads()) > before and time.monotonic() < deadline:
            time.sleep(0.02)
        assert len(stream_threads()) == before

        with open_stream(port, '/stream/zedx_nano/depth.mjpg') as response:
            read_parts(response, 1)
            app.close()
            assert response.fp.read().strip() == b''   # the server ended the stream (after the last part's CRLF)


def test_bad_stream_and_snapshot_requests():
    app = make_app()
    with serving(app) as port:
        assert request(port, 'GET', '/stream/nope/rgb.mjpg')[0] == 404
        assert request(port, 'GET', '/stream/zedx/color.mjpg')[0] == 404
        assert request(port, 'GET', '/stream/zedx/depth.mjpg?near=900&far=100')[0] == 400
        assert request(port, 'GET', '/stream/zedx/depth.mjpg?near=abc')[0] == 400
        assert request(port, 'GET', '/snapshot/zedx/rgb.png')[0] == 503   # no frame yet
        assert request(port, 'GET', '/no/such/page')[0] == 404


def test_depth_range_query():
    assert depth_range_query({}) == (None, None)
    assert depth_range_query({'auto': ['1'], 'near': ['5'], 'far': ['6']}) == (None, None)
    assert depth_range_query({'near': ['250'], 'far': ['900']}) == (250.0, 900.0)
    assert depth_range_query({'auto': ['0']}) == (100.0, 1000.0)
    with pytest.raises(ValueError):
        depth_range_query({'near': ['-1'], 'far': ['900']})


def test_snapshots_are_png_downloads():
    app = make_app()
    feed(app)
    with serving(app) as port:
        status, headers, body = request(port, 'GET', '/snapshot/zedx_nano/depth.png')
        assert status == 200 and headers['Content-Type'] == 'image/png'
        assert headers['Content-Disposition'] == 'attachment; filename="zedx_nano_depth_7.png"'
        depth = decode(body, cv2.IMREAD_UNCHANGED)
        assert depth.dtype == np.uint16 and np.array_equal(depth, depth_ramp(400, 1600))
        status, headers, body = request(port, 'GET', '/snapshot/zedx_nano/rgb.png')
        assert status == 200 and np.array_equal(decode(body), rgb_image())


def test_depth_at():
    app = make_app()
    feed(app)
    with serving(app) as port:
        status, body = get_json(port, '/api/depth_at?camera=zedx_nano&u=0.5&v=0.5')
        assert status == 200 and body['depth_mm'] == pytest.approx(1000, abs=3)
        x, y, z = body['xyz_mm']
        assert abs(x) < 3 and y == 0.0 and z == body['depth_mm'] and 0 <= body['age_s'] < 5
        assert get_json(port, '/api/depth_at?camera=zedx_nano&u=0.5&v=0.01')[1]['depth_mm'] is None   # invalid band
        assert get_json(port, '/api/depth_at?camera=zedx&u=0.5&v=0.5')[1] == {'depth_mm': None, 'xyz_mm': None, 'age_s': None}
        assert get_json(port, '/api/depth_at?camera=zedx_nano&u=1.5&v=0.5')[0] == 400
        assert get_json(port, '/api/depth_at?camera=zedx_nano&u=x&v=0.5')[0] == 400
        assert get_json(port, '/api/depth_at?camera=nope&u=0.5&v=0.5')[0] == 400


def test_select_calls_the_hub_and_reports_its_answer():
    calls = []

    def select(camera):
        calls.append(camera)
        return 200, {'ok': camera != 'zedx', 'message': 'held by another process' if camera == 'zedx' else 'switched'}
    app = make_app(select)
    with serving(app) as port:
        def post(body):
            status, _, reply = request(port, 'POST', '/api/select', json.dumps(body) if not isinstance(body, bytes) else body)
            return status, json.loads(reply)
        assert post({'camera': 'zedx_nano'}) == (200, {'ok': True, 'message': 'switched'})
        assert post({'camera': 'zedx'}) == (200, {'ok': False, 'message': 'held by another process'})
        assert post({'camera': None})[0] == 200 and post({'camera': 'none'})[0] == 200
        assert post({'camera': 'zedx_mini'})[0] == 400
        assert post({'camera': ['zedx']})[0] == 400
        assert post({'cam': 'zedx'})[0] == 400
        assert post(b'{not json')[0] == 400
    assert calls == ['zedx_nano', 'zedx', None, None]

    with serving(make_app(select=None)) as port:
        status, _, reply = request(port, 'POST', '/api/select', json.dumps({'camera': 'zedx'}))
        assert status == 503 and json.loads(reply)['ok'] is False


def test_status_shows_a_switch_in_progress():
    entered, release = threading.Event(), threading.Event()

    def slow_select(camera):
        entered.set()
        release.wait(5)
        return 200, {'ok': True, 'message': ''}
    app = make_app(slow_select)
    worker = threading.Thread(target=app.select, args=('zedx',))
    worker.start()
    try:
        assert entered.wait(5)
        assert app.status()['switching']['camera'] == 'zedx'
    finally:
        release.set()
        worker.join()
    assert app.status()['switching'] is None


def test_select_takes_only_same_origin_json():
    calls = []

    def select(camera):
        calls.append(camera)
        return 200, {'ok': True, 'message': 'released'}
    with serving(make_app(select)) as port:
        def post(headers):
            status, _, reply = request(port, 'POST', '/api/select', json.dumps({'camera': None}), headers=headers)
            return status, json.loads(reply)
        # what any web page can make a browser send without a CORS preflight
        assert post({'Content-Type': 'text/plain', 'Origin': 'http://evil.example'})[0] == 403
        assert post({'Content-Type': 'text/plain'})[0] == 415
        assert post({'Content-Type': 'application/x-www-form-urlencoded'})[0] == 415
        assert post({})[0] == 415
        assert post({'Content-Type': 'application/json', 'Origin': 'null'})[0] == 403
        assert calls == []
        # the page itself (same origin) and scripts (no Origin)
        assert post({'Content-Type': 'application/json', 'Origin': f'http://127.0.0.1:{port}'}) == (200, {'ok': True, 'message': 'released'})
        assert post({'Content-Type': 'application/json; charset=utf-8'})[0] == 200
    assert calls == [None, None]


def test_switches_do_not_queue_behind_one_that_waits():
    """A newer switch or a release goes to the hub at once (which supersedes the waiting one), and the newest shows."""
    answer = {camera: threading.Event() for camera in ('zedx', 'zedx_nano', None)}
    started = []

    def select(camera):
        started.append(camera)
        assert answer[camera].wait(5)
        return 200, {'ok': True, 'message': f'{camera} done'}
    app = make_app(select)
    replies = {}

    def switch(camera):
        worker = threading.Thread(target=lambda: replies.__setitem__(camera, app.select(camera)))
        worker.start()
        deadline = time.monotonic() + 5
        while camera not in started and time.monotonic() < deadline:
            time.sleep(0.01)
        return worker
    zedx = switch('zedx')                      # its camera never delivers
    assert app.status()['switching']['camera'] == 'zedx'
    nano = switch('zedx_nano')                 # not queued behind it
    assert app.status()['switching']['camera'] == 'zedx_nano'
    answer['zedx'].set()                       # the superseded switch ends first: the newer one keeps showing
    zedx.join(5)
    assert app.status()['switching']['camera'] == 'zedx_nano'
    release = switch(None)
    answer[None].set()
    release.join(5)
    assert app.status()['switching'] is None   # the newest one ended; zedx_nano is superseded at the hub
    answer['zedx_nano'].set()
    nano.join(5)
    assert started == ['zedx', 'zedx_nano', None] and app.status()['switching'] is None
    assert replies == {camera: (200, {'ok': True, 'message': f'{camera} done'}) for camera in ('zedx', 'zedx_nano', None)}
