"""The HTTP clients against the fake camera hub: per-camera routes, inactive cameras and switching."""
import threading
import time
import urllib.error

import pytest

from camera_fakes import FakeHub
from zedx_nano_camera import stream
from zedx_nano_camera.stream import (CameraHubClient, CameraInactive, NanoStreamClient, iter_mjpeg_parts,
                                     split_stereo_part)


@pytest.fixture
def hub():
    with FakeHub(active='zedx') as fake:
        yield fake


def test_camera_client_prefixes_every_path_but_time(hub):
    client = NanoStreamClient('127.0.0.1', hub.port, camera='zedx')
    assert client.url('/info') == f'http://127.0.0.1:{hub.port}/cameras/zedx/info'
    assert client.url('/time') == f'http://127.0.0.1:{hub.port}/time' and client.name == 'zedx'
    info = client.fetch_info('stereo')
    assert info['model'] == 'ZED X' and info['camera'] == 'zedx'
    client.synchronize_clock(samples=2)
    conf, calibration = client.load_calibration(info)
    assert calibration['camera_model'] == 'ZED X' and calibration['serial'] == 42757821
    assert calibration['stereo']['baseline_m'] == pytest.approx(0.11987)
    assert calibration['left']['fx'] == pytest.approx(367.5) and conf['STEREO']['Baseline'] == pytest.approx(119.87)
    response = client.open_feed('stereo')
    headers, body = next(iter_mjpeg_parts(response))
    response.close()
    assert headers['x-camera'] == 'zedx' and split_stereo_part(headers, body) == hub.pairs['zedx']
    assert hub.paths() == ['/cameras/zedx/info', '/time', '/time', '/cameras/zedx/calibration.conf',
                           '/cameras/zedx/video_feed/stereo']


def test_root_client_uses_the_legacy_routes(hub):
    hub.select('zedx_nano')
    client = NanoStreamClient('127.0.0.1', hub.port)
    assert client.name == 'zedx_nano' and client.url('/info') == f'http://127.0.0.1:{hub.port}/info'
    info = client.fetch_info()
    assert client.load_calibration(info)[1]['camera_model'] == 'ZED X Nano'
    response = client.open_feed()
    headers, _ = next(iter_mjpeg_parts(response))
    response.close()
    assert headers['x-camera'] == 'zedx_nano'
    assert hub.paths() == ['/info', '/calibration.conf', '/video_feed/stereo']


def test_inactive_camera_raises_camera_inactive_but_serves_calibration(hub):
    nano = NanoStreamClient('127.0.0.1', hub.port, camera='zedx_nano')
    with pytest.raises(CameraInactive, match='zedx_nano is not the active camera') as raised:
        nano.fetch_info()                    # /info says "active": false
    assert raised.value.active == 'zedx'
    with pytest.raises(CameraInactive) as raised:
        nano.open_feed()                     # feeds answer 503
    assert raised.value.active == 'zedx' and 'not active' in str(raised.value)
    assert nano.load_calibration(nano.get_json('/info'))[1]['camera_model'] == 'ZED X Nano'
    legacy = NanoStreamClient('127.0.0.1', hub.port)
    with pytest.raises(CameraInactive, match='not the active camera') as raised:
        legacy.fetch_info()                  # the root /info answers 503 while the Nano is not active
    assert raised.value.active == 'zedx'
    hub.select(None)
    with pytest.raises(CameraInactive) as raised:
        legacy.open_feed()
    assert raised.value.active is None
    assert isinstance(raised.value, RuntimeError)   # callers that catch RuntimeError keep working


def test_unknown_camera_is_a_clear_error(hub):
    with pytest.raises(RuntimeError, match="no camera 'zed2'"):
        NanoStreamClient('127.0.0.1', hub.port, camera='zed2').fetch_info()


def test_feed_ends_when_the_hub_switches_cameras(hub):
    response = NanoStreamClient('127.0.0.1', hub.port, camera='zedx').open_feed()
    parts = iter_mjpeg_parts(response)
    first = int(next(parts)[0]['x-frame-id'])
    threading.Timer(0.2, hub.select, ('zedx_nano',)).start()
    t0 = time.monotonic()
    ids = [int(headers['x-frame-id']) for headers, _ in parts]
    response.close()
    assert time.monotonic() - t0 < 2.0 and ids == list(range(first + 1, first + 1 + len(ids)))


def test_hub_client_status_and_select(hub):
    client = CameraHubClient('127.0.0.1', hub.port)
    status = client.status()
    assert status['server'] == 'camera_hub' and status['active'] == 'zedx' and status['state'] == 'streaming'
    reply = client.select('zedx_nano', timeout=12.5)
    assert reply['ok'] and reply['status']['active'] == 'zedx_nano' and hub.active == 'zedx_nano'
    assert hub.selects[-1] == {'timeout': ['12.5']}
    reply = client.select(None)
    assert reply['ok'] and reply['status']['active'] is None and reply['status']['state'] == 'idle'
    reply = client.select('zed2')                      # 400
    assert not reply['ok'] and 'unknown camera' in reply['error']
    hub.failing.add('zedx')
    reply = client.select('zedx')                      # 502, stays selected
    assert not reply['ok'] and reply['error'] == 'zedx failed to start' and reply['status']['state'] == 'error'
    assert hub.active == 'zedx'


def test_hub_client_raises_when_there_is_no_hub(hub, monkeypatch):
    port = hub.port
    hub.close()
    client = CameraHubClient('127.0.0.1', port, timeout=1.0)
    with pytest.raises(urllib.error.URLError):
        client.status()
    with pytest.raises(urllib.error.URLError):
        client.select('zedx')

    def old_server(url, timeout=None):   # the Nano-only server has no hub API
        raise urllib.error.HTTPError(str(url), 404, 'NOT FOUND', {}, None)
    monkeypatch.setattr(stream, '_open', old_server)
    with pytest.raises(RuntimeError, match='old Nano-only server'):
        client.status()
    with pytest.raises(RuntimeError, match='old Nano-only server'):
        client.select('zedx')
