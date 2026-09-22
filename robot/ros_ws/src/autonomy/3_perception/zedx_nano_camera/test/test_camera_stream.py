import io
import time

import numpy as np
import pytest

from camera_fakes import CONF, INFO, jpeg, part, fake_server
from zedx_nano_camera import stream
from zedx_nano_camera.stream import (NanoStreamClient, captured_monotonic, decode_jpeg, fetch_calibration_text,
                                     frame_header, iter_mjpeg_parts, split_stereo_part)

HEADERS = {'x-frame-id': '7', 'x-capture-ns': '1050000000', 'x-ready-ns': '1060000000', 'x-sensor': 'left'}


def test_parser_splits_parts_and_stops_at_truncation():
    parts = list(iter_mjpeg_parts(io.BytesIO(part(1) + part(2) + part(3)[:-40])))
    assert [int(h['x-frame-id']) for h, _ in parts] == [1, 2]
    assert parts[0][1] == jpeg() and parts[0][0]['content-type'] == 'image/jpeg'


def test_parser_skips_noise_and_rejects_missing_or_huge_length():
    parts = list(iter_mjpeg_parts(io.BytesIO(b'\r\n\r\n' + part(5))))
    assert [int(h['x-frame-id']) for h, _ in parts] == [5]
    with pytest.raises(ValueError):
        list(iter_mjpeg_parts(io.BytesIO(b'--frame\r\nContent-Type: image/jpeg\r\n\r\nabc\r\n')))
    with pytest.raises(ValueError):
        list(iter_mjpeg_parts(io.BytesIO(b'--frame\r\nContent-Length: 999999999\r\n\r\n')))


def test_decode_jpeg():
    bgr = decode_jpeg(jpeg())
    assert bgr.shape == (6, 8, 3) and bgr[1, 1, 2] > 250 and bgr[1, 1, 0] < 5
    with pytest.raises(ValueError):
        decode_jpeg(b'not a jpeg')


def test_frame_header_and_capture_time():
    header = frame_header(HEADERS, 8, 6)
    assert header['frame_id'] == 7 and header['t_image_ns'] == 1_050_000_000 and header['image_size'] == [8, 6]
    assert header['t_grab_done_ns'] == header['t_pub_ns'] == 1_060_000_000 and header['capture_mono_ns'] is None
    # wall clock: received at local 1.0 s wall / 10.0 s monotonic, Xavier 0.1 s ahead -> captured 50 ms ago
    assert captured_monotonic(header, 1_000_000_000, 10.0, 100_000_000) == pytest.approx(9.95)
    # monotonic stamp wins when both sides report it
    header = frame_header(dict(HEADERS, **{'x-capture-mono-ns': '5000000000'}), 8, 6)
    assert captured_monotonic(header, 0, 10.0, 0, mono_offset_ns=-4_000_000_000) == pytest.approx(9.0)
    stereo = frame_header(dict(HEADERS, **{'x-right-frame-id': '9', 'x-sync-us': '120'}), 8, 6)
    assert stereo['right_frame_id'] == 9 and stereo['stereo_sync_us'] == 120


@pytest.mark.parametrize('headers', [
    {},
    {'x-frame-id': 'x', 'x-capture-ns': '1', 'x-ready-ns': '1'},
    {'x-frame-id': '-1', 'x-capture-ns': '1', 'x-ready-ns': '1'},
    {'x-frame-id': '1', 'x-capture-ns': '1', 'x-ready-ns': '1', 'x-capture-mono-ns': '-5'},
])
def test_frame_header_rejects_bad_metadata(headers):
    with pytest.raises(ValueError):
        frame_header(headers, 8, 6)


def test_split_stereo_part():
    headers = {'x-left-length': '3'}
    assert split_stereo_part(headers, b'abcdefg') == (b'abc', b'defg')
    with pytest.raises(ValueError):
        split_stereo_part({'x-left-length': '7'}, b'abcdefg')
    with pytest.raises(ValueError):
        split_stereo_part({}, b'abcdefg')


def test_client_info_clock_calibration_and_feed(monkeypatch):
    requests = fake_server(monkeypatch)
    client = NanoStreamClient('xavier', 8090)
    info = client.fetch_info('stereo')
    assert info['serial'] == INFO['serial'] and requests[-1] == 'http://xavier:8090/info'
    offset, uncertainty, mono_offset = client.synchronize_clock(samples=3)
    assert abs(offset) < 50_000_000 and 0 <= uncertainty < 50_000_000 and abs(mono_offset) < 50_000_000
    conf, calibration = client.load_calibration(info)
    assert conf['STEREO']['Baseline'] == pytest.approx(18.01) and calibration['source'] == 'xavier:/calibration.conf'
    assert calibration['left']['fx'] == pytest.approx(475) and calibration['resolution'] == [960, 600]
    response = client.open_feed('stereo')
    headers, body = next(iter_mjpeg_parts(response))
    assert split_stereo_part(headers, body) == (b'L', b'R') and headers['x-right-frame-id'] == '3'
    response.close()


def test_client_rejects_old_servers_and_missing_feeds(monkeypatch):
    fake_server(monkeypatch, info={'server_version': 1})
    with pytest.raises(RuntimeError, match='too old'):
        NanoStreamClient('xavier').fetch_info()
    fake_server(monkeypatch, info={'stereo': {'available': False}})
    with pytest.raises(RuntimeError, match='no stereo feed'):
        NanoStreamClient('xavier').fetch_info('stereo')
    assert NanoStreamClient('xavier').fetch_info('left')['sensors']['left'] == '/dev/video3'
    with pytest.raises(RuntimeError, match='no sensor'):
        NanoStreamClient('xavier').fetch_info('middle')
    with pytest.raises(RuntimeError, match='content type'):
        NanoStreamClient('xavier').open_feed('html')


def test_calibration_falls_back_to_stereolabs_by_serial(monkeypatch):
    fake_server(monkeypatch, calibration_status=404)
    client = NanoStreamClient('xavier')
    fetched = []

    class Response(io.BytesIO):
        pass

    def urlopen(url, timeout=None):
        fetched.append(url)
        return Response(CONF.encode())
    monkeypatch.setattr(stream.urllib.request, 'urlopen', urlopen)
    text, source = fetch_calibration_text(client.get, 99292912)
    assert source == 'calib.stereolabs.com' and fetched == ['https://calib.stereolabs.com/?SN=99292912']
    assert '[STEREO]' in text
    with pytest.raises(RuntimeError, match='--serial'):
        fetch_calibration_text(client.get, 0)


def test_captured_time_is_recent_for_a_live_part():
    now_ns, mono = time.time_ns(), time.monotonic()
    header = frame_header({'x-frame-id': '1', 'x-capture-ns': str(now_ns - 20_000_000), 'x-ready-ns': str(now_ns)}, 4, 4)
    assert mono - captured_monotonic(header, now_ns, mono, 0) == pytest.approx(0.02, abs=1e-6)
    assert np.isfinite(captured_monotonic(header, now_ns, mono, 0))
