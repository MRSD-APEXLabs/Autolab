"""Frame store, depth colorization, auto range, encoding cache and depth probe on synthetic frames."""
from pathlib import Path
import re
import threading

import cv2
import numpy as np
import pytest

from camera_ui import frames
from camera_ui.frames import AutoRange, FrameStore, Renderer, auto_range, jpeg_size, point_at
from ui_fakes import FX, H, INTRINSICS, W, FakeClock, decode, depth_ramp, jpeg, rgb_image
from zedx_nano_depth.stereo import colorize_depth

CAMERAS = ('zedx', 'zedx_nano')


def make_store(stale_after=1.0):
    clock = FakeClock()
    return FrameStore(CAMERAS, stale_after, clock=clock), clock


@pytest.fixture
def encodes(monkeypatch):
    """Counts JPEG encodings done by the renderer."""
    calls = []
    real = frames.encode_jpeg

    def counting(bgr, quality):
        calls.append(bgr.shape)
        return real(bgr, quality)
    monkeypatch.setattr(frames, 'encode_jpeg', counting)
    return calls


def test_rate_age_and_freshness_follow_the_store_clock():
    store, clock = make_store()
    for _ in range(11):
        store.update('zedx', 'rect', rgb_image(), stamp_ns=1)
        clock.now += 0.1
    frame = store.get('zedx', 'rect')
    assert store.rate('zedx', 'rect') == pytest.approx(10.0)
    assert store.age(frame) == pytest.approx(0.1) and store.fresh(frame)
    assert store.get('zedx_nano', 'rect') is None and store.rate('zedx_nano', 'rect') == 0.0
    clock.now += 2.5
    assert not store.fresh(frame) and store.rate('zedx', 'rect') == 0.0
    assert store.get('zedx', 'rect').seq == frame.seq   # stale frames are kept for the last image


def test_rgb_prefers_the_fresh_rectified_image_and_falls_back_to_the_raw_jpeg():
    store, clock = make_store()
    assert store.rgb('zedx') == (None, None)
    store.update('zedx', 'raw', jpeg(rgb_image()))
    assert store.rgb('zedx')[1] == 'raw'
    store.update('zedx', 'rect', rgb_image())
    assert store.rgb('zedx')[1] == 'rectified'
    clock.now += 0.6
    store.update('zedx', 'raw', jpeg(rgb_image()))
    clock.now += 0.6   # rectified stale (depth node stopped), raw still fresh
    assert store.rgb('zedx')[1] == 'raw'
    clock.now += 5.0   # both stale: the newer one
    assert store.rgb('zedx')[1] == 'raw'
    store.update('zedx', 'rect', rgb_image())
    clock.now += 5.0
    assert store.rgb('zedx')[1] == 'rectified'


def test_depth_is_turbo_colored_near_red_far_blue_invalid_black():
    store, _ = make_store()
    store.update('zedx', 'depth', depth_ramp(400, 1600))
    seq, data = Renderer(store, jpeg_quality=95).depth_jpeg('zedx', 400, 1600)
    image = decode(data).astype(int)
    assert seq == store.get('zedx', 'depth').seq and image.shape == (H, W, 3)
    near, far, invalid = image[300, 5], image[300, int(0.8 * W)], image[20, 480]
    assert near[2] > 90 and near[2] > near[0] + 60           # BGR: red near
    assert far[0] > 180 and far[0] > far[2] + 100             # blue far (the last few % fade to dark violet)
    assert invalid.max() < 25                                 # black invalid
    expected = colorize_depth(depth_ramp(400, 1600), 400, 1600).astype(int)
    assert np.abs(image - expected).mean() < 4                # the same colormap as the depth node's preview


def test_page_colorbar_matches_the_depth_colormap():
    page = (Path(frames.__file__).parent / 'web' / 'index.html').read_text()
    gradient = re.search(r'--turbo: linear-gradient\(90deg,([^;]*)\);', page, re.S).group(1)
    stops = re.findall(r'#([0-9a-f]{6}) (\d+)%', gradient)
    assert len(stops) >= 5
    for color, percent in stops:
        depth = np.full((1, 1), 100 + 9 * int(percent), np.uint16)   # near 100 mm, far 1000 mm
        b, g, r = colorize_depth(depth, 100, 1000)[0, 0].astype(int)
        css = [int(color[i:i + 2], 16) for i in (0, 2, 4)]
        assert np.abs(np.array(css) - (r, g, b)).max() <= 16, (percent, css, (r, g, b))


def test_auto_range_takes_percentiles_of_valid_depth_and_smooths_over_time():
    depth = depth_ramp(400, 1600)
    near, far = auto_range(depth)
    assert near == pytest.approx(424, abs=6) and far == pytest.approx(1576, abs=6)
    assert auto_range(np.zeros((H, W), np.uint16)) is None

    store, clock = make_store()
    smoothed = AutoRange(tau=0.5)
    store.update('zedx', 'depth', depth)
    first = smoothed.update(store.get('zedx', 'depth'))
    assert first == pytest.approx((near, far))
    clock.now += 0.1
    store.update('zedx', 'depth', depth_ramp(1400, 2600))
    moved = smoothed.update(store.get('zedx', 'depth'))
    assert near < moved[0] < 1424 and far < moved[1] < 2576   # partway, no jump
    assert smoothed.update(store.get('zedx', 'depth')) == moved  # once per frame
    clock.now += 10.0   # after a long gap (camera switched back) it starts over
    store.update('zedx', 'depth', depth_ramp(1400, 2600))
    assert smoothed.update(store.get('zedx', 'depth')) == pytest.approx((1424, 2576), abs=6)


def test_encoding_is_shared_per_frame_and_parameters(encodes):
    store, _ = make_store()
    renderer = Renderer(store)
    store.update('zedx', 'rect', rgb_image())
    store.update('zedx', 'depth', depth_ramp())
    first = renderer.rgb_jpeg('zedx')
    assert renderer.rgb_jpeg('zedx') == first and len(encodes) == 1
    assert first[2] == 'rectified' and decode(first[1]).shape == (H, W, 3)
    store.update('zedx', 'rect', rgb_image())
    assert renderer.rgb_jpeg('zedx')[0] != first[0] and len(encodes) == 2

    for _ in range(3):
        renderer.depth_jpeg('zedx', 100, 1000)
        renderer.depth_jpeg('zedx', 300, 900)
        renderer.depth_jpeg('zedx')   # auto
    assert len(encodes) == 5

    store.update('zedx', 'rect', rgb_image())   # many tabs asking at once: one encoding
    barrier = threading.Barrier(8)
    results = []

    def tab():
        barrier.wait()
        results.append(renderer.rgb_jpeg('zedx'))
    threads = [threading.Thread(target=tab) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert len(encodes) == 6 and len(set(results)) == 1


def test_raw_jpeg_is_forwarded_without_reencoding(encodes):
    store, _ = make_store()
    data = jpeg(rgb_image())
    store.update('zedx_nano', 'raw', data)
    seq, forwarded, source = Renderer(store).rgb_jpeg('zedx_nano')
    assert forwarded is data and source == 'raw' and not encodes
    assert jpeg_size(data) == (W, H)
    assert jpeg_size(b'\xff\xd8garbage') is None and jpeg_size(b'not a jpeg') is None


def test_point_at_takes_the_median_of_valid_pixels_and_projects_it():
    depth = np.full((H, W), 1000, np.uint16)
    depth[300, 480] = 0          # a hole at the centre does not matter
    depth[301, 481] = 9000       # nor does one outlier
    depth_mm, xyz = point_at(depth, INTRINSICS, 0.5, 0.5)
    assert depth_mm == 1000 and xyz == [0.0, 0.0, 1000.0]
    depth_mm, xyz = point_at(depth, INTRINSICS, 0.75, 0.25)
    assert depth_mm == 1000
    assert xyz == pytest.approx([(720 - W / 2) * 1000 / FX, (150 - H / 2) * 1000 / FX, 1000.0], abs=0.1)
    assert point_at(depth, INTRINSICS, 1.0, 1.0)[0] == 1000   # the far edge is the last pixel
    assert point_at(depth, None, 0.5, 0.5) == (1000, None)
    assert point_at(depth, dict(INTRINSICS, width=1920, height=1200), 0.5, 0.5) == (1000, None)
    depth[:10, :10] = 0
    assert point_at(depth, INTRINSICS, 0.0, 0.0) == (None, None)


def test_camera_status_reports_rates_sizes_depth_statistics_and_intrinsics():
    store, clock = make_store()
    renderer = Renderer(store)
    empty = renderer.camera_status('zedx')
    assert empty['rgb'] == {'fps': 0.0, 'age_s': None, 'source': None, 'width': None, 'height': None}
    assert empty['depth']['median_mm'] is None and empty['intrinsics'] is None
    for _ in range(5):
        store.update('zedx', 'rect', rgb_image())
        store.update('zedx', 'depth', depth_ramp(400, 1600))
        store.update('zedx', 'info', INTRINSICS)
        clock.now += 0.05
    status = renderer.camera_status('zedx')
    assert status['rgb'] == {'fps': 20.0, 'age_s': 0.05, 'source': 'rectified', 'width': W, 'height': H}
    depth = status['depth']
    assert depth['fps'] == 20.0 and depth['valid_fraction'] == pytest.approx(0.9)
    assert depth['min_mm'] == 400 and depth['max_mm'] == 1600 and depth['median_mm'] == pytest.approx(1000, abs=2)
    assert depth['auto_range_mm'] == [pytest.approx(424, abs=6), pytest.approx(1576, abs=6)]
    assert status['intrinsics'] == {'fx': FX, 'fy': FX, 'cx': W / 2, 'cy': H / 2}
    store.update('zedx_nano', 'raw', jpeg(rgb_image()[:300, :480]))
    rgb = renderer.camera_status('zedx_nano')['rgb']
    assert rgb['source'] == 'raw' and (rgb['width'], rgb['height']) == (480, 300)   # read from the JPEG header


def test_png_snapshots_are_lossless():
    store, _ = make_store()
    renderer = Renderer(store)
    assert renderer.rgb_png('zedx') == (None, None) and renderer.depth_png('zedx') == (None, None)
    image, depth = rgb_image(), depth_ramp()
    store.update('zedx', 'rect', image, stamp_ns=42)
    store.update('zedx', 'depth', depth, stamp_ns=42)
    frame, png = renderer.depth_png('zedx')
    decoded = decode(png, cv2.IMREAD_UNCHANGED)
    assert frame.stamp_ns == 42 and decoded.dtype == np.uint16 and np.array_equal(decoded, depth)
    assert np.array_equal(decode(renderer.rgb_png('zedx')[1]), image)
    store.update('zedx_nano', 'raw', jpeg(image))
    assert decode(renderer.rgb_png('zedx_nano')[1]).shape == (H, W, 3)
