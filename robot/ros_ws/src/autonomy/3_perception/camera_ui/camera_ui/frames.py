"""Newest frame of every camera stream, and its rendering for the browser (plain Python, no ROS).

The node's callbacks only store the newest message of each stream, so they stay cheap however
fast the cameras publish. Encoding happens lazily in the HTTP threads, and `Renderer` encodes a
frame at most once per set of render parameters, however many browser tabs watch it.

Streams kept per camera:

  rect   rectified left image, BGR array (zedx_nano_depth's left/image_rect_color)
  raw    unrectified left JPEG bytes (the camera node's left/image_raw/compressed)
  depth  uint16 depth in mm, 0 = invalid, aligned to rect (depth/image_rect)
  info   intrinsics of rect and depth: dict fx, fy, cx, cy, width, height (depth/camera_info)
"""
from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass
import itertools
import math
import threading
import time

import cv2
import numpy as np

from zedx_nano_depth.stereo import colorize_depth, depth_stats

STREAMS = ('rect', 'raw', 'depth', 'info')


@dataclass(frozen=True)
class Frame:
    data: object      # see the module docstring
    stamp_ns: int     # header stamp
    received: float   # store clock (time.monotonic) on arrival
    seq: int          # store-wide arrival counter; identifies the frame in caches


class RateMeter:
    """Arrival rate over the last `window` seconds; 0 once the stream has been silent that long."""

    def __init__(self, window=2.0):
        self.window = window
        self._times = deque()

    def tick(self, now):
        self._times.append(now)
        while self._times[0] < now - self.window:
            self._times.popleft()

    def rate(self, now):
        times = [t for t in self._times if t >= now - self.window]
        if len(times) < 2 or times[-1] <= times[0]:
            return 0.0
        return (len(times) - 1) / (times[-1] - times[0])


class FrameStore:
    """Thread-safe newest frame and arrival rate of each (camera, stream)."""

    def __init__(self, cameras, stale_after=1.0, clock=time.monotonic):
        self.cameras = tuple(cameras)
        self.stale_after = float(stale_after)
        self.clock = clock
        self._frames = {}
        self._meters = {(camera, stream): RateMeter() for camera in self.cameras for stream in STREAMS}
        self._seq = itertools.count(1)
        self._lock = threading.Lock()

    def update(self, camera, stream, data, stamp_ns=0):
        now = self.clock()
        with self._lock:
            self._frames[camera, stream] = Frame(data, int(stamp_ns), now, next(self._seq))
            self._meters[camera, stream].tick(now)

    def get(self, camera, stream):
        with self._lock:
            return self._frames.get((camera, stream))

    def rate(self, camera, stream):
        with self._lock:
            return self._meters[camera, stream].rate(self.clock())

    def age(self, frame):
        return None if frame is None else self.clock() - frame.received

    def fresh(self, frame):
        return frame is not None and self.clock() - frame.received <= self.stale_after

    def rgb(self, camera):
        """(frame, 'rectified' | 'raw') shown as the camera's RGB image, (None, None) before any frame.

        The rectified image is aligned to the depth, so it wins while it is fresh; the raw JPEG
        covers a camera whose depth node isn't running. With neither fresh, the newer one is returned.
        """
        rect, raw = self.get(camera, 'rect'), self.get(camera, 'raw')
        if self.fresh(rect):
            return rect, 'rectified'
        if self.fresh(raw):
            return raw, 'raw'
        if raw is None or (rect is not None and rect.received >= raw.received):
            return (rect, 'rectified') if rect is not None else (None, None)
        return raw, 'raw'


# -- depth --------------------------------------------------------------------------------------
def depth_summary(depth_mm):
    """depth_stats() (valid_fraction, median_mm, min_mm) plus the farthest valid depth, max_mm."""
    stats = depth_stats(depth_mm)
    stats['max_mm'] = int(depth_mm.max()) if stats['median_mm'] is not None else None
    return stats


def auto_range(depth_mm, low=2.0, high=98.0, step=4):
    """(near, far) mm: percentiles of the valid depth on every `step`-th pixel; None without enough depth."""
    values = depth_mm[::step, ::step]
    values = values[values > 0]
    if values.size < 16:
        return None
    near, far = np.percentile(values, (low, high))
    return float(near), float(max(far, near + 10.0))


class AutoRange:
    """auto_range() of a depth stream, smoothed with time constant `tau` s so the colours don't flicker."""

    def __init__(self, tau=0.5):
        self.tau = tau
        self.value = None
        self._seq = self._received = None

    def update(self, frame):
        if frame.seq == self._seq:
            return self.value
        self._seq = frame.seq
        measured = auto_range(frame.data)
        if measured is None:
            return self.value
        if self.value is None:
            self.value = measured
        else:   # a long gap (camera switched back) makes alpha ~1: start over from the new frame
            alpha = 1.0 - math.exp(-max(frame.received - self._received, 0.0) / self.tau)
            self.value = tuple(old + alpha * (new - old) for old, new in zip(self.value, measured))
        self._received = frame.received
        return self.value


def point_at(depth_mm, intrinsics, u, v, window=5):
    """(depth mm, [x, y, z] mm in the optical frame) at normalized image coordinates u, v in [0, 1].

    The depth is the median of the valid pixels in a window x window neighbourhood; xyz needs
    the rectified intrinsics of the same resolution. Unknown values are None.
    """
    height, width = depth_mm.shape
    col, row = min(max(int(u * width), 0), width - 1), min(max(int(v * height), 0), height - 1)
    half = window // 2
    patch = depth_mm[max(row - half, 0):row + half + 1, max(col - half, 0):col + half + 1]
    valid = patch[patch > 0]
    if not valid.size:
        return None, None
    z = float(np.median(valid))
    if not intrinsics or (intrinsics['width'], intrinsics['height']) != (width, height):
        return int(round(z)), None
    x = (col - intrinsics['cx']) * z / intrinsics['fx']
    y = (row - intrinsics['cy']) * z / intrinsics['fy']
    return int(round(z)), [round(x, 1), round(y, 1), round(z, 1)]


# -- encoding -----------------------------------------------------------------------------------
def encode_jpeg(bgr, quality):
    ok, data = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError('JPEG encoding failed')
    return data.tobytes()


def encode_png(image):
    """PNG bytes; a uint16 image stays 16-bit."""
    ok, data = cv2.imencode('.png', image)
    if not ok:
        raise ValueError('PNG encoding failed')
    return data.tobytes()


def jpeg_size(data):
    """(width, height) from a JPEG's start-of-frame segment, None if there is none."""
    if data[:2] != b'\xff\xd8':
        return None
    i = 2
    while i + 9 <= len(data) and data[i] == 0xFF:
        marker = data[i + 1]
        if marker == 0xFF:          # fill byte
            i += 1
            continue
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            return int.from_bytes(data[i + 7:i + 9], 'big'), int.from_bytes(data[i + 5:i + 7], 'big')
        i += 2 + int.from_bytes(data[i + 2:i + 4], 'big')
    return None


class Renderer:
    """Browser views of a FrameStore: JPEG streams, PNG snapshots, depth statistics and probes.

    Results are cached per (camera, view, frame, parameters): the first HTTP thread asking for a
    new frame encodes it, the others wait for it and share the bytes.
    """

    VIEWS = ('rgb', 'depth', 'stats', 'range')
    CACHED_PARAMS = 8   # per view: distinct depth ranges asked for by open tabs

    def __init__(self, store, jpeg_quality=80):
        self.store = store
        self.jpeg_quality = int(jpeg_quality)
        keys = [(camera, view) for camera in store.cameras for view in self.VIEWS]
        self._locks = {key: threading.Lock() for key in keys}
        self._caches = {key: OrderedDict() for key in keys}
        self._auto = {camera: AutoRange() for camera in store.cameras}

    def _cached(self, camera, view, frame, params, make):
        with self._locks[camera, view]:
            cache = self._caches[camera, view]
            hit = cache.get(params)
            if hit is None or hit[0] != frame.seq:
                hit = cache[params] = (frame.seq, make())
            cache.move_to_end(params)
            while len(cache) > self.CACHED_PARAMS:
                cache.popitem(last=False)
            return hit[1]

    def rgb_jpeg(self, camera):
        """(frame seq, JPEG, source) of the RGB view; (None, None, None) before the first frame."""
        frame, source = self.store.rgb(camera)
        if frame is None:
            return None, None, None
        if source == 'raw':
            return frame.seq, frame.data, source   # forwarded as the camera published it
        quality = self.jpeg_quality
        return frame.seq, self._cached(camera, 'rgb', frame, quality, lambda: encode_jpeg(frame.data, quality)), source

    def depth_range(self, camera, frame=None):
        """Smoothed auto range (near, far) mm of the camera's depth, None before any valid depth."""
        frame = frame or self.store.get(camera, 'depth')
        if frame is None:
            return None
        with self._locks[camera, 'range']:
            return self._auto[camera].update(frame)

    def depth_jpeg(self, camera, near_mm=None, far_mm=None):
        """(frame seq, JPEG) of the TURBO-colorized depth; near_mm None = auto range. (None, None) without depth."""
        frame = self.store.get(camera, 'depth')
        if frame is None:
            return None, None
        if near_mm is None:
            params = 'auto'
            near_mm, far_mm = self.depth_range(camera, frame) or (100.0, 1000.0)
        else:
            params = (float(near_mm), float(far_mm))
        return frame.seq, self._cached(camera, 'depth', frame, params, lambda: encode_jpeg(
            colorize_depth(frame.data, near_mm, far_mm), self.jpeg_quality))

    def rgb_png(self, camera):
        """(frame, PNG) of the RGB view as a lossless snapshot; (None, None) before the first frame."""
        frame, source = self.store.rgb(camera)
        if frame is None:
            return None, None
        if source == 'raw':
            bgr = cv2.imdecode(np.frombuffer(frame.data, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError('Undecodable camera JPEG')
            return frame, encode_png(bgr)
        return frame, encode_png(frame.data)

    def depth_png(self, camera):
        """(frame, 16-bit PNG of the depth in mm); (None, None) without depth."""
        frame = self.store.get(camera, 'depth')
        return (None, None) if frame is None else (frame, encode_png(frame.data))

    def depth_at(self, camera, u, v):
        """{'depth_mm', 'xyz_mm', 'age_s'} at normalized coordinates of the rectified image."""
        frame, info = self.store.get(camera, 'depth'), self.store.get(camera, 'info')
        if frame is None:
            return {'depth_mm': None, 'xyz_mm': None, 'age_s': None}
        depth_mm, xyz = point_at(frame.data, info.data if info is not None else None, u, v)
        return {'depth_mm': depth_mm, 'xyz_mm': xyz, 'age_s': round(self.store.age(frame), 3)}

    def camera_status(self, camera):
        """Rates, ages and sizes of the camera's RGB and depth streams, depth statistics and intrinsics."""
        store = self.store

        def timing(frame, stream):
            age = store.age(frame)
            return {'fps': round(store.rate(camera, stream), 1), 'age_s': None if age is None else round(age, 3)}
        rgb, source = store.rgb(camera)
        size = None
        if rgb is not None:
            size = (rgb.data.shape[1], rgb.data.shape[0]) if source == 'rectified' else jpeg_size(rgb.data)
        width, height = size or (None, None)
        depth, info = store.get(camera, 'depth'), store.get(camera, 'info')
        depth_status = {**timing(depth, 'depth'), 'valid_fraction': None, 'median_mm': None, 'min_mm': None,
                        'max_mm': None, 'auto_range_mm': None}
        if depth is not None:
            depth_status.update(self._cached(camera, 'stats', depth, None, lambda: depth_summary(depth.data)))
            auto = self.depth_range(camera, depth)
            depth_status['auto_range_mm'] = None if auto is None else [int(round(value)) for value in auto]
        return {'rgb': {**timing(rgb, 'rect' if source == 'rectified' else 'raw'), 'source': source,
                        'width': width, 'height': height},
                'depth': depth_status,
                'intrinsics': None if info is None else {key: round(info.data[key], 3) for key in ('fx', 'fy', 'cx', 'cy')}}
