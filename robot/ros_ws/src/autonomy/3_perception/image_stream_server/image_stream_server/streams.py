"""Viewer bookkeeping and the newest frame of each stream (plain Python, no ROS).

A stream is keyed (kind, camera), e.g. ('annotated', 'zedx'), and reads one image topic. HTTP
threads `acquire` a stream while they serve it and `release` it after; the node subscribes the
topics of `wanted()` streams only, so nothing is received while nobody watches. Callbacks only keep
the newest message; `latest_jpeg` encodes it on demand, at most once per frame whatever the number
of viewers.
"""
from collections import defaultdict
import itertools
import re
import threading
import time

NAME = re.compile(r'[A-Za-z][A-Za-z0-9_]*')


def parse_streams(entries):
    """{(kind, camera): topic} of ['<kind>/<camera>=/<topic>', ...]; ValueError names the first bad entry."""
    streams = {}
    for entry in entries:
        key, sep, topic = str(entry).partition('=')
        kind, slash, camera = key.strip().partition('/')
        topic = topic.strip()
        if not (sep and slash and NAME.fullmatch(kind) and NAME.fullmatch(camera) and topic.startswith('/')):
            raise ValueError(f'bad stream entry {entry!r}: expected <kind>/<camera>=/<topic>')
        if (kind, camera) in streams:
            raise ValueError(f'duplicate stream {kind}/{camera}')
        streams[(kind, camera)] = topic
    if not streams:
        raise ValueError('streams must not be empty')
    return streams


class StreamHub:
    """Thread-safe viewer counts, newest message and JPEG cache per stream.

    `encode(message) -> bytes` turns a stored message into a JPEG; it runs in the HTTP threads.
    """

    def __init__(self, streams, encode, clock=time.monotonic):
        self.streams = dict(streams)
        self._encode = encode
        self._clock = clock
        self._lock = threading.Lock()
        self._encode_locks = defaultdict(threading.Lock)
        self._seq = itertools.count(1)
        self._viewers = {key: 0 for key in self.streams}
        self._frames = {}     # key -> (seq, received, message)
        self._encoded = {}    # key -> (seq, jpeg or None)
        self._errors = {}     # key -> last encode error text
        self._subscribed = set()

    def acquire(self, key):
        with self._lock:
            self._viewers[key] += 1

    def release(self, key):
        """One viewer fewer; the last one leaving drops the frame, so a later viewer never sees an old one."""
        with self._lock:
            self._viewers[key] = max(0, self._viewers[key] - 1)
            if self._viewers[key] == 0:
                self._frames.pop(key, None)
                self._encoded.pop(key, None)

    def wanted(self):
        with self._lock:
            return {key for key, count in self._viewers.items() if count > 0}

    def set_subscribed(self, keys):
        with self._lock:
            self._subscribed = set(keys)

    def update(self, key, message):
        """Keep the newest message; ignored while nobody watches (the node unsubscribes a moment later)."""
        with self._lock:
            if self._viewers[key] > 0:
                self._frames[key] = (next(self._seq), self._clock(), message)

    def latest_jpeg(self, key):
        """(seq, jpeg) of the newest frame; (None, None) before any; jpeg None when encoding it failed."""
        with self._lock:
            frame, cached = self._frames.get(key), self._encoded.get(key)
        if frame is None:
            return None, None
        seq, _, message = frame
        if cached is not None and cached[0] == seq:
            return cached
        with self._encode_locks[key]:
            with self._lock:
                cached = self._encoded.get(key)
            if cached is not None and cached[0] == seq:
                return cached
            try:
                jpeg = self._encode(message)
            except Exception as exc:   # a bad frame must not end the stream
                jpeg = None
                with self._lock:
                    self._errors[key] = f'{type(exc).__name__}: {exc}'
            with self._lock:
                self._encoded[key] = (seq, jpeg)
            return seq, jpeg

    def health(self):
        now = self._clock()
        with self._lock:
            return {f'{kind}/{camera}': {
                'topic': topic,
                'subscribed': (kind, camera) in self._subscribed,
                'viewers': self._viewers[(kind, camera)],
                'last_frame_age_s': (now - self._frames[(kind, camera)][1]) if (kind, camera) in self._frames else None,
                'error': self._errors.get((kind, camera)),
            } for (kind, camera), topic in self.streams.items()}
