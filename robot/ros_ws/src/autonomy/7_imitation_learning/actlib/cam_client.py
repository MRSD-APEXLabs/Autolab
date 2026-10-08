"""Client for the Xavier's record mode.

  XavierControl  short-lived synchronous connections to the control port (:8765): mode switch, status,
                 and the clock probe used to convert Xavier capture times to this machine's clock.
  FrameStream    background receiver for the record port (:8768). Frames are only kept while recording;
                 rate / gap statistics are always live.

A record frame is one binary message: [u32 LE header_len][JSON header][left JPEG][right JPEG][depth PNG16 uint16 mm, 0 = invalid].
Frames are stamped on the Xavier at capture, so streaming latency does not shift the recorded data.
"""

import asyncio
import collections
import json
import logging
import struct
import threading
import time
from typing import List, Optional, Tuple

import websockets
from websockets.sync.client import connect as connect_sync

log = logging.getLogger("act.cam")


class XavierControl:
    def __init__(self, host: str, port: int = 8765, timeout: float = 5.0):
        self.uri = f"ws://{host}:{port}"
        self.timeout = timeout

    def _open(self):
        ws = connect_sync(self.uri, open_timeout=self.timeout, max_size=None, compression=None)
        ws.recv(timeout=self.timeout)  # greeting: {"ok": true, "message": "connected"}
        return ws

    def call(self, msg: dict) -> dict:
        with self._open() as ws:
            ws.send(json.dumps(msg))
            reply = json.loads(ws.recv(timeout=self.timeout))
        if not reply.get("ok"):
            raise RuntimeError(f"Xavier control {msg} failed: {reply}")
        return reply

    def status(self) -> dict:
        return self.call({"cmd": "status"})

    def set_mode(self, mode: str) -> str:
        return self.call({"cmd": "mode", "mode": mode})["mode"]

    def configure_record(self, scale: float, jpeg_quality: int) -> dict:
        """Set the streamed image size (fraction of native) and JPEG quality on the Xavier."""
        return self.call({"cmd": "record_config", "scale": scale, "jpeg_quality": jpeg_quality})["record"]

    def measure_offset(self, probes: int = 20, spacing_s: float = 0.01) -> Tuple[float, float]:
        """(Xavier clock - this clock in seconds, best round-trip in ms), from the lowest-RTT probe."""
        best = None
        with self._open() as ws:
            for _ in range(probes):
                t0 = time.time_ns()
                ws.send('{"cmd":"time"}')
                reply = json.loads(ws.recv(timeout=self.timeout))
                t1 = time.time_ns()
                if not reply.get("ok"):
                    continue
                rtt = t1 - t0
                offset = reply["t_ns"] - (t0 + t1) / 2.0
                if best is None or rtt < best[0]:
                    best = (rtt, offset)
                time.sleep(spacing_s)
        if best is None:
            raise RuntimeError("clock probe got no reply from the Xavier")
        return best[1] / 1e9, best[0] / 1e6


class FrameStream:
    def __init__(self, host: str, port: int = 8768):
        self.uri = f"ws://{host}:{port}"
        self.calibration: Optional[dict] = None
        self._lock = threading.Lock()
        self._frames: List[dict] = []
        self._recording = False
        self._stop = False
        self._connected = threading.Event()
        self._recent = collections.deque(maxlen=90)  # capture times (s, Xavier clock) of the latest frames
        self._last_recv = 0.0
        self._last_id: Optional[int] = None
        self._loop = None
        self._task = None
        self.gaps = 0     # frame_id gaps seen while recording (frames lost on the Xavier or in transit)
        self.total = 0
        self._thread = threading.Thread(target=lambda: asyncio.run(self._main()), name="cam-recv", daemon=True)

    # ------------------------------------------------------------------ lifecycle
    def start(self, timeout: float = 15.0):
        self._thread.start()
        if not self._connected.wait(timeout):
            raise RuntimeError(f"could not connect to the Xavier record port {self.uri}")
        t_end = time.time() + timeout
        while self.calibration is None and time.time() < t_end:
            time.sleep(0.05)

    def stop(self):
        self._stop = True
        if self._loop is not None and self._task is not None:
            self._loop.call_soon_threadsafe(self._task.cancel)
        self._thread.join(timeout=3.0)

    async def _main(self):
        self._loop = asyncio.get_running_loop()
        self._task = asyncio.current_task()
        while not self._stop:
            try:
                async with websockets.connect(self.uri, max_size=None, compression=None,
                                              ping_interval=None, open_timeout=5) as ws:
                    self._connected.set()
                    async for msg in ws:
                        self._handle(msg, time.time())
            except asyncio.CancelledError:
                return
            except Exception as e:
                log.warning("record stream: %s (%s) - retrying", type(e).__name__, e)
            self._connected.clear()
            with self._lock:
                self._last_id = None
            if not self._stop:
                await asyncio.sleep(1.0)

    # ------------------------------------------------------------------ receive
    def _handle(self, msg, t_recv: float):
        if isinstance(msg, str):
            d = json.loads(msg)
            if d.get("type") == "calibration":
                self.calibration = d
            return
        (hl,) = struct.unpack_from("<I", msg, 0)
        h = json.loads(bytes(msg[4:4 + hl]))
        fid = h["frame_id"]
        frame = None
        with self._lock:
            self.total += 1
            if t_recv - self._last_recv > 1.0:
                self._recent.clear()  # the stream restarted after a pause: do not average across the gap
            self._last_recv = t_recv
            self._recent.append(h["t_image_ns"] / 1e9)
            if self._recording:
                if self._last_id is not None and fid > self._last_id + 1:
                    self.gaps += fid - self._last_id - 1
                o = 4 + hl
                ll, rl, dl = h["left_len"], h["right_len"], h["depth_len"]
                frame = {
                    "frame_id": fid, "t_image_ns": h["t_image_ns"], "t_grab_done_ns": h["t_grab_done_ns"],
                    "t_pub_ns": h["t_pub_ns"], "t_recv": t_recv, "image_size": h["image_size"],
                    "scale": h["scale"], "depth_codec": h["depth_codec"],
                    "left": bytes(msg[o:o + ll]), "right": bytes(msg[o + ll:o + ll + rl]),
                    "depth": bytes(msg[o + ll + rl:o + ll + rl + dl]),
                }
                self._frames.append(frame)
            self._last_id = fid

    # ------------------------------------------------------------------ recording + stats
    def begin_recording(self):
        with self._lock:
            self._frames = []
            self._recording = True
            self._last_id = None
            self.gaps = 0

    def end_recording(self) -> List[dict]:
        with self._lock:
            self._recording = False
            frames, self._frames = self._frames, []
        return frames

    def recorded_count(self) -> int:
        with self._lock:
            return len(self._frames)

    def hz(self) -> float:
        """Capture-time frame rate over the most recent frames; 0 if the stream has gone quiet."""
        with self._lock:
            if len(self._recent) < 2 or time.time() - self._last_recv > 1.0:
                return 0.0
            span = self._recent[-1] - self._recent[0]
            return (len(self._recent) - 1) / span if span > 0 else 0.0
