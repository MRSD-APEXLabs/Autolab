"""Record mode for the Camera-Edge server (ACT data collection).

A long-lived worker (same start_run / abort_run pattern as ZEDInspectWorker) that grabs the wrist
ZED X with depth enabled and publishes one binary message per frame:

    [u32 little-endian header_len][JSON header][left JPEG][right JPEG][depth PNG16, uint16 mm, 0 = invalid]

Frames are stamped at capture (ZED image timestamp), so streaming latency never shifts the data.
Images and depth are optionally downscaled before streaming (`scale`) and depth is always lossless PNG16:
the Xavier's Ethernet link can negotiate only 100 Mb/s, and full-size raw frames are 1.7 MB each.
Messages go into a bounded FIFO (RecordFeed) and every websocket client drains it in order, so a short
network stall is absorbed instead of dropping frames. If the FIFO overflows the oldest frames are
discarded and the client sees the gap in `frame_id`.

Runs on the Xavier (system python 3.8) -- keep the syntax 3.8-compatible.
"""

import asyncio
import base64
import collections
import json
import logging
import queue
import struct
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import pyzed.sl as sl
import websockets

from cam_worker import CameraWorker
from config import RECORD_JPEG_QUALITY, RECORD_SCALE
from streamhub import StreamHub

logger = logging.getLogger(__name__)

FEED_MAX_FRAMES = 45        # ~1.5 s of frames at 30 Hz: absorbs stalls without adding seconds of latency
ENCODE_QUEUE_DEPTH = 4
RAW_REPUBLISH_EVERY = 6     # ~5 Hz to the GCS "wrist_raw" tile (the idle worker's wrist grab is paused)
STATS_INTERVAL_S = 2.0


class RecordFeed:
    """Bounded FIFO of encoded frames shared by the worker (producer) and websocket clients."""

    def __init__(self, maxlen: int = FEED_MAX_FRAMES):
        self._lock = threading.Lock()
        self._frames = collections.deque(maxlen=maxlen)
        self._seq = 0
        self.calibration = None

    def publish(self, payload: bytes) -> int:
        with self._lock:
            self._seq += 1
            self._frames.append((self._seq, payload))
            return self._seq

    def latest_seq(self) -> int:
        with self._lock:
            return self._seq

    def since(self, last_seq: int):
        """Frames newer than last_seq, oldest first."""
        with self._lock:
            return [f for f in self._frames if f[0] > last_seq]


def _calibration_dict(camera) -> dict:
    """Intrinsics for both eyes + baseline, sent once to each client. Never raises."""
    out = {"resolution": None, "fps": None, "baseline_mm": None, "left": None, "right": None,
           "depth": {"unit": "mm", "dtype": "uint16", "invalid": 0, "aligned_to": "left"}}  # intrinsics are at native size
    try:
        cfg = camera.get_camera_information().camera_configuration
        out["resolution"] = [int(cfg.resolution.width), int(cfg.resolution.height)]
        out["fps"] = float(cfg.fps)
        cal = cfg.calibration_parameters

        def eye(c):
            return {"fx": float(c.fx), "fy": float(c.fy), "cx": float(c.cx), "cy": float(c.cy),
                    "disto": [float(x) for x in c.disto]}

        out["left"], out["right"] = eye(cal.left_cam), eye(cal.right_cam)
        try:
            out["baseline_mm"] = float(cal.get_camera_baseline())  # wrist camera uses the default mm units
        except Exception:
            out["baseline_mm"] = abs(float(cal.stereo_transform.get_translation().get()[0]))
    except Exception as e:  # calibration is informational; never break server start-up over it
        logger.warning("record: calibration unavailable: %s", e)
    return out


class ZEDRecordWorker(CameraWorker):
    """Grab wrist left/right/depth and publish encoded frames while record mode is active."""

    def __init__(self, camera, grab_lock: threading.Lock, feed: RecordFeed, stream_hub: StreamHub):
        super().__init__("record", camera, grab_lock)
        self.feed = feed
        self.stream_hub = stream_hub
        self.runtime = sl.RuntimeParameters()
        self.runtime.enable_depth = True
        self.left_mat = sl.Mat()
        self.right_mat = sl.Mat()
        self.depth_mat = sl.Mat()
        self._run_event = threading.Event()
        self._on_run_complete = None
        self._enc_q = queue.Queue(maxsize=ENCODE_QUEUE_DEPTH)
        self._encoder = None
        self.dropped_encode = 0
        self._enc_ms = 0.0
        self._enc_n = 0
        self._cfg_lock = threading.Lock()
        self._cfg = {"scale": float(RECORD_SCALE), "jpeg_quality": int(RECORD_JPEG_QUALITY)}
        self._pool = ThreadPoolExecutor(max_workers=3, thread_name_prefix="record-enc")
        feed.calibration = _calibration_dict(camera)

    def configure(self, scale=None, jpeg_quality=None) -> dict:
        """Change the streamed image size / JPEG quality (takes effect on the next frame)."""
        with self._cfg_lock:
            if scale is not None:
                scale = float(scale)
                if not 0.25 <= scale <= 1.0:
                    raise ValueError("scale must be within [0.25, 1.0]")
                self._cfg["scale"] = scale
            if jpeg_quality is not None:
                quality = int(jpeg_quality)
                if not 50 <= quality <= 100:
                    raise ValueError("jpeg_quality must be within [50, 100]")
                self._cfg["jpeg_quality"] = quality
            return dict(self._cfg)

    # -------------------------------------------------------------------------
    # State machine control (same contract as the inspect / servo workers)
    # -------------------------------------------------------------------------

    def start_run(self, on_complete=None):
        self._on_run_complete = on_complete
        self._run_event.set()

    def abort_run(self):
        self._run_event.clear()

    def stop(self):
        self.stop_event.set()
        self._run_event.set()  # unblock wait so the thread can exit
        self._pool.shutdown(wait=False)

    def should_stop(self) -> bool:
        return not self._run_event.is_set() or self.stop_event.is_set()

    # -------------------------------------------------------------------------
    # Encoding (own thread: cv2 releases the GIL, so this overlaps the next grab)
    # -------------------------------------------------------------------------

    def _encode_frame(self, item, cfg):
        frame_id, t_img_ns, t_done_ns, left, right, depth = item
        scale, quality = cfg["scale"], cfg["jpeg_quality"]
        h0, w0 = left.shape[:2]
        w, h = (w0, h0) if scale > 0.999 else (int(round(w0 * scale)), int(round(h0 * scale)))

        def jpeg(img):
            bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            if (w, h) != (w0, h0):
                bgr = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
            if not ok:
                raise RuntimeError("jpeg encode failed")
            return buf.tobytes()

        def png16():
            d = np.squeeze(np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0))
            np.clip(d, 0.0, 65535.0, out=d)
            d16 = d.astype(np.uint16)
            if (w, h) != (w0, h0):  # nearest: never blends invalid (0) pixels into valid depth
                d16 = cv2.resize(d16, (w, h), interpolation=cv2.INTER_NEAREST)
            ok, buf = cv2.imencode(".png", d16, [int(cv2.IMWRITE_PNG_COMPRESSION), 1])
            if not ok:
                raise RuntimeError("png encode failed")
            return buf.tobytes()

        fl, fr, fd = self._pool.submit(jpeg, left), self._pool.submit(jpeg, right), self._pool.submit(png16)
        left_b, right_b, depth_b = fl.result(), fr.result(), fd.result()
        header = {
            "type": "record_frame",
            "frame_id": frame_id,
            "t_image_ns": t_img_ns,          # ZED capture time (Xavier clock)
            "t_grab_done_ns": t_done_ns,     # Xavier clock when grab+retrieve returned
            "t_pub_ns": time.time_ns(),
            "image_size": [w, h],
            "scale": scale,
            "jpeg_quality": quality,
            "depth_codec": "png16",
            "depth_shape": [h, w],
            "left_len": len(left_b),
            "right_len": len(right_b),
            "depth_len": len(depth_b),
        }
        hb = json.dumps(header, separators=(",", ":")).encode("utf-8")
        self.feed.publish(struct.pack("<I", len(hb)) + hb + left_b + right_b + depth_b)
        if frame_id % RAW_REPUBLISH_EVERY == 0:
            self.stream_hub.publish(
                "wrist_raw",
                {"type": "raw_frame", "camera": "wrist",
                 "image_jpeg_b64": base64.b64encode(right_b).decode("ascii")},
            )

    def _encode_loop(self):
        while True:
            item = self._enc_q.get()
            if item is None:
                return
            try:
                t0 = time.time()
                with self._cfg_lock:
                    cfg = dict(self._cfg)
                self._encode_frame(item, cfg)
                self._enc_ms += (time.time() - t0) * 1e3
                self._enc_n += 1
            except Exception as e:
                self.dropped_encode += 1
                logger.exception("record encode failed: %s", e)

    def _start_encoder(self):
        while not self._enc_q.empty():  # drop leftovers from an aborted run
            try:
                self._enc_q.get_nowait()
            except queue.Empty:
                break
        self._encoder = threading.Thread(target=self._encode_loop, name="record-encode", daemon=True)
        self._encoder.start()

    def _stop_encoder(self):
        try:
            self._enc_q.put(None, timeout=2.0)
        except queue.Full:
            pass
        if self._encoder is not None:
            self._encoder.join(timeout=3.0)
            self._encoder = None

    # -------------------------------------------------------------------------
    # Grab loop
    # -------------------------------------------------------------------------

    def _execute_run(self):
        logger.info("record run starting")
        self.dropped_encode = 0
        self._enc_ms, self._enc_n = 0.0, 0
        self._start_encoder()
        frame_id = 0
        n, grab_ms, t_report = 0, 0.0, time.time()
        try:
            while not self.should_stop():
                t0 = time.time()
                with self.grab_lock:
                    err = self.camera.grab(self.runtime)
                    if err == sl.ERROR_CODE.SUCCESS:
                        self.camera.retrieve_image(self.left_mat, sl.VIEW.LEFT)
                        self.camera.retrieve_image(self.right_mat, sl.VIEW.RIGHT)
                        self.camera.retrieve_measure(self.depth_mat, sl.MEASURE.DEPTH)
                        t_img_ns = self.camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
                if err != sl.ERROR_CODE.SUCCESS:
                    time.sleep(0.001)
                    continue
                t_done_ns = time.time_ns()
                grab_ms += (time.time() - t0) * 1e3
                frame_id += 1
                item = (frame_id, t_img_ns, t_done_ns,
                        self.left_mat.get_data().copy(),
                        self.right_mat.get_data().copy(),
                        self.depth_mat.get_data().copy())
                try:
                    self._enc_q.put_nowait(item)
                except queue.Full:
                    self.dropped_encode += 1  # shows up as a frame_id gap on the client
                n += 1
                now = time.time()
                if now - t_report >= STATS_INTERVAL_S:
                    logger.info(
                        "record  fps=%.1f | grab+retrieve=%.1fms encode=%.1fms | encode_drops=%d",
                        n / (now - t_report), grab_ms / max(n, 1),
                        self._enc_ms / max(self._enc_n, 1), self.dropped_encode)
                    n, grab_ms, t_report = 0, 0.0, now
                    self._enc_ms, self._enc_n = 0.0, 0
        finally:
            self._stop_encoder()
        logger.info("record run finished")

    def _run(self):
        logger.info("record worker ready")
        while not self.stop_event.is_set():
            if not self._run_event.wait(timeout=0.5):
                continue
            if self.stop_event.is_set():
                break
            try:
                self._execute_run()
            except Exception as e:
                logger.exception("record worker crashed: %s", e)
                self.exc = e
            finally:
                self._run_event.clear()
                if self._on_run_complete:
                    self._on_run_complete()


async def handle_record_client(ws, feed: RecordFeed):
    """Stream RecordFeed frames to one client in order (calibration first, as text JSON)."""
    if feed.calibration is not None:
        await ws.send(json.dumps(dict(feed.calibration, type="calibration")))
    last_seq = feed.latest_seq()  # a new client starts from "now", not from the 3 s backlog
    try:
        while True:
            batch = feed.since(last_seq)
            if not batch:
                await asyncio.sleep(0.003)
                continue
            for seq, payload in batch:
                await ws.send(payload)
                last_seq = seq
    except websockets.exceptions.ConnectionClosed:
        pass
