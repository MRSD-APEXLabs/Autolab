"""Web viewer for recorded episodes: left / right / depth video with joint angles, TCP and commands, in sync.

    ./view.sh                              # data/wellplate_place on http://localhost:8090
    ./view.sh data/other_dataset --port 8091

Read-only. Left / right frames are served exactly as stored (JPEG); depth (PNG16, mm) is colourised on the fly with
the same colour scale as inspect_episode.py. Runs as its own process: collect.py starts it next to a session.
"""

import argparse
import collections
import json
import re
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import h5py
import numpy as np

from .episode_io import decode_depth, episode_files
from .inspect_episode import depth_vis

PROJECT = Path(__file__).resolve().parents[1]
STATIC_DIR = Path(__file__).with_name("viewer_static")
MAX_PLOT_POINTS = 4000
DEPTH_CACHE_FRAMES = 400


def _rounded(a, nd=3):
    return np.round(np.asarray(a, dtype=np.float64), nd).tolist()


def _stride(n: int) -> slice:
    return slice(None, None, max(1, int(np.ceil(n / MAX_PLOT_POINTS))))


class Store:
    """Open-file cache + JSON / JPEG builders for one dataset directory."""

    def __init__(self, root: Path):
        self.root = Path(root)
        self._lock = threading.Lock()
        self._open = {}
        self._depth = collections.OrderedDict()

    def _file(self, ep: int) -> h5py.File:
        p = self.root / "episodes" / f"episode_{ep:04d}.hdf5"
        if not p.exists():
            raise KeyError(ep)
        m = p.stat().st_mtime
        with self._lock:
            cur = self._open.get(ep)
            if cur is None or cur[0] != m:
                if cur is not None:
                    cur[1].close()
                cur = (m, h5py.File(p, "r"))
                self._open[ep] = cur
            return cur[1]

    def episodes(self):
        out = []
        for p in episode_files(self.root):
            ep = int(re.search(r"episode_(\d+)", p.name).group(1))
            try:
                f = self._file(ep)
                a = f.attrs
                n = int(f["frames/t"].shape[0])
                out.append({"id": ep, "created": str(a["created"]), "duration_s": float(a["stat_saved_duration_s"] if "stat_saved_duration_s" in a else a["stat_duration_s"]), "n_frames": n,
                            "n_recorded": int(a["stat_n_recorded"]) if "stat_n_recorded" in a else n,
                            "size_mb": round(p.stat().st_size / 1e6, 1), "faults": int(a["stat_faults"])})
            except Exception as e:  # a half-written or foreign file must not break the list
                out.append({"id": ep, "error": str(e)})
        return out

    def meta(self, ep: int) -> dict:
        f = self._file(ep)
        a = f.attrs
        rt = f["robot/t"][:]
        t0 = float(rt[0])
        rs = _stride(len(rt))
        q, tcp = f["robot/q_deg"][:][rs], f["robot/tcp_pose"][:][rs]
        ct = f["control/t"][:]
        cs = _stride(len(ct))
        cmd = f["control/cmd_vel"][:][cs]
        gt = f["gripper/t"][:]
        fsr, sp = f["gripper/fsr"][:], f["gripper/setpoint"][:]
        n = int(f["frames/t"].shape[0])
        return {
            "id": ep,
            "task": str(a["task"]),
            "created": str(a["created"]),
            "duration": float(rt[-1] - t0),
            "info": {
                "duration_s": float(a["stat_saved_duration_s"] if "stat_saved_duration_s" in a else a["stat_duration_s"]), "kept": n,
                "recorded": int(a["stat_n_recorded"]) if "stat_n_recorded" in a else n,
                "cam_hz": float(a["stat_cam_hz"]), "max_frame_gap_ms": float(a["stat_max_frame_gap_ms"]),
                "frames_lost": int(a["stat_frame_id_gaps"]), "robot_hz": float(a["stat_robot_hz"]),
                "faults": int(a["stat_faults"]), "clock_offset_ms": [float(a["clock_offset_pre_s"]) * 1e3, float(a["clock_offset_post_s"]) * 1e3],
                "timestamps": str(a["timestamp_source"]), "image": [int(x) for x in a["image_size_wh"]],
                "warnings": str(a["warnings"]) if "warnings" in a else "",
            },
            "frames": {"t": _rounded(f["frames/t"][:] - t0, 4), "frame_id": f["frames/frame_id"][:].tolist()},
            "robot": {"t": _rounded(rt[rs] - t0, 4), "q": [_rounded(q[:, j], 3) for j in range(6)],
                      "tcp": [_rounded(tcp[:, j], 2) for j in range(3)]},
            "control": {"t": _rounded(ct[cs] - t0, 4), "cmd": [_rounded(cmd[:, j], 2) for j in range(4)]},
            "gripper": {"t": _rounded(gt - t0, 3), "setpoint": sp.tolist(), "fsr": [fsr[:, 0].tolist(), fsr[:, 1].tolist()]},
            "events": [[round(e[0] - t0, 3), e[1]] for e in json.loads(a["events"])],
        }

    def frame(self, ep: int, i: int, kind: str) -> bytes:
        f = self._file(ep)
        if not 0 <= i < f["frames/t"].shape[0]:
            raise IndexError(i)
        if kind in ("left", "right"):
            return f[f"frames/cam_{kind}"][i].tobytes()
        key = (ep, i)
        with self._lock:
            hit = self._depth.get(key)
            if hit is not None:
                self._depth.move_to_end(key)
                return hit
        jpg = cv2.imencode(".jpg", depth_vis(decode_depth(f["frames/depth"][i])), [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
        with self._lock:
            self._depth[key] = jpg
            while len(self._depth) > DEPTH_CACHE_FRAMES:
                self._depth.popitem(last=False)
        return jpg


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    store: Store = None

    def log_message(self, *args):
        pass

    def _send(self, code: int, body: bytes, ctype: str, cache: bool = False):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "max-age=3600" if cache else "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urllib.parse.urlparse(self.path).path
        try:
            if path in ("/", "/index.html"):
                return self._send(200, (STATIC_DIR / "index.html").read_bytes(), "text/html; charset=utf-8")
            if path == "/api/episodes":
                return self._send(200, json.dumps(self.store.episodes()).encode(), "application/json")
            m = re.fullmatch(r"/api/episode/(\d+)", path)
            if m:
                return self._send(200, json.dumps(self.store.meta(int(m[1]))).encode(), "application/json")
            m = re.fullmatch(r"/frame/(\d+)/(\d+)/(left|right|depth)\.jpg", path)
            if m:
                return self._send(200, self.store.frame(int(m[1]), int(m[2]), m[3]), "image/jpeg", cache=True)
            self._send(404, b"not found", "text/plain")
        except (KeyError, IndexError):
            self._send(404, b"no such episode or frame", "text/plain")
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:
            try:
                self._send(500, str(e).encode(), "text/plain")
            except Exception:
                pass


def main(argv=None):
    ap = argparse.ArgumentParser(description="Browse recorded ACT episodes in a web page.")
    ap.add_argument("dataset", nargs="?", default=str(PROJECT / "data" / "wellplate_place"), help="dataset directory")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--host", default="127.0.0.1", help="use 0.0.0.0 to open it from another machine on the lab network")
    args = ap.parse_args(argv)
    root = Path(args.dataset)
    if not (root / "episodes").is_dir():
        raise SystemExit(f"no episodes folder in {root}")
    handler = type("H", (Handler,), {"store": Store(root)})
    server = ThreadingHTTPServer((args.host, args.port), handler)
    server.daemon_threads = True
    server.request_queue_size = 128
    print(f"viewing {root} at http://{'localhost' if args.host == '127.0.0.1' else args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
