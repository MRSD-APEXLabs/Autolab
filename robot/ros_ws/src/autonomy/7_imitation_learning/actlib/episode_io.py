"""Dataset storage: one HDF5 file per kept episode, plus index.csv and dataset.yaml.

    <dataset>/
      dataset.yaml              conventions, units, camera calibration (written once)
      index.csv                 one row per kept episode
      episodes/episode_0001.hdf5
      sessions/<date>.log       console log of every collect session, including discarded attempts

Episode file (all `t` arrays are seconds on the collecting machine's clock):
  attrs                       episode metadata (see write_episode)
  /frames/t                   camera capture time, Xavier clock converted with the measured offset
  /frames/t_xavier_ns         raw ZED image timestamp     /frames/t_recv  arrival time on this machine
  /frames/frame_id            Xavier grab counter (gaps = frames lost)
  /frames/cam_left|cam_right  JPEG bytes (variable length)   /frames/depth  PNG16 bytes, uint16 mm, 0 = invalid
  /robot/t, q_deg, dq_deg_s, tcp_pose, tau, state, mode     controller real-time report (~100 Hz)
  /control/t, cmd_vel, key_vel, state, mode, error, preset  what was commanded (mm/s, deg/s: vx vy vz wz)
  /gripper/t, setpoint, fsr                                 gripper state, ~5 Hz
  /aligned/t, qpos, tcp, tau, cmd_vel                       robot state interpolated at each camera frame time
"""

import csv
import datetime
import json
import os
import re
import subprocess
import threading
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import h5py
import numpy as np
import yaml

FORMAT_VERSION = 1
ZED_TS_FALLBACK_LATENCY_S = 0.068   # measured capture -> grab-return latency, used only if ZED timestamps look invalid
INDEX_COLUMNS = ["episode_id", "file", "created", "duration_s", "n_frames", "n_recorded", "cam_hz", "max_frame_gap_ms",
                 "frame_id_gaps", "robot_rows", "robot_hz", "faults", "clock_offset_ms", "size_mb", "notes"]


# --------------------------------------------------------------------------- paths / ids
def episode_files(dataset_dir: Path) -> List[Path]:
    return sorted((Path(dataset_dir) / "episodes").glob("episode_*.hdf5"))


def next_episode_id(dataset_dir: Path) -> int:
    ids = [int(re.search(r"episode_(\d+)", p.name).group(1)) for p in episode_files(dataset_dir)]
    return (max(ids) + 1) if ids else 1


def git_hash(path: Path) -> str:
    try:
        out = subprocess.run(["git", "-C", str(path), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=5)
        return out.stdout.strip() or "unknown"
    except Exception:
        return "unknown"


# --------------------------------------------------------------------------- time alignment
def frame_times(frames: List[dict], offset_pre: Tuple[float, float], offset_post: Tuple[float, float]):
    """Camera capture times on this machine's clock.

    offset_* = (labx time when measured, Xavier clock - labx clock). The offset is interpolated linearly across
    the episode (the two clocks slew against each other by ~1 ms/min). Returns (t, source).
    """
    t_img = np.array([f["t_image_ns"] for f in frames], dtype=np.float64) / 1e9
    t_done = np.array([f["t_grab_done_ns"] for f in frames], dtype=np.float64) / 1e9
    t_recv = np.array([f["t_recv"] for f in frames], dtype=np.float64)
    lat = t_done - t_img
    if np.all((lat > 0.0) & (lat < 1.0)):
        t_x, source = t_img, "zed_image_timestamp"
    else:  # ZED timestamps not on the Xavier's wall clock: fall back to the grab-return time
        t_x, source = t_done - ZED_TS_FALLBACK_LATENCY_S, "grab_done_minus_68ms"
    (tp, op), (tq, oq) = offset_pre, offset_post
    off = np.interp(t_recv, [tp, tq], [op, oq]) if tq > tp else np.full(len(frames), op)
    return t_x - off, source


def trim_to_span(frames: List[dict], t_cam: np.ndarray, t_lo: float, t_hi: float):
    """Keep only frames captured inside the robot log's time span, so aligned robot state is never extrapolated."""
    keep = (t_cam >= t_lo) & (t_cam <= t_hi)
    order = np.argsort(t_cam[keep], kind="stable")
    idx = np.flatnonzero(keep)[order]
    return [frames[i] for i in idx], t_cam[idx]


def dedupe_frames(frames: List[dict], t_cam: np.ndarray, robot_t: np.ndarray, robot_q: np.ndarray) -> np.ndarray:
    """Indices of the frames to keep. A frame is dropped when its joint angles (at its capture time) are EXACTLY the
    same as the previous frame's. Joint angles only: not the picture, not the timestamps, no tolerance. The first frame
    of each still stretch is kept. The robot log itself is never thinned."""
    n = len(frames)
    if n == 0:
        return np.zeros(0, dtype=int)
    q = _interp(t_cam, robot_t + np.arange(len(robot_t)) * 1e-9, robot_q)
    same_as_previous = (np.diff(q, axis=0) == 0).all(axis=1)
    return np.concatenate([[0], 1 + np.flatnonzero(~same_as_previous)]).astype(int)


def _interp(t_new: np.ndarray, t: np.ndarray, y: np.ndarray, angular_cols=()) -> np.ndarray:
    """Column-wise linear interpolation; angular columns (degrees) are unwrapped first so +-180 does not blend."""
    out = np.empty((len(t_new), y.shape[1]), dtype=np.float64)
    for c in range(y.shape[1]):
        col = y[:, c]
        if c in angular_cols:
            col = np.rad2deg(np.unwrap(np.deg2rad(col)))
            v = np.interp(t_new, t, col)
            out[:, c] = (v + 180.0) % 360.0 - 180.0
        else:
            out[:, c] = np.interp(t_new, t, col)
    return out


def _hold(t_new: np.ndarray, t: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Zero-order hold (last value at or before t_new)."""
    idx = np.clip(np.searchsorted(t, t_new, side="right") - 1, 0, len(t) - 1)
    return y[idx]


# --------------------------------------------------------------------------- statistics / warnings
def episode_stats(t_cam: np.ndarray, frame_ids: np.ndarray, robot_t: np.ndarray, faults: int) -> dict:
    n = len(t_cam)
    dur = float(t_cam[-1] - t_cam[0]) if n > 1 else 0.0
    gaps_ms = np.diff(t_cam) * 1e3 if n > 1 else np.zeros(1)
    rt_gaps = np.diff(robot_t) * 1e3 if len(robot_t) > 1 else np.zeros(1)
    return {
        "n_frames": n,
        "duration_s": dur,
        "cam_hz": (n - 1) / dur if dur > 0 else 0.0,
        "max_frame_gap_ms": float(gaps_ms.max()),
        "frame_id_gaps": int(np.sum(np.maximum(np.diff(frame_ids) - 1, 0))) if n > 1 else 0,
        "robot_rows": len(robot_t),
        "robot_hz": (len(robot_t) - 1) / (robot_t[-1] - robot_t[0]) if len(robot_t) > 1 and robot_t[-1] > robot_t[0] else 0.0,
        "max_robot_gap_ms": float(rt_gaps.max()),
        "faults": faults,
    }


def quality_warnings(st: dict, rec_cfg: dict) -> List[str]:
    w = []
    if st["n_frames"] < 2:
        return ["no camera frames were recorded"]
    if st["duration_s"] < rec_cfg["min_duration_s"]:
        w.append(f"short episode: {st['duration_s']:.1f} s")
    if st["frame_id_gaps"] > 0:
        w.append(f"{st['frame_id_gaps']} camera frames lost")
    if st["max_frame_gap_ms"] > rec_cfg["max_frame_gap_ms"]:
        w.append(f"camera gap of {st['max_frame_gap_ms']:.0f} ms")
    if st["cam_hz"] < rec_cfg["min_cam_hz"]:
        w.append(f"camera rate only {st['cam_hz']:.1f} Hz")
    if st["robot_rows"] < 2 or st["robot_hz"] < 80:
        w.append(f"robot state rate only {st['robot_hz']:.0f} Hz")
    if st["max_robot_gap_ms"] > 50:
        w.append(f"robot state gap of {st['max_robot_gap_ms']:.0f} ms")
    if st["faults"]:
        w.append(f"{st['faults']} arm fault/re-arm event(s) during the episode")
    return w


# --------------------------------------------------------------------------- writing
def _put_vlen(grp: h5py.Group, name: str, blobs: List[bytes]):
    ds = grp.create_dataset(name, (len(blobs),), dtype=h5py.vlen_dtype(np.dtype("uint8")))
    for i, b in enumerate(blobs):
        ds[i] = np.frombuffer(b, dtype=np.uint8)


def write_episode(dataset_dir: Path, episode_id: int, frames: List[dict], t_cam: np.ndarray, ts_source: str,
                  robot: Dict[str, np.ndarray], control: Dict[str, np.ndarray], gripper: Dict[str, np.ndarray],
                  events: List[Tuple[float, str]], offset_pre: Tuple[float, float, float],
                  offset_post: Tuple[float, float, float], meta: dict, stats: dict) -> Path:
    """frames/t_cam come from frame_times() + trim_to_span(). offset_* = (labx time, Xavier - labx offset seconds,
    round-trip ms). Returns the file path."""
    dataset_dir = Path(dataset_dir)
    (dataset_dir / "episodes").mkdir(parents=True, exist_ok=True)
    path = dataset_dir / "episodes" / f"episode_{episode_id:04d}.hdf5"
    tmp = path.with_suffix(".hdf5.tmp")

    rt, q, tcp, tau = robot["t"], robot["q"], robot["tcp"], robot["tau"]
    tt = rt + np.arange(len(rt)) * 1e-9          # strictly increasing for gradient / interp
    dq = np.gradient(q, tt, axis=0) if len(rt) > 1 else np.zeros_like(q)
    aligned = {
        "qpos": _interp(t_cam, tt, q),
        "tcp": _interp(t_cam, tt, tcp, angular_cols=(3, 4, 5)),
        "tau": _interp(t_cam, tt, tau),
        "cmd_vel": _hold(t_cam, control["t"], control["cmd"]) if len(control["t"]) else np.zeros((len(t_cam), 4)),
    }

    # depth is already PNG16 from the Xavier: store as is (no re-encode)
    with h5py.File(tmp, "w") as f:
        f.attrs["format_version"] = FORMAT_VERSION
        f.attrs["episode_id"] = episode_id
        f.attrs["created"] = meta["created"]
        f.attrs["dataset"] = meta["dataset"]
        f.attrs["task"] = meta["task"]
        f.attrs["git_hash"] = meta["git_hash"]
        f.attrs["collect_config"] = meta["collect_config"]
        f.attrs["warnings"] = meta.get("warnings", "")
        f.attrs["timestamp_source"] = ts_source
        f.attrs["clock_offset_pre_s"], f.attrs["clock_offset_post_s"] = offset_pre[1], offset_post[1]
        f.attrs["clock_rtt_pre_ms"], f.attrs["clock_rtt_post_ms"] = offset_pre[2], offset_post[2]
        f.attrs["clock_offset_note"] = "Xavier clock minus this machine's clock; frames/t already corrected"
        f.attrs["image_size_wh"] = frames[0]["image_size"]
        f.attrs["image_scale"] = frames[0]["scale"]
        f.attrs["depth_codec"] = frames[0]["depth_codec"]
        f.attrs["events"] = json.dumps([[round(t, 4), s] for t, s in events])
        for k, v in stats.items():
            f.attrs[f"stat_{k}"] = v

        g = f.create_group("frames")
        g["t"] = t_cam
        g["t_xavier_ns"] = np.array([fr["t_image_ns"] for fr in frames], dtype=np.int64)
        g["t_recv"] = np.array([fr["t_recv"] for fr in frames], dtype=np.float64)
        g["frame_id"] = np.array([fr["frame_id"] for fr in frames], dtype=np.int64)
        _put_vlen(g, "cam_left", [fr["left"] for fr in frames])
        _put_vlen(g, "cam_right", [fr["right"] for fr in frames])
        _put_vlen(g, "depth", [fr["depth"] for fr in frames])

        g = f.create_group("robot")
        g["t"], g["q_deg"], g["dq_deg_s"], g["tcp_pose"], g["tau"] = rt, q, dq, tcp, tau
        g["state"], g["mode"] = robot["state"], robot["mode"]
        g.attrs["dq_note"] = "finite difference of q_deg (deg/s); the controller reports joint speeds at 5 Hz only"

        g = f.create_group("control")
        g["t"], g["cmd_vel"], g["key_vel"] = control["t"], control["cmd"], control["key"]
        g["state"], g["mode"], g["error"], g["preset"] = control["state"], control["mode"], control["error"], control["preset"]
        g.attrs["units"] = "vx vy vz mm/s (base frame), wz deg/s about base +Z (= last-joint rotation with the tool pointing down)"

        g = f.create_group("gripper")
        g["t"], g["setpoint"], g["fsr"] = gripper["t"], gripper["setpoint"], gripper["fsr"]

        g = f.create_group("aligned")
        g["t"] = t_cam
        for k, v in aligned.items():
            g[k] = v
    os.replace(tmp, path)
    return path


_INDEX_LOCK = threading.Lock()


def _index_row(path: Path) -> dict:
    with h5py.File(path, "r") as f:
        a = f.attrs
        n = int(f["frames/t"].shape[0])
        return {
            "episode_id": int(a["episode_id"]), "file": path.name, "created": str(a["created"]),
            "duration_s": round(float(a["stat_saved_duration_s"] if "stat_saved_duration_s" in a else a["stat_duration_s"]), 2), "n_frames": n,
            "n_recorded": int(a["stat_n_recorded"]) if "stat_n_recorded" in a else n,
            "cam_hz": round(float(a["stat_cam_hz"]), 2), "max_frame_gap_ms": round(float(a["stat_max_frame_gap_ms"]), 1),
            "frame_id_gaps": int(a["stat_frame_id_gaps"]), "robot_rows": int(a["stat_robot_rows"]),
            "robot_hz": round(float(a["stat_robot_hz"]), 1), "faults": int(a["stat_faults"]),
            "clock_offset_ms": round(float(a["clock_offset_post_s"]) * 1e3, 2),
            "size_mb": round(path.stat().st_size / 1e6, 1), "notes": str(a["warnings"]) if "warnings" in a else "",
        }


def index_rows(dataset_dir: Path) -> List[dict]:
    """One row per episode file that exists right now, read from the files themselves."""
    rows = []
    for p in episode_files(dataset_dir):
        try:
            rows.append(_index_row(p))
        except Exception:  # a half-written or foreign file is simply not listed
            continue
    return rows


def rebuild_index(dataset_dir: Path) -> int:
    """index.csv is derived from the episode files, never edited by hand: delete an episode file and its row disappears
    at the next rebuild; a re-recorded number gets exactly one row. Written atomically."""
    rows = index_rows(dataset_dir)
    p = Path(dataset_dir) / "index.csv"
    with _INDEX_LOCK:
        tmp = p.with_suffix(".csv.tmp")
        with open(tmp, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=INDEX_COLUMNS)
            w.writeheader()
            w.writerows(rows)
        os.replace(tmp, p)
    return len(rows)


def ensure_dataset_yaml(dataset_dir: Path, task: str, calibration: Optional[dict], scale: float, cfg: dict, keys_doc: dict):
    p = Path(dataset_dir) / "dataset.yaml"
    if p.exists():
        return
    doc = {
        "format_version": FORMAT_VERSION,
        "task": task,
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "robot": "UFactory xArm6, custom parallel gripper (gripper state logged, not commanded)",
        "camera": "ZED X wrist camera (left + right eye, depth aligned to LEFT)",
        "image_scale_of_native": scale,
        "calibration_native": calibration,
        "calibration_note": "intrinsics are for the native 960x600; multiply fx fy cx cy by image_scale_of_native for stored images",
        "units": {"joints": "deg", "tcp_pose": "x y z mm, roll pitch yaw deg (xArm base frame)", "depth": "uint16 mm, 0 = invalid",
                  "velocity": "mm/s and deg/s", "torque": "controller-estimated joint torque (N*m)", "time": "s, collecting machine clock"},
        "teleop_keys": keys_doc,
        "collect_config": cfg,
    }
    with open(p, "w") as fh:
        yaml.safe_dump(doc, fh, sort_keys=False)


# --------------------------------------------------------------------------- reading (used by inspect_episode and training)
def decode_image(b: np.ndarray) -> np.ndarray:
    return cv2.imdecode(np.asarray(b, dtype=np.uint8), cv2.IMREAD_COLOR)


def decode_depth(b: np.ndarray) -> np.ndarray:
    return cv2.imdecode(np.asarray(b, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
