"""Episode dataset for ACT: (observation at frame t) -> (next `chunk_size` actions).

Reads the `aligned/` arrays of every kept episode (robot state interpolated at each camera frame) and decodes the
JPEG / PNG16 frames lazily from the HDF5 files.

Inputs (any subset):  cam_left  cam_right  depth  joints  tcp
Actions:              joint_abs [q1..q6 deg] | tcp_abs [x y z yaw_rel] | cmd_vel [vx vy vz wz]
"""

import json
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import h5py
import numpy as np
import torch
from torch.utils.data import Dataset

from .episode_io import decode_depth, decode_image, episode_files

IMAGE_INPUTS = ("cam_left", "cam_right", "depth")
LOWDIM_INPUTS = ("joints", "tcp")
ALL_INPUTS = IMAGE_INPUTS + LOWDIM_INPUTS
ACTIONS = ("tcp_abs", "joint_abs", "cmd_vel")
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def load_episode_arrays(path: Path) -> Dict[str, np.ndarray]:
    with h5py.File(path) as f:
        return {"qpos": f["aligned/qpos"][:].astype(np.float32), "tcp": f["aligned/tcp"][:].astype(np.float64),
                "cmd_vel": f["aligned/cmd_vel"][:].astype(np.float32), "n": int(f["frames/t"].shape[0])}


def tcp_features(tcp: np.ndarray, yaw_ref: float) -> np.ndarray:
    """[x y z yaw_rel] with yaw expressed relative to a dataset reference, wrapped once and then unwrapped within the
    episode so it is continuous (raw yaw jumps at +-180)."""
    yaw = np.rad2deg(np.unwrap(np.deg2rad(wrap180(tcp[:, 5] - yaw_ref))))
    return np.column_stack([tcp[:, 0], tcp[:, 1], tcp[:, 2], yaw]).astype(np.float32)


def action_array(kind: str, ep: Dict[str, np.ndarray], yaw_ref: float) -> np.ndarray:
    if kind == "joint_abs":
        return ep["qpos"]
    if kind == "tcp_abs":
        return tcp_features(ep["tcp"], yaw_ref)
    if kind == "cmd_vel":
        return ep["cmd_vel"]
    raise ValueError(f"unknown action '{kind}', choose from {ACTIONS}")


def state_array(inputs: Sequence[str], ep: Dict[str, np.ndarray], yaw_ref: float) -> np.ndarray:
    parts = []
    if "joints" in inputs:
        parts.append(ep["qpos"])
    if "tcp" in inputs:
        parts.append(tcp_features(ep["tcp"], yaw_ref))
    return np.concatenate(parts, axis=1) if parts else np.zeros((ep["n"], 0), np.float32)


def split_episodes(files: List[Path], val_fraction: float, seed: int) -> Tuple[List[Path], List[Path]]:
    n = len(files)
    n_val = 0 if n < 3 else max(1, int(round(n * val_fraction)))
    order = np.random.default_rng(seed).permutation(n)
    val = sorted(files[i] for i in order[:n_val])
    train = sorted(files[i] for i in order[n_val:])
    return train, val


def compute_norm_stats(train_files: List[Path], inputs: Sequence[str], action: str, yaw_ref: float) -> dict:
    S, A = [], []
    for p in train_files:
        ep = load_episode_arrays(p)
        S.append(state_array(inputs, ep, yaw_ref))
        A.append(action_array(action, ep, yaw_ref))
    S, A = np.concatenate(S), np.concatenate(A)

    def ms(x):
        return {"mean": x.mean(0).tolist() if x.shape[1] else [], "std": np.clip(x.std(0), 1e-2, None).tolist() if x.shape[1] else []}
    return {"state": ms(S), "action": ms(A), "yaw_ref_deg": yaw_ref}


def dataset_yaw_ref(files: List[Path]) -> float:
    """Circular mean of the first-frame yaw over all episodes: the reference around which yaw is made continuous."""
    ys = []
    for p in files:
        with h5py.File(p) as f:
            ys.append(np.deg2rad(f["aligned/tcp"][0, 5]))
    ys = np.array(ys)
    return float(np.rad2deg(np.arctan2(np.sin(ys).mean(), np.cos(ys).mean())))


class ACTDataset(Dataset):
    def __init__(self, files: List[Path], inputs: Sequence[str], action: str, chunk_size: int, image_wh: Tuple[int, int],
                 depth_max_m: float, stats: dict):
        for i in inputs:
            if i not in ALL_INPUTS:
                raise ValueError(f"unknown input '{i}', choose from {ALL_INPUTS}")
        self.files, self.inputs, self.chunk = list(files), list(inputs), chunk_size
        self.image_wh, self.depth_max = tuple(image_wh), float(depth_max_m)
        self.yaw_ref = stats["yaw_ref_deg"]
        self.s_mean = np.array(stats["state"]["mean"], np.float32)
        self.s_std = np.array(stats["state"]["std"], np.float32)
        self.a_mean = np.array(stats["action"]["mean"], np.float32)
        self.a_std = np.array(stats["action"]["std"], np.float32)
        self.state, self.action, self.index = [], [], []
        for ei, p in enumerate(self.files):
            ep = load_episode_arrays(p)
            self.state.append((state_array(inputs, ep, self.yaw_ref) - self.s_mean) / self.s_std if len(self.s_mean) else state_array(inputs, ep, self.yaw_ref))
            self.action.append((action_array(action, ep, self.yaw_ref) - self.a_mean) / self.a_std)
            self.index += [(ei, t) for t in range(ep["n"])]
        self._handles: Dict[int, h5py.File] = {}

    def __len__(self):
        return len(self.index)

    def _h5(self, ei: int) -> h5py.File:  # one lazily-opened handle per episode per worker process
        if ei not in self._handles:
            self._handles[ei] = h5py.File(self.files[ei], "r")
        return self._handles[ei]

    def _image(self, kind: str, f: h5py.File, t: int) -> np.ndarray:
        w, h = self.image_wh
        if kind == "depth":
            d = decode_depth(f["frames/depth"][t])
            d = cv2.resize(d, (w, h), interpolation=cv2.INTER_NEAREST).astype(np.float32) / 1000.0
            valid = (d > 0).astype(np.float32)
            dn = np.clip(d / self.depth_max, 0.0, 1.0) * valid
            return np.stack([dn, valid]).astype(np.float32)          # (2, H, W)
        img = decode_image(f["frames/cam_left" if kind == "cam_left" else "frames/cam_right"][t])
        img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)[:, :, ::-1].astype(np.float32) / 255.0
        return ((img - IMAGENET_MEAN) / IMAGENET_STD).transpose(2, 0, 1).astype(np.float32)   # (3, H, W)

    def __getitem__(self, i: int) -> dict:
        ei, t = self.index[i]
        f = self._h5(ei)
        act = self.action[ei]
        chunk = act[t:t + self.chunk]
        n_real = len(chunk)
        if n_real < self.chunk:  # pad with the last action, as ACT does, and mask it out of the loss
            chunk = np.concatenate([chunk, np.repeat(act[-1:], self.chunk - n_real, axis=0)])
        is_pad = np.zeros(self.chunk, dtype=bool)
        is_pad[n_real:] = True
        out = {"state": torch.from_numpy(self.state[ei][t].astype(np.float32)),
               "actions": torch.from_numpy(chunk.astype(np.float32)), "is_pad": torch.from_numpy(is_pad)}
        for k in self.inputs:
            if k in IMAGE_INPUTS:
                out[k] = torch.from_numpy(self._image(k, f, t))
        return out
