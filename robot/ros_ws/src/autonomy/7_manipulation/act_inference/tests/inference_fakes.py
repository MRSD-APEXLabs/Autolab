"""Fakes shared by the act_inference tests: an in-memory Xavier stereo server and a tiny ACT checkpoint."""
import io
import json
import pickle
import threading
import time

import cv2
import numpy as np

from zedx_nano_camera import stream

W, H, FX, BASELINE_MM, SHIFT = 960, 600, 480.0, 18.0, 24
EXPECTED_MM = FX * BASELINE_MM / SHIFT


def synthetic_pair(shift=SHIFT, seed=0):
    """A textured fronto-parallel plane: the right view is the left shifted `shift` px to the left."""
    rng = np.random.RandomState(seed)
    texture = cv2.GaussianBlur((rng.rand(H, W + shift, 3) * 255).astype(np.uint8), (0, 0), 1.2)
    return texture[:, :W].copy(), texture[:, shift:shift + W].copy()


def conf_text():
    """Ideal rig in the FHD1200 capture section (2x the streamed size), no distortion or rotation."""
    eye = 'fx=%f\nfy=%f\ncx=%f\ncy=%f\nk1=0\nk2=0\np1=0\np2=0\nk3=0\n' % (FX * 2, FX * 2, W, H)
    return ('[LEFT_CAM_FHD1200]\n' + eye + '[RIGHT_CAM_FHD1200]\n' + eye +
            '[STEREO]\nBaseline=%f\nTY=0\nTZ=0\nCV_FHD1200=0\nRX_FHD1200=0\nRZ_FHD1200=0\n' % BASELINE_MM)


def jpeg(bgr):
    return cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes()


def part(frame_id, left_jpeg, right_jpeg, capture_ns=None, capture_mono_ns=None):
    capture_ns = time.time_ns() if capture_ns is None else capture_ns
    capture_mono_ns = time.monotonic_ns() if capture_mono_ns is None else capture_mono_ns
    body = left_jpeg + right_jpeg
    head = (b'--frame\r\nContent-Type: application/octet-stream\r\nContent-Length: %d\r\nX-Frame-Id: %d\r\n'
            b'X-Capture-Ns: %d\r\nX-Ready-Ns: %d\r\nX-Capture-Mono-Ns: %d\r\nX-Sensor: stereo\r\n'
            b'X-Left-Length: %d\r\nX-Right-Frame-Id: %d\r\nX-Sync-Us: 120\r\n\r\n'
            % (len(body), frame_id, capture_ns, capture_ns + 1000, capture_mono_ns, len(left_jpeg), frame_id + 2))
    return head + body + b'\r\n'


class BlockingStream:
    """Serves bytes, then blocks like a live socket until close() instead of reporting EOF."""

    def __init__(self, data):
        self.buffer = io.BytesIO(data)
        self.closed = threading.Event()
        self.headers = {'Content-Type': 'multipart/x-mixed-replace; boundary=frame'}

    def _wait_eof(self):
        self.closed.wait(5)
        return b''

    def readline(self):
        line = self.buffer.readline()
        return line if line else self._wait_eof()

    def read(self, n):
        data = self.buffer.read(n)
        return data if data else self._wait_eof()

    def close(self):
        self.closed.set()


def fake_stereo_server(monkeypatch, pairs=2):
    info = {'server_version': 4, 'model': 'ZED X Nano', 'serial': 99292912,
            'capture': {'width': 2 * W, 'height': 2 * H, 'fps': 30}, 'output': {'width': W, 'height': H},
            'sensors': {'left': '/dev/video3', 'right': '/dev/video2'}, 'stereo': {'available': True}}
    left, right = synthetic_pair()
    lj, rj = jpeg(left), jpeg(right)

    def urlopen(url, timeout=None):
        path = url.split('//', 1)[1].split('/', 1)[1]
        if path == 'info':
            return io.BytesIO(json.dumps(info).encode())
        if path == 'time':
            return io.BytesIO(json.dumps({'t_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns()}).encode())
        if path == 'calibration.conf':
            return io.BytesIO(conf_text().encode())
        if path == 'video_feed/stereo':
            return BlockingStream(b''.join(part(i + 1, lj, rj) for i in range(pairs)))
        raise AssertionError(url)
    monkeypatch.setattr(stream, '_open', urlopen)


def write_tiny_checkpoint(directory, use_depth=False, chunk=10):
    """A small, randomly initialised xArm ACT checkpoint in the layout training writes."""
    import torch
    from act_inference.act.image_inputs import DEPTH_ENCODING
    from act_inference.act.policy import ACTPolicy
    from act_inference.config import STATE_SPACE
    torch.manual_seed(0)
    policy_config = {'lr': 1e-5, 'num_queries': chunk, 'kl_weight': 10, 'hidden_dim': 32, 'dim_feedforward': 64,
                     'lr_backbone': 1e-5, 'backbone': 'resnet18', 'enc_layers': 1, 'dec_layers': 1, 'nheads': 2,
                     'camera_names': ['wrist_rgb'], 'state_dim': 6, 'use_depth': use_depth}
    if use_depth:
        policy_config['depth_encoding'] = DEPTH_ENCODING
    policy = ACTPolicy(policy_config)
    directory.mkdir(parents=True, exist_ok=True)
    torch.save(policy.state_dict(), directory / 'policy_best.ckpt')
    config = {'task_name': 'xarm6_demo', 'robot': 'xarm6', 'state_space': STATE_SPACE, 'policy_class': 'ACT',
              'state_dim': 6, 'camera_names': ['wrist_rgb'], 'policy_config': policy_config, 'use_depth': use_depth}
    with open(directory / 'config.pkl', 'wb') as f:
        pickle.dump(config, f)
    stats = {'qpos_mean': np.array([.3, 0, .2, 3.1, 0, 0]), 'qpos_std': np.full(6, .05),
             'action_mean': np.array([.3, 0, .2, 3.1, 0, 0]), 'action_std': np.full(6, .01)}
    with open(directory / 'dataset_stats.pkl', 'wb') as f:
        pickle.dump(stats, f)
    return directory
