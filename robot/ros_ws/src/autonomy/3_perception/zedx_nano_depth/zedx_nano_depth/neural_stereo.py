"""Neural stereo disparity backends for the Nano depth pipeline (GPU, PyTorch).

Models (the ``model`` option):

* ``raft-realtime`` (default): RAFT-Stereo real-time variant (MIT, vendored under
  ``raft_stereo``). About 45 ms per 640x400 pair on the lab's Jetson AGX Thor
  in eager mode, less when captured as a CUDA graph; dense output. Weights:
  ``<models_dir>/raftstereo-realtime.pth`` (downloaded on first use).
* ``raft-middlebury``: RAFT-Stereo full model, more accurate but ~0.5 s per pair here.
* ``fast-foundation``: NVIDIA Fast-FoundationStereo (CVPR 2026), best zero-shot accuracy,
  ~150 ms per 640x400 pair here in eager PyTorch. Needs a checkout of
  https://github.com/NVlabs/Fast-FoundationStereo (``repo`` option, else
  ``$FAST_FOUNDATION_STEREO_DIR``, else ``<models_dir>/Fast-FoundationStereo``; code is NVIDIA
  Source Code License-NC) and the commercially licensed weights from Hugging Face
  (``nvidia/c-fast-foundationstereo``, downloaded on first use into the checkout's
  ``pretrained_models/``).
* a path to a ``.pth`` file: loaded as a RAFT-Stereo checkpoint (full model unless the file
  name contains ``realtime``).

``models_dir`` defaults to ``$ZEDX_NANO_DEPTH_MODELS``, else ``~/.cache/zedx_nano_depth``.

The network runs in a child process: an iterative stereo network is hundreds of small
kernel launches, each needing the interpreter lock, and sharing that lock with the 50 Hz
control loop made inference 2-10x slower in-process. Frames cross to the worker through
shared memory. `NeuralStereoBackend` is the in-process proxy with the backend interface
(``disparity(left_bgr, right_bgr) -> float32 disparity``, ``name``, ``warmup``, ``close``).
"""
from __future__ import annotations

import io
import multiprocessing
import os
import sys
import threading
import time
import types
import urllib.request
import zipfile
from multiprocessing import shared_memory
from pathlib import Path

import cv2
import numpy as np

MODELS_DIR = Path(os.environ.get('ZEDX_NANO_DEPTH_MODELS') or Path.home() / '.cache' / 'zedx_nano_depth').expanduser()
RAFT_ZIP_URL = ('https://www.dropbox.com/scl/fi/5khx1bhz84dapi8vtwapg/models.zip'
                '?rlkey=ggddrn1du1iiq6mgc2dsdpmwi&dl=1')
FAST_FOUNDATION_WEIGHTS_URL = 'https://huggingface.co/nvidia/c-fast-foundationstereo/resolve/main/model_best_bp2_serialize.pth'
FAST_FOUNDATION_CFG_URL = 'https://huggingface.co/nvidia/c-fast-foundationstereo/resolve/main/cfg.yaml'

MODELS = {
    'raft-realtime': {'kind': 'raft', 'file': 'raftstereo-realtime.pth', 'realtime': True, 'iters': 7},
    'raft-middlebury': {'kind': 'raft', 'file': 'raftstereo-middlebury.pth', 'realtime': False, 'iters': 12},
    'fast-foundation': {'kind': 'fast-foundation', 'iters': 4, 'max_disp': 192},
}
DEFAULT_MODEL = 'raft-realtime'
STARTUP_TIMEOUT_S = 900.0   # covers a first-time weight download
INFER_TIMEOUT_S = 30.0


def resolve_model(model, repo=None, models_dir=None):
    """Registry entry for a model name or .pth path (ValueError when unknown); cheap, no torch."""
    models_dir = Path(models_dir).expanduser() if models_dir else MODELS_DIR
    name = model or DEFAULT_MODEL
    spec = MODELS.get(name)
    if spec is None:
        path = Path(name)
        if path.suffix != '.pth' or not path.exists():
            raise ValueError(f'Unknown depth model {name!r}; expected one of {sorted(MODELS)} or a .pth path')
        realtime = 'realtime' in path.name
        spec = {'kind': 'raft', 'path': str(path), 'realtime': realtime, 'iters': 7 if realtime else 12}
    spec = dict(spec, name=name, models_dir=str(models_dir))
    if spec['kind'] == 'fast-foundation':
        spec['repo'] = str(repo or os.environ.get('FAST_FOUNDATION_STEREO_DIR') or models_dir / 'Fast-FoundationStereo')
    return spec


def _download(url, destination, timeout=600):
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    tmp = destination.with_suffix(destination.suffix + '.part')
    with urllib.request.urlopen(url, timeout=timeout) as response, open(tmp, 'wb') as out:
        while True:
            chunk = response.read(1 << 20)
            if not chunk:
                break
            out.write(chunk)
    os.replace(tmp, destination)
    return destination


def ensure_raft_weights(file_name, models_dir=MODELS_DIR):
    """Path to a RAFT-Stereo checkpoint, downloading the authors' models.zip once if needed."""
    path = Path(models_dir) / file_name
    if path.exists():
        return path
    print(f'Downloading RAFT-Stereo weights ({file_name}) to {models_dir} ...', flush=True)
    with urllib.request.urlopen(RAFT_ZIP_URL, timeout=900) as response:
        archive = zipfile.ZipFile(io.BytesIO(response.read()))
    Path(models_dir).mkdir(parents=True, exist_ok=True)
    for member in archive.namelist():
        if member.endswith('.pth') and '__MACOSX' not in member:
            (Path(models_dir) / Path(member).name).write_bytes(archive.read(member))
    if not path.exists():
        raise RuntimeError(f'{file_name} was not in the RAFT-Stereo models archive')
    return path


class _OptionalStub(types.ModuleType):
    """Stand-in for optional packages (open3d, scikit-image) that Fast-FoundationStereo imports
    for point-cloud output but never touches during disparity inference."""

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        return _OptionalStub(f'{self.__name__}.{name}')

    def __call__(self, *args, **kwargs):
        return None


# ---------------------------------------------------------------------------------------------
# In-process model (runs inside the worker process)
# ---------------------------------------------------------------------------------------------
class LocalNeuralModel:
    """Loads and runs a stereo network on the CUDA device of the current process."""

    def __init__(self, spec, iters=None, max_disp=None):
        try:
            import torch
        except ImportError as exc:
            raise RuntimeError('The neural depth backend needs PyTorch with CUDA (use the sgbm backend otherwise)') from exc
        if not torch.cuda.is_available():
            raise RuntimeError('The neural depth backend needs a CUDA GPU (use the sgbm backend otherwise)')
        self.torch = torch
        self.spec = spec
        self.kind = spec['kind']
        self.iters = iters or spec['iters']
        self.max_disp = max_disp or spec.get('max_disp', 192)
        self.graph = None
        if self.kind == 'raft':
            self._load_raft(spec)
            self.name = 'raft_stereo_realtime' if spec['realtime'] else 'raft_stereo'
        else:
            self._load_fast_foundation(spec)
            self.name = 'fast_foundation_stereo'

    def _load_raft(self, spec):
        import argparse
        torch = self.torch
        from .raft_stereo import RAFTStereo, InputPadder
        realtime = spec['realtime']
        args = argparse.Namespace(mixed_precision=True, hidden_dims=[128] * 3,
                                  corr_implementation='reg' if realtime else 'alt', shared_backbone=realtime,
                                  corr_levels=4, corr_radius=4, n_downsample=3 if realtime else 2,
                                  context_norm='batch', slow_fast_gru=realtime, n_gru_layers=2 if realtime else 3)
        path = Path(spec['path']) if spec.get('path') else ensure_raft_weights(spec['file'], spec['models_dir'])
        model = torch.nn.DataParallel(RAFTStereo(args), device_ids=[0])
        model.load_state_dict(torch.load(path, map_location='cuda'))
        self.model = model.module.cuda().eval()
        self.weights_path = str(path)
        self._padder_cls = InputPadder

    def _load_fast_foundation(self, spec):
        torch = self.torch
        repo = Path(spec['repo'])
        if not (repo / 'core' / 'foundation_stereo.py').exists():
            raise RuntimeError(f'Fast-FoundationStereo checkout not found at {repo}; '
                               'git clone https://github.com/NVlabs/Fast-FoundationStereo there or set the repo option')
        weights = repo / 'pretrained_models' / 'model_best_bp2_serialize.pth'
        if not weights.exists():
            print(f'Downloading Fast-FoundationStereo weights to {weights} ...', flush=True)
            _download(FAST_FOUNDATION_WEIGHTS_URL, weights)
            _download(FAST_FOUNDATION_CFG_URL, weights.with_name('cfg.yaml'))
        for module in ('open3d', 'skimage', 'skimage.measure', 'trimesh'):
            if module not in sys.modules:
                try:
                    __import__(module)
                except Exception:
                    sys.modules[module] = _OptionalStub(module)
        for entry in (str(repo), str(repo / 'core')):
            if entry not in sys.path:
                sys.path.insert(0, entry)
        from core.utils.utils import InputPadder  # noqa: E402  (the checkout's module)
        import Utils  # noqa: E402  (the checkout's helpers: AMP_DTYPE)
        model = torch.load(weights, map_location='cpu', weights_only=False)  # pickled nn.Module
        model.args.normalize = model.args.get('normalize', True)
        model.args.valid_iters = self.iters
        model.args.max_disp = self.max_disp
        self.model = model.cuda().eval()
        self.weights_path = str(weights)
        self._padder_cls = InputPadder
        self._amp_dtype = Utils.AMP_DTYPE

    def _to_tensor(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return self.torch.from_numpy(np.ascontiguousarray(rgb)).permute(2, 0, 1).float()[None].cuda()

    def _pad(self, left, right):
        if self.kind == 'raft':
            padder = self._padder_cls(left.shape, divis_by=32)
        else:
            padder = self._padder_cls(left.shape, divis_by=32, force_square=False)
        return padder, padder.pad(left, right)

    def _forward(self, left, right):
        torch = self.torch
        if self.kind == 'raft':
            with torch.autocast('cuda', enabled=True):
                _, flow = self.model(left, right, iters=self.iters, test_mode=True)
            return -flow
        with torch.amp.autocast('cuda', enabled=True, dtype=self._amp_dtype):
            return self.model.forward(left, right, iters=self.iters, test_mode=True,
                                      optimize_build_volume='pytorch1').float()

    def try_capture_graph(self, size):
        """Capture the forward pass as a CUDA graph for this (width, height); False if not possible."""
        if self.kind != 'raft':
            return False
        torch = self.torch
        try:
            width, height = size
            blank = np.zeros((height, width, 3), dtype=np.uint8)
            left, right = self._to_tensor(blank), self._to_tensor(blank)
            padder, (pl, pr) = self._pad(left, right)
            static_left, static_right = pl.clone(), pr.clone()
            stream = torch.cuda.Stream()
            stream.wait_stream(torch.cuda.current_stream())
            with torch.cuda.stream(stream), torch.no_grad():
                for _ in range(3):
                    self._forward(static_left, static_right)
            torch.cuda.current_stream().wait_stream(stream)
            graph = torch.cuda.CUDAGraph()
            with torch.no_grad(), torch.cuda.graph(graph):
                static_out = self._forward(static_left, static_right)
            self.graph = (size, graph, static_left, static_right, static_out, padder)
            return True
        except Exception as exc:  # any capture problem: keep eager execution
            print(f'CUDA graph capture unavailable ({exc.__class__.__name__}: {exc}); using eager mode', flush=True)
            self.graph = None
            return False

    def disparity(self, left_bgr, right_bgr):
        torch = self.torch
        with torch.no_grad():
            left, right = self._to_tensor(left_bgr), self._to_tensor(right_bgr)
            size = (left_bgr.shape[1], left_bgr.shape[0])
            if self.graph is not None and self.graph[0] == size:
                _, graph, static_left, static_right, static_out, padder = self.graph
                pl, pr = padder.pad(left, right)
                static_left.copy_(pl)
                static_right.copy_(pr)
                graph.replay()
                out = padder.unpad(static_out)
            else:
                padder, (pl, pr) = self._pad(left, right)
                out = padder.unpad(self._forward(pl, pr))
            return out.squeeze().float().cpu().numpy()

    def describe(self):
        return {'backend': 'neural', 'model': self.spec['name'], 'weights': self.weights_path, 'iters': self.iters,
                'cuda_graph': self.graph is not None}


# ---------------------------------------------------------------------------------------------
# Worker process
# ---------------------------------------------------------------------------------------------
def _worker_main(conn, spec, iters, max_disp):
    """Child process: loads the model, then serves 'setup'/'infer'/'quit' requests over `conn`."""
    try:
        model = LocalNeuralModel(spec, iters=iters, max_disp=max_disp)
        conn.send(('ready', model.name, model.describe()))
    except Exception as exc:
        conn.send(('error', f'{exc.__class__.__name__}: {exc}'))
        return
    buffers = None
    while True:
        try:
            request = conn.recv()
        except (EOFError, OSError):
            break
        kind = request[0]
        try:
            if kind == 'quit':
                break
            if kind == 'setup':
                _, names, height, width = request
                if buffers:
                    for shm in buffers[:3]:
                        shm.close()
                left_shm, right_shm, out_shm = (shared_memory.SharedMemory(name=n) for n in names)
                left = np.ndarray((height, width, 3), np.uint8, buffer=left_shm.buf)
                right = np.ndarray((height, width, 3), np.uint8, buffer=right_shm.buf)
                out = np.ndarray((height, width), np.float32, buffer=out_shm.buf)
                buffers = (left_shm, right_shm, out_shm, left, right, out)
                model.try_capture_graph((width, height))
                for _ in range(2):  # warm-up so live frames never pay the first-call cost
                    model.disparity(left, right)
                model.torch.cuda.synchronize()
                conn.send(('ok', model.describe()))
            elif kind == 'infer':
                if buffers is None:
                    raise RuntimeError('infer before setup')
                _, _, _, left, right, out = buffers
                t0 = time.monotonic()
                out[...] = model.disparity(left, right)
                conn.send(('ok', round(1000 * (time.monotonic() - t0), 1)))
            else:
                raise ValueError(f'unknown request {kind!r}')
        except Exception as exc:
            conn.send(('error', f'{exc.__class__.__name__}: {exc}'))
    if buffers:
        for shm in buffers[:3]:
            shm.close()


# ---------------------------------------------------------------------------------------------
# In-process proxy (the backend object used by StereoMatcher)
# ---------------------------------------------------------------------------------------------
class NeuralStereoBackend:
    """GPU stereo network in a worker process; ``disparity()`` returns a float32 disparity map."""

    def __init__(self, model=DEFAULT_MODEL, repo=None, iters=None, max_disp=None, models_dir=None, **_ignored):
        self.spec = resolve_model(model, repo, models_dir)
        self.model_name = self.spec['name']
        self.iters, self.max_disp = iters, max_disp
        self.num_disparities = max_disp or self.spec.get('max_disp', 192)
        self._lock = threading.Lock()
        self._shape = None
        self._shm = []
        self._arrays = None
        self.last_infer_ms = None
        ctx = multiprocessing.get_context('spawn')
        self._conn, child_conn = ctx.Pipe()
        self._proc = ctx.Process(target=_worker_main, args=(child_conn, self.spec, iters, max_disp),
                                 name='neural-stereo', daemon=True)
        self._proc.start()
        child_conn.close()
        reply = self._recv(STARTUP_TIMEOUT_S, 'model load')
        self.name, self.info = reply[1], reply[2]

    def _recv(self, timeout, what):
        if not self._conn.poll(timeout):
            self.close()
            raise RuntimeError(f'Neural depth worker did not answer within {timeout:.0f} s ({what})')
        try:
            reply = self._conn.recv()
        except EOFError:
            self.close()
            raise RuntimeError(f'Neural depth worker exited during {what}; the "spawn" start method re-imports '
                               'the main script, so run from a script file (not stdin/interactive)') from None
        if reply[0] == 'error':
            raise RuntimeError(f'Neural depth worker failed ({what}): {reply[1]}')
        return reply

    def _setup(self, shape):
        height, width = shape
        self._release_shm()
        sizes = (height * width * 3, height * width * 3, height * width * 4)
        self._shm = [shared_memory.SharedMemory(create=True, size=size) for size in sizes]
        self._arrays = (np.ndarray((height, width, 3), np.uint8, buffer=self._shm[0].buf),
                        np.ndarray((height, width, 3), np.uint8, buffer=self._shm[1].buf),
                        np.ndarray((height, width), np.float32, buffer=self._shm[2].buf))
        self._conn.send(('setup', [shm.name for shm in self._shm], height, width))
        self.info = self._recv(STARTUP_TIMEOUT_S, 'warm-up')[1]
        self._shape = shape

    def warmup(self, size):
        width, height = size
        with self._lock:
            if self._shape != (height, width):
                self._setup((height, width))

    def disparity(self, left_bgr, right_bgr):
        if left_bgr.shape != right_bgr.shape or left_bgr.ndim != 3:
            raise ValueError('Stereo pair must be two BGR images of the same size')
        with self._lock:
            if self._proc is None or not self._proc.is_alive():
                raise RuntimeError('Neural depth worker process has exited')
            shape = left_bgr.shape[:2]
            if self._shape != shape:
                self._setup(shape)
            left, right, out = self._arrays
            np.copyto(left, left_bgr)
            np.copyto(right, right_bgr)
            self._conn.send(('infer',))
            self.last_infer_ms = self._recv(INFER_TIMEOUT_S, 'inference')[1]
            return out.copy()

    def describe(self):
        return dict(self.info, worker_pid=self._proc.pid if self._proc else None)

    def _release_shm(self):
        for shm in self._shm:
            try:
                shm.close()
                shm.unlink()
            except Exception:
                pass
        self._shm, self._arrays = [], None

    def close(self):
        proc, self._proc = self._proc, None
        if proc is not None:
            try:
                self._conn.send(('quit',))
            except Exception:
                pass
            proc.join(timeout=3)
            if proc.is_alive():
                proc.kill()
            try:
                self._conn.close()
            except Exception:
                pass
        self._release_shm()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
