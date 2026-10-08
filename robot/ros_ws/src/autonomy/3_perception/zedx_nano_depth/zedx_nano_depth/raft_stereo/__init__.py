"""RAFT-Stereo (Lipson, Teed, Deng; 3DV 2021), vendored from https://github.com/princeton-vl/RAFT-Stereo
(MIT License, see LICENSE). Imports were made relative, opt_einsum.contract replaced by torch.einsum, and the few tensors the
forward pass built on the CPU are now created on the input's device so the forward can be captured as
a CUDA graph; the model code is otherwise unchanged. Used by zedx_nano_depth/neural_stereo.py.
"""
from .raft_stereo import RAFTStereo
from .utils import InputPadder
