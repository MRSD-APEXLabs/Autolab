"""ACT (Action Chunking with Transformers) policy, inference subset of the repository's act/ folder.

Copied from act/ (policy.py, image_inputs.py, detr/) with package-relative imports, without the
IPython debugging hooks and without downloading ImageNet backbone weights, which a trained
checkpoint overwrites anyway. Training code stays in the original act/ folder. Licenses: LICENSE
(ACT, MIT) and detr/LICENSE (DETR, Apache 2.0).
"""
