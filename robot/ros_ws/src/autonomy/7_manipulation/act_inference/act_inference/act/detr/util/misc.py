# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
"""
The one DETR utility the ACT model uses (NestedTensor, only in type annotations); the rest of
DETR's util/misc.py (distributed training and logging helpers) is not needed for inference.
"""
from typing import Optional

from torch import Tensor


class NestedTensor(object):
    def __init__(self, tensors, mask: Optional[Tensor]):
        self.tensors = tensors
        self.mask = mask

    def to(self, device):
        # type: (Device) -> NestedTensor # noqa
        cast_tensor = self.tensors.to(device)
        mask = self.mask
        if mask is not None:
            assert mask is not None
            cast_mask = mask.to(device)
        else:
            cast_mask = None
        return NestedTensor(cast_tensor, cast_mask)

    def decompose(self):
        return self.tensors, self.mask

    def __repr__(self):
        return str(self.tensors)
