"""Exact-stamp pairing of two message streams that are published together (stereo eyes, image + depth)."""
from __future__ import annotations

from collections import OrderedDict
import threading


class PairBuffer:
    """Collects items of two streams by stamp and returns each pair once both halves arrived.

    Streams are assumed to arrive in stamp order, so completing a pair discards every older,
    still incomplete stamp; at most `keep` incomplete stamps are held.
    """

    def __init__(self, keep=8):
        self._keep = keep
        self._pending = OrderedDict()
        self._lock = threading.Lock()

    def add(self, index, stamp, item):
        """Store `item` as half `index` (0 or 1) of `stamp`; returns (first, second) when complete, else None."""
        with self._lock:
            slot = self._pending.get(stamp)
            if slot is None:
                slot = self._pending[stamp] = [None, None]
                while len(self._pending) > self._keep:
                    self._pending.popitem(last=False)
            slot[index] = item
            if slot[0] is None or slot[1] is None:
                return None
            while self._pending:
                key, _ = self._pending.popitem(last=False)
                if key == stamp:
                    break
            return slot[0], slot[1]
