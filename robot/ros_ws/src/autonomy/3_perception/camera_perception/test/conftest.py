import sys
from pathlib import Path

# Run from the source tree without building: this package, its tests' helpers and the sibling zedx_nano_depth (pairing).
_ROOT = Path(__file__).resolve().parents[2]
for _source in (Path(__file__).resolve().parent, _ROOT / 'camera_perception', _ROOT / 'zedx_nano_depth'):
    if str(_source) not in sys.path:
        sys.path.insert(0, str(_source))
