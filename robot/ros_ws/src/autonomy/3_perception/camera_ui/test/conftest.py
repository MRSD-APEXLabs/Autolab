import sys
from pathlib import Path

# Run from the source tree without building: this package plus the sibling zedx_nano_depth (colormap) it imports.
_ROOT = Path(__file__).resolve().parents[2]
for _source in (_ROOT / 'camera_ui', _ROOT / 'zedx_nano_depth', _ROOT / 'zedx_nano_camera'):
    if str(_source) not in sys.path:
        sys.path.insert(0, str(_source))
