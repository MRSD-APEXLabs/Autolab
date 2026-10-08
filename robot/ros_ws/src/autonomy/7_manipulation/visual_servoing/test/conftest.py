import sys
from pathlib import Path

# Run from the source tree without building: this package and its siblings camera_perception (YOLO, geometry)
# and zedx_nano_depth (pairing). The siblings sit next to this package (apple_server_manip/2_manipulation) or
# in another group folder of the same workspace (Autolab autonomy/3_perception, next to autonomy/7_manipulation).
_ROOT = Path(__file__).resolve().parents[2]


def _source_of(package):
    candidates = [_ROOT / package] + sorted(_ROOT.parent.glob(f'*/{package}'))
    return next((path for path in candidates if (path / package / '__init__.py').is_file()), None)


for _source in [Path(__file__).resolve().parent, _ROOT / 'visual_servoing',
                _source_of('camera_perception'), _source_of('zedx_nano_depth')]:
    if _source is not None and str(_source) not in sys.path:
        sys.path.insert(0, str(_source))
