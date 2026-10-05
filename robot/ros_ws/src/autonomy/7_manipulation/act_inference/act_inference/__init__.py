"""Live ACT inference for the xArm6 with the ZED X Nano wrist camera.

The Nano stream client and the stereo depth code are the plain-Python modules of the sibling
ROS 2 packages (zedx_nano_camera, zedx_nano_depth). Their source folders are put on sys.path
here, so this folder runs without building a ROS workspace and both paths share one
implementation. The siblings sit next to this folder (apple_server_manip/2_manipulation) or in
another group folder of the same workspace (Autolab autonomy/3_perception, next to
autonomy/7_manipulation). If they are missing, installed zedx_nano_* packages are used instead.
"""
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]


def _source_of(package):
    """The package's source folder, or None: next to this one, or in a sibling group folder."""
    candidates = [_ROOT / package] + sorted(_ROOT.parent.glob(f'*/{package}'))
    return next((path for path in candidates if (path / package / '__init__.py').is_file()), None)


for _package in ('zedx_nano_camera', 'zedx_nano_depth'):
    _source = _source_of(_package)
    if _source is not None and str(_source) not in sys.path:
        sys.path.insert(0, str(_source))
