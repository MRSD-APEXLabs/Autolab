import sys
from pathlib import Path

# `import act_inference` from the source tree; it puts the sibling zedx_nano_* packages on sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import act_inference  # noqa: E402,F401
