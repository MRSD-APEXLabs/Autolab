import sys
from pathlib import Path

# Run from the source tree without building the package.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
