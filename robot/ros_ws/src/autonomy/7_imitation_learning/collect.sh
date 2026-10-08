#!/usr/bin/env bash
# Run the ACT data collector inside its own venv.
# PYTHONPATH is cleared because the ROS Jazzy environment leaks into virtualenvs on this machine.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
exec env -u PYTHONPATH "$HERE/.venv/bin/python" -m actlib.collect "$@"
