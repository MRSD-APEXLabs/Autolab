#!/usr/bin/env bash
# Open the recorded episodes in a web page (see actlib/viewer.py). Default: data/wellplate_place on http://localhost:8090
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
exec env -u PYTHONPATH "$HERE/.venv/bin/python" -m actlib.viewer "$@"
