#!/usr/bin/env bash
# Inspect a recorded episode / dataset (see actlib/inspect_episode.py).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
exec env -u PYTHONPATH "$HERE/.venv/bin/python" -m actlib.inspect_episode "$@"
