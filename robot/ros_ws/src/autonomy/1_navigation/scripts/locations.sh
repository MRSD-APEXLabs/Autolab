#!/usr/bin/env bash
# Saved stations for the active navigation map (default best); no motor setup.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ros_env.sh"
set +u
if [[ -f "$NAV_WS/install/setup.bash" ]]; then source "$NAV_WS/install/setup.bash"; fi
set -u
exec /usr/bin/python3 "$SCRIPT_DIR/locations.py" "$@"
