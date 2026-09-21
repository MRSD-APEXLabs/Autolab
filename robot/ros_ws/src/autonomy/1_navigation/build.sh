#!/usr/bin/env bash
# Build navigation_bringup + swerve_navigation ON THE HOST (ROS 2 Jazzy).
#
#   autonomy/1_navigation/build.sh [extra colcon arguments]
#
# WHY A SEPARATE WORKSPACE: robot/ros_ws/{build,install} belong to the Humble robot container
# (it is bind-mounted there and `bws` writes into it).  A Jazzy build into the same directories
# would overwrite the container's install tree.  This script therefore builds the two navigation
# packages into robot/ros_ws_nav/ and leaves everything the container owns untouched.
#
# Needs no sudo and touches no hardware.  --symlink-install: later edits to the launch file, the
# YAML configs and the Python nodes are live without rebuilding.
set -euo pipefail

SRC_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SRC_DIR/../../.." && pwd)_nav"     # .../robot/ros_ws -> .../robot/ros_ws_nav

# ROS 2 nodes must run on /usr/bin/python3: drop any active venv from PATH before sourcing.
unset VIRTUAL_ENV PYTHONHOME
PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -vE '(/\.venv/bin|/venv/bin)$' | paste -sd:)"
export PATH
set +u; source /opt/ros/jazzy/setup.bash; set -u

if [[ "$(command -v python3)" != /usr/bin/python3 ]]; then
  echo "ERROR: python3 resolves to $(command -v python3), expected /usr/bin/python3." >&2
  exit 1
fi

echo "Building $SRC_DIR -> $WS"
mkdir -p "$WS"
colcon build --symlink-install \
  --base-paths "$SRC_DIR" \
  --build-base "$WS/build" --install-base "$WS/install" --log-base "$WS/log" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  "$@"

cat << MSG

Build finished.  Source it in any host terminal:
  source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash

Then:
  $SRC_DIR/run.sh                 dry run: everything starts, the robot cannot move
  $SRC_DIR/run.sh --hardware      REAL ROBOT: it drives to the goals it is given
MSG
