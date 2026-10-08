#!/usr/bin/env bash
# Build navigation_bringup + swerve_navigation.  Works in the robot container and on the host.
#
#   autonomy/1_navigation/build.sh [extra colcon arguments]
#
# In the robot container (ROS 2 Humble) the packages go into robot/ros_ws/install like every other
# Autolab package (`bws` builds them too).  On the host (ROS 2 Jazzy) they go into
# robot/ros_ws_nav/, because robot/ros_ws/{build,install} belong to the container.
#
# Needs no sudo and touches no hardware.  --symlink-install: later edits to the launch file, the
# YAML configs and the Python nodes are live without rebuilding.
set -euo pipefail

# shellcheck source=scripts/ros_env.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/ros_env.sh"

if [[ "$(command -v python3)" != /usr/bin/python3 ]]; then
  echo "ERROR: python3 resolves to $(command -v python3), expected /usr/bin/python3." >&2
  exit 1
fi

echo "Building $NAV_SRC_DIR -> $NAV_WS/install   [$NAV_WHERE]"
mkdir -p "$NAV_WS"
colcon --log-base "$NAV_WS/log" build --symlink-install \
  --base-paths "$NAV_SRC_DIR" \
  --build-base "$NAV_WS/build" --install-base "$NAV_WS/install" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  "$@"

cat << MSG

Build finished.  Source it in any terminal of this $NAV_WHERE:
  source /opt/ros/$NAV_DISTRO/setup.bash && source $NAV_WS/install/setup.bash

Then:
  $NAV_SRC_DIR/run.sh                 dry run: everything starts, the robot cannot move
  $NAV_SRC_DIR/run.sh --hardware      REAL ROBOT: it drives to the goals it is given
MSG
