# Sourced by build.sh and run.sh: pick the ROS distro and the workspace the navigation packages
# are built into.  Sets NAV_DISTRO, NAV_WS and NAV_WHERE.
#
#   humble  the Autolab robot container.  Builds into robot/ros_ws itself, exactly where `bws`
#           puts everything else, so `sws` and the autonomy bringup see the packages.
#   jazzy   the host.  Builds into robot/ros_ws_nav: robot/ros_ws/{build,install} is bind-mounted
#           into the Humble container and a Jazzy build there would overwrite the container's.
# shellcheck shell=bash

NAV_SRC_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
NAV_ROS_WS="$(cd -- "$NAV_SRC_DIR/../../.." && pwd)"          # .../robot/ros_ws

if [[ -f /opt/ros/humble/setup.bash ]]; then
  NAV_DISTRO=humble
  NAV_WS="$NAV_ROS_WS"
  NAV_WHERE="robot container (ROS 2 Humble)"
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  NAV_DISTRO=jazzy
  NAV_WS="${NAV_ROS_WS}_nav"
  NAV_WHERE="host (ROS 2 Jazzy)"
else
  echo "ERROR: neither /opt/ros/humble nor /opt/ros/jazzy is installed here." >&2
  exit 1
fi

# ROS 2 nodes must run on /usr/bin/python3: drop any active venv from PATH before sourcing.
unset VIRTUAL_ENV PYTHONHOME
PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -vE '(/\.venv/bin|/venv/bin)$' | paste -sd:)"
export PATH
set +u
# shellcheck disable=SC1090
source "/opt/ros/$NAV_DISTRO/setup.bash"
set -u
