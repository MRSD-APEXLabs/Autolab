#!/usr/bin/env bash
# Run the navigation UI on the host (ROS 2 Jazzy), connected to the robot container's ROS domain.
# The robot stack itself keeps running if this process is closed or restarted.
set -euo pipefail

SRC_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
AUTOLAB_ROOT="$(cd -- "$SRC_DIR/../../../../.." && pwd)"
# The host's login shell normally uses domain 1 for the GCS/domain bridge, while the real robot
# and this navigation stack run on domain 0.  Do not inherit ROS_DOMAIN_ID blindly: doing so opens
# a healthy RViz with no navigation publishers and an entirely blank view.  If no domain is given,
# probe the robot domain first and then the caller's ambient domain.
AMBIENT_DOMAIN="${ROS_DOMAIN_ID:-}"
DOMAIN="${NAVIGATION_DOMAIN_ID:-0}"
DOMAIN_EXPLICIT=0
RVIZ_CONFIG="$SRC_DIR/navigation_bringup/rviz/navigation.rviz"
FIXED_FRAME=map

usage() {
  cat <<'EOF'
Usage: rviz.sh [--domain ID] [--config FILE] [--fixed-frame FRAME]

Run this on the host while run.sh is active in the robot container.  With no --domain argument it
looks for a /map publisher on the robot's normal domain (0), then on the shell's ROS_DOMAIN_ID.
NAVIGATION_DOMAIN_ID changes the default without changing the host's GCS ROS domain.
EOF
}

while (($#)); do
  case "$1" in
    --domain)
      (($# >= 2)) || { echo "--domain needs a value" >&2; exit 2; }
      DOMAIN="$2"; DOMAIN_EXPLICIT=1; shift 2 ;;
    --config)
      (($# >= 2)) || { echo "--config needs a value" >&2; exit 2; }
      RVIZ_CONFIG="$2"; shift 2 ;;
    --fixed-frame)
      (($# >= 2)) || { echo "--fixed-frame needs a value" >&2; exit 2; }
      FIXED_FRAME="$2"; shift 2 ;;
    -h | --help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$DOMAIN" =~ ^[0-9]+$ ]] || { echo "ROS domain must be a non-negative integer: $DOMAIN" >&2; exit 2; }
[[ -f "$RVIZ_CONFIG" ]] || { echo "RViz config not found: $RVIZ_CONFIG" >&2; exit 1; }
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "This launcher must run on the host with ROS 2 Jazzy, not in the Humble robot container." >&2
  exit 1
fi

# A Jazzy Fast-DDS participant must not join the live Humble navigation domain. On this machine it
# makes Humble readers emit deserialization errors and can freeze AMCL's map->odom output. This
# check is specific to the standard AutoLab container; failure to inspect Docker is non-fatal so
# the script remains usable for a host-only navigation stack.
if command -v docker >/dev/null 2>&1 &&
   docker inspect -f '{{.State.Running}}' autolab-robot-l4t-1 2>/dev/null | grep -qx true &&
   docker exec autolab-robot-l4t-1 pgrep -f 'ros2 launch navigation_bringup' >/dev/null 2>&1; then
  cat >&2 <<'EOF'
REFUSING: Humble navigation is active in autolab-robot-l4t-1.

Do not connect Jazzy RViz to it: that cross-distribution Fast-DDS mix froze AMCL in the live test.
Stop the container run, then start navigation and RViz together on the host with:

  cd ~/coding/Autolab/robot/ros_ws
  ./src/autonomy/1_navigation/build.sh
  ./src/autonomy/1_navigation/run.sh --hardware
EOF
  exit 2
fi

# Keep host Python environments out of ROS, and use the same UDP-only DDS transport as the robot.
unset VIRTUAL_ENV PYTHONHOME
PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -vE '(/\.venv/bin|/venv/bin)$' | paste -sd:)"
export PATH
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID="$DOMAIN"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
DDS_PROFILE="$AUTOLAB_ROOT/common/ros_packages/fastdds.xml"
if [[ -f "$DDS_PROFILE" && "$RMW_IMPLEMENTATION" == rmw_fastrtps_cpp ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE"
fi

domain_has_map() {
  local candidate="$1" info
  # Bypass the long-lived ros2 daemon: it is tied to the domain/profile with which it was started
  # and can otherwise make a domain probe return stale results.
  info="$(env ROS_DOMAIN_ID="$candidate" ROS2CLI_DISABLE_DAEMON=1 \
    timeout 3 ros2 topic info /map 2>/dev/null || true)"
  grep -Eq '^Publisher count: [1-9][0-9]*$' <<< "$info"
}

if ((DOMAIN_EXPLICIT == 0)); then
  candidates=("$DOMAIN")
  if [[ "$AMBIENT_DOMAIN" =~ ^[0-9]+$ && "$AMBIENT_DOMAIN" != "$DOMAIN" ]]; then
    candidates+=("$AMBIENT_DOMAIN")
  fi
  for candidate in "${candidates[@]}"; do
    if domain_has_map "$candidate"; then
      DOMAIN="$candidate"
      break
    fi
  done
fi

export ROS_DOMAIN_ID="$DOMAIN"
if domain_has_map "$DOMAIN"; then
  echo "Navigation discovered: /map has a publisher on ROS domain $DOMAIN"
else
  echo "WARNING: no /map publisher is visible on ROS domain $DOMAIN." >&2
  echo "         RViz will start, but it will stay blank until run.sh is active on this domain." >&2
  echo "         Use --domain ID if run.sh printed a different ROS_DOMAIN_ID." >&2
fi

echo "Starting host RViz: domain=$ROS_DOMAIN_ID fixed_frame=$FIXED_FRAME config=$RVIZ_CONFIG"
exec rviz2 -d "$RVIZ_CONFIG" -f "$FIXED_FRAME"
