#!/usr/bin/env bash
# Start RViz as a separate ROS 2 Humble process inside the running robot container. The X11 window
# appears on the host, but no host Jazzy ROS participant joins the navigation domain.
set -euo pipefail

CONTAINER="${AUTOLAB_ROBOT_CONTAINER:-autolab-robot-l4t-1}"
DOMAIN="${NAVIGATION_DOMAIN_ID:-42}"
CONFIG=/home/robot/AutoLab/robot/ros_ws/src/autonomy/1_navigation/navigation_bringup/rviz/navigation.rviz
DDS_PROFILE=/home/robot/AutoLab/robot/ros_ws/src/autonomy/1_navigation/scripts/fastdds_no_shm.xml

usage() {
  cat <<'EOF'
Usage: rviz_docker.sh [--domain ID] [--container NAME]

Run this on the host after the container's run.sh has started. The default navigation domain is 42.
EOF
}

while (($#)); do
  case "$1" in
    --domain)
      (($# >= 2)) || { echo "--domain needs a value" >&2; exit 2; }
      DOMAIN="$2"; shift 2 ;;
    --container)
      (($# >= 2)) || { echo "--container needs a value" >&2; exit 2; }
      CONTAINER="$2"; shift 2 ;;
    -h | --help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$DOMAIN" =~ ^[0-9]+$ ]] || { echo "ROS domain must be a non-negative integer: $DOMAIN" >&2; exit 2; }
command -v docker >/dev/null 2>&1 || { echo "docker is not installed on this host" >&2; exit 1; }
if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  echo "container is not running: $CONTAINER" >&2
  exit 1
fi
NAV_PID="$(docker exec "$CONTAINER" pgrep -fo 'ros2 launch navigation_bringup' 2>/dev/null || true)"
if [[ -z "$NAV_PID" ]]; then
  echo "navigation is not running in $CONTAINER; start run.sh first" >&2
  exit 1
fi

# A separate RViz process can discover Nav2 only when both use the same domain. Read the launch
# process's actual environment instead of trusting that the caller remembered its override.
NAV_DOMAIN="$(docker exec "$CONTAINER" bash -c \
  "tr '\0' '\n' < /proc/$NAV_PID/environ | sed -n 's/^ROS_DOMAIN_ID=//p'" 2>/dev/null || true)"
NAV_DOMAIN="${NAV_DOMAIN:-0}"
NAV_RMW="$(docker exec "$CONTAINER" bash -c \
  "tr '\0' '\n' < /proc/$NAV_PID/environ | sed -n 's/^RMW_IMPLEMENTATION=//p'" 2>/dev/null || true)"
NAV_RMW="${NAV_RMW:-rmw_fastrtps_cpp}"
if [[ "$DOMAIN" != "$NAV_DOMAIN" ]]; then
  echo "REFUSING: navigation uses ROS domain $NAV_DOMAIN, but RViz was asked to use $DOMAIN." >&2
  echo "Run this script with: --domain $NAV_DOMAIN" >&2
  exit 2
fi
if docker exec "$CONTAINER" pgrep -x rviz2 >/dev/null 2>&1; then
  echo "RViz is already running in $CONTAINER" >&2
  exit 1
fi

# Refuse a known-bad mixed-distro domain. Container processes are visible in the host PID namespace,
# so checking every readable environment catches both ordinary host nodes and container processes.
conflicts=()
for proc in /proc/[0-9]*; do
  [[ -r "$proc/environ" && -r "$proc/cmdline" ]] || continue
  [[ "${proc##*/}" != "$$" ]] || continue
  # /proc can pass the mode-bit check and still deny ptrace-protected environments. Put the
  # redirection inside a subshell so that denial is silent as intended.
  env_text="$( (tr '\0' '\n' < "$proc/environ") 2>/dev/null || true)"
  proc_domain="$(sed -n 's/^ROS_DOMAIN_ID=//p' <<< "$env_text")"
  proc_domain="${proc_domain:-0}"
  if grep -qx 'ROS_DISTRO=jazzy' <<< "$env_text" && [[ "$proc_domain" == "$DOMAIN" ]]; then
    cmd="$( (tr '\0' ' ' < "$proc/cmdline") 2>/dev/null || true)"
    conflicts+=("  pid ${proc##*/}: ${cmd:-unknown command}")
  fi
done
if ((${#conflicts[@]})); then
  echo "REFUSING: host Jazzy process(es) already use ROS domain $DOMAIN:" >&2
  printf '%s\n' "${conflicts[@]}" >&2
  echo "Stop them or choose another NAVIGATION_DOMAIN_ID for both run.sh and this script." >&2
  exit 2
fi

docker_tty=(-i)
[[ -t 0 && -t 1 ]] && docker_tty=(-it)
docker_env=(-e "ROS_DOMAIN_ID=$DOMAIN" -e "RMW_IMPLEMENTATION=$NAV_RMW")
if [[ "$NAV_RMW" == rmw_fastrtps_cpp ]]; then
  docker_env+=(-e "FASTRTPS_DEFAULT_PROFILES_FILE=$DDS_PROFILE")
fi
echo "Starting Docker RViz: container=$CONTAINER ROS=Humble domain=$DOMAIN RMW=$NAV_RMW"
exec docker exec "${docker_tty[@]}" "${docker_env[@]}" \
  -e "DISPLAY=${DISPLAY:-:1}" \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=/.Xauthority \
  "$CONTAINER" bash -lc "
    install -d -m 700 /tmp/runtime-robot
    export XDG_RUNTIME_DIR=/tmp/runtime-robot
    source /opt/ros/humble/setup.bash
    source /home/robot/AutoLab/robot/ros_ws/install/setup.bash
    exec rviz2 -d '$CONFIG' -f map
  "
