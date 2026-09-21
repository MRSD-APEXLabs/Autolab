#!/usr/bin/env bash
# Start navigation on the HOST (ROS 2 Jazzy), localised in the saved map, taking goals from
# /nav/goal_pose, /nav/goal_location, RViz, or a saved place clicked in RViz.
#
#   autonomy/1_navigation/run.sh [--hardware] [--yes] [launch arguments ...]
#
#   --hardware     ENABLE THE DRIVETRAIN.  Without it everything runs on the phoenix6 simulator:
#                  every topic behaves normally, the robot cannot move.
#   --yes          skip the typed confirmation (--hardware only)
#   launch args    passed to navigation.launch.py, e.g.
#                    map:=/path/to/other.yaml       another map (default: the one in the package)
#                    x:=1.0 y:=0.5 yaw:=1.57        initial pose in the map
#                    rviz:=false                    headless
#                    api_namespace:=/robot_1        publish the API as /robot_1/nav/...
#                    nav_motion:=omni               allow strafing - read the blind-wedge warning
#
# WITH --hardware THE ROBOT MOVES BY ITSELF.  Keep the Xbox pad in your hand: it is the e-stop.
set -euo pipefail

SRC_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SRC_DIR/../../.." && pwd)_nav"

HARDWARE=0
ASSUME_YES=0
LAUNCH_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --hardware) HARDWARE=1 ;;
    --yes) ASSUME_YES=1 ;;
    -h | --help)
      sed -n '2,18p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    hardware:=*)
      echo "use --hardware instead of '$arg'" >&2
      exit 2
      ;;
    joy:=*)
      # The pad is the override AND the only e-stop.
      echo "'$arg' is not accepted: the robot never drives autonomously without the Xbox pad." >&2
      exit 2
      ;;
    *:=*) LAUNCH_ARGS+=("$arg") ;;
    *)
      echo "unknown argument: $arg (try --help)" >&2
      exit 2
      ;;
  esac
done

if ((HARDWARE)) && [[ "${SWERVE_NAV_FORCE_SIM:-0}" == 1 ]]; then
  echo "REFUSING: SWERVE_NAV_FORCE_SIM=1 is set in this shell (simulation-only environment)." >&2
  exit 2
fi

unset VIRTUAL_ENV PYTHONHOME
PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -vE '(/\.venv/bin|/venv/bin)$' | paste -sd:)"
export PATH
set +u
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "ERROR: $WS/install does not exist - run $SRC_DIR/build.sh first." >&2
  exit 1
fi
source "$WS/install/setup.bash"
set -u

# Two stacks on one ROS domain would both publish /cmd_vel.
running="$(pgrep -af 'ros2 launch navigation_bringup|lib/swerve_navigation/|ros2 launch nav5_bringup|lib/nav5/' || true)"
if [[ -n "$running" ]]; then
  echo "REFUSING: a navigation stack is already running:" >&2
  echo "$running" >&2
  exit 1
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0 (unset)}  RMW=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp (default)}"

if ((HARDWARE)); then
  # Brings the CANivore's SocketCAN network up (needs no sudo, commands no motion).
  if ! "$SRC_DIR/scripts/canivore_up.sh"; then
    echo "Could not bring the CANivore network up: is it plugged in? (check: caniv -l)" >&2
    exit 1
  fi
  if [[ ! -e /dev/input/event0 && ! -e /dev/input/js0 ]]; then
    echo "REFUSING: no joystick found. The Xbox pad is the e-stop: plug it in first." >&2
    exit 1
  fi
  cat << 'EOF2'

================================================================================================
  AUTONOMOUS NAVIGATION                                                     *** REAL ROBOT ***
================================================================================================
  This ENABLES THE SWERVE DRIVETRAIN and THE ROBOT WILL MOVE BY ITSELF towards the goals it is
  given - from RViz, from a saved place, or from /nav/goal_pose and /nav/goal_location published
  by the rest of the system (up to 0.40 m/s).  Keep the Xbox pad IN YOUR HAND:

      B          E-STOP (latched)          Start   release it (LT released, sticks centred)
      hold LT    joystick OVERRIDES navigation     X       cancel the current goal
      LT + RB    turbo                             A       re-seed "field forward"

  THE ROBOT IS BLIND behind it and to its right (~136 deg shadowed by its own structure): it
  drives nose-first, and nobody should walk up to it from the rear/right while it moves.
  Put it where MAPPING STARTED, or pass x:= y:= yaw:=, or use "2D Pose Estimate" in RViz, and
  check that the live scan lies on the map walls BEFORE the first goal.
================================================================================================
EOF2
  if ((ASSUME_YES)); then
    echo "--yes given: skipping the interactive confirmation."
  elif [[ ! -t 0 ]]; then
    echo "stdin is not a terminal and --yes was not given: refusing." >&2
    exit 1
  else
    read -r -p "Type YES (capitals) to ENABLE THE DRIVETRAIN, anything else aborts: " answer
    [[ "$answer" == "YES" ]] || { echo "Aborted: nothing was started."; exit 1; }
  fi
else
  echo
  echo "  DRY RUN: swerve_bridge is on the phoenix6 SIMULATOR - the robot cannot move."
  echo "  Every /nav topic behaves as on the real robot. Add --hardware to let it drive."
fi

HW=false; ((HARDWARE)) && HW=true
echo
echo "Starting: ros2 launch navigation_bringup navigation.launch.py hardware:=$HW ${LAUNCH_ARGS[*]}"
exec ros2 launch navigation_bringup navigation.launch.py "hardware:=$HW" "${LAUNCH_ARGS[@]}"
