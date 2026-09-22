#!/usr/bin/env bash
# Start navigation, localised in the saved map, taking goals from /nav/goal_pose,
# /nav/goal_location, RViz, or a saved place clicked in RViz. Hardware navigation runs in the
# robot container (ROS 2 Humble); use rviz_docker.sh for a separate Humble RViz process.
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
# Humble defaults to Cyclone DDS here. Fast DDS repeatedly left controller_server's /tf reader
# frozen while every other subscriber continued normally.
#
# WITH --hardware THE ROBOT MOVES BY ITSELF.  Keep the Xbox pad in your hand: it is the e-stop.
set -euo pipefail

SRC_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

HARDWARE=0
ASSUME_YES=0
RVIZ_EXPLICIT=0
LAUNCH_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --hardware) HARDWARE=1 ;;
    --yes) ASSUME_YES=1 ;;
    -h | --help)
      sed -n '2,21p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
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
    rviz:=*)
      RVIZ_EXPLICIT=1
      LAUNCH_ARGS+=("$arg")
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

# shellcheck source=scripts/ros_env.sh
source "$SRC_DIR/scripts/ros_env.sh"

# Domain 0 already contains unrelated host Jazzy processes on this Jetson. Humble/Jazzy Fast-DDS
# discovery is unsupported and was observed to corrupt Humble readers, freezing AMCL's map->odom
# transform. Keep this self-contained Docker stack on an isolated domain; rviz_docker.sh uses the
# same default. Deliberately override with NAVIGATION_DOMAIN_ID when integration requires it.
if [[ "$NAV_DISTRO" == humble ]]; then
  export RMW_IMPLEMENTATION="${NAVIGATION_RMW:-rmw_cyclonedds_cpp}"
  export ROS_DOMAIN_ID="${NAVIGATION_DOMAIN_ID:-42}"
  [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || {
    echo "NAVIGATION_DOMAIN_ID must be a non-negative integer: $ROS_DOMAIN_ID" >&2
    exit 2
  }
fi

if [[ ! -f "$NAV_WS/install/navigation_bringup/share/navigation_bringup/package.xml" ]]; then
  echo "ERROR: navigation_bringup is not built in $NAV_WS/install - run $SRC_DIR/build.sh first." >&2
  exit 1
fi
set +u
source "$NAV_WS/install/setup.bash"
set -u

# Two stacks on one ROS domain would both publish /cmd_vel.
running="$(pgrep -af 'ros2 launch navigation_bringup|lib/swerve_navigation/|ros2 launch nav5_bringup|lib/nav5/' || true)"
if [[ -n "$running" ]]; then
  echo "REFUSING: a navigation stack is already running:" >&2
  echo "$running" >&2
  exit 1
fi

# Middleware transport. Must come before ros2 launch: participant configuration is read once.
# Cyclone DDS is the Humble default above. If NAVIGATION_RMW selects Fast DDS, this helper keeps
# its shared-memory transport off because that transport previously wedged individual readers.
# shellcheck source=scripts/dds_transport.sh
source "$SRC_DIR/scripts/dds_transport.sh"

# Keep RViz out of the main launch process so a GUI crash/restart cannot disturb the drivetrain or
# Nav2. rviz_docker.sh starts a second process in the same Humble container and ROS domain.
if [[ "$NAV_DISTRO" == humble && $RVIZ_EXPLICIT -eq 0 ]]; then
  LAUNCH_ARGS+=("rviz:=false")
  echo "RViz:       separate Humble process; on the host run:"
  echo "              ./src/autonomy/1_navigation/rviz_docker.sh --domain $ROS_DOMAIN_ID"
elif [[ "$NAV_DISTRO" == humble ]] && printf '%s\n' "${LAUNCH_ARGS[@]}" | grep -qx 'rviz:=true'; then
  echo "WARNING: embedded RViz couples GUI failure to the main launch; prefer rviz_docker.sh." >&2
fi

echo "$NAV_WHERE   ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0 (unset)}  RMW=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp (default)}"
echo "            DDS transport: $NAV_DDS_TRANSPORT"

if ((HARDWARE)); then
  # Brings the CANivore's SocketCAN network up (needs no sudo, commands no motion).
  if ! "$SRC_DIR/scripts/canivore_up.sh"; then
    echo "Could not bring the CANivore network up: is it plugged in? (check: caniv -l)" >&2
    exit 1
  fi
  # The pad has to be READABLE, not just present: in the robot container /dev/input/event* is
  # root:input 0660 and the container user is not in the host's input group unless
  # docker-compose's group_add says so - joy_node then silently finds no joystick at all.
  # SDL uses the EVENT devices, not the legacy /dev/input/js*: js0 is world-readable here, so
  # checking that one would pass while joy_node still sees nothing.
  have_input=0
  readable_input=0
  for dev in /dev/input/event*; do
    [[ -e "$dev" ]] || continue
    have_input=1
    [[ -r "$dev" ]] && readable_input=1
  done
  if ((have_input == 0)); then
    echo "REFUSING: no joystick found. The Xbox pad is the e-stop: plug it in first." >&2
    exit 1
  fi
  if ((readable_input == 0)); then
    gid="$(stat -c '%g' /dev/input/event* 2> /dev/null | head -1)"
    echo "REFUSING: /dev/input/* exists but this user cannot read it, so joy_node would find no" >&2
    echo "          gamepad - and the pad is the e-stop.  You are $(id -un), groups: $(id -Gn)." >&2
    echo "          The devices belong to group ${gid:-?}.  In the robot container:" >&2
    echo "            now:        sg hostinput -c '$0 --hardware'" >&2
    echo "            permanent:  group_add: [\"${gid:-996}\"] for this service in" >&2
    echo "                        robot/docker/docker-compose.yaml, then docker compose up -d" >&2
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
