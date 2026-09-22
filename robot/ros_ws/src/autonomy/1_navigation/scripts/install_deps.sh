#!/usr/bin/env bash
# Install what navigation needs beyond the Autolab robot image: the Velodyne driver, Cyclone DDS,
# and the phoenix6 PYTHON package (the image's apt phoenix6 is C++, which the bridge cannot use).
#
#   autonomy/1_navigation/scripts/install_deps.sh
#
# Run it inside the robot container (uses sudo there).  Installs into the container, so it lasts
# until the container is recreated; robot/docker/Dockerfile.robot installs the same things
# into the image, so a rebuilt image does not need this.  Safe to re-run.  Touches no hardware.
set -euo pipefail

# Must match the firmware / C++ phoenix6 on the robot (26.3.0 is what drove it from the host).
PHOENIX6_VERSION="${PHOENIX6_VERSION:-26.3.0}"
PHOENIX6_DIR=/opt/phoenix6_python      # navigation.launch.py looks here first

if [[ -f /opt/ros/humble/setup.bash ]]; then
  DISTRO=humble
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  DISTRO=jazzy
else
  echo "ERROR: no ROS 2 Humble or Jazzy here." >&2
  exit 1
fi

apt_packages=()
if [[ ! -d "/opt/ros/$DISTRO/share/velodyne_driver" ||
      ! -d "/opt/ros/$DISTRO/share/velodyne_pointcloud" ]]; then
  apt_packages+=("ros-$DISTRO-velodyne")
fi
if [[ "$DISTRO" == humble && ! -d /opt/ros/humble/share/rmw_cyclonedds_cpp ]]; then
  apt_packages+=(ros-humble-rmw-cyclonedds-cpp)
fi
if ((${#apt_packages[@]})); then
  echo "apt: installing ${apt_packages[*]}"
  sudo apt-get update
  sudo apt-get install -y "${apt_packages[@]}"
else
  echo "velodyne and Cyclone DDS: already installed"
fi

if [[ -f "$PHOENIX6_DIR/phoenix6/__init__.py" ]] &&
   ls "$PHOENIX6_DIR"/phoenix6-"$PHOENIX6_VERSION".dist-info > /dev/null 2>&1; then
  echo "phoenix6 (Python) $PHOENIX6_VERSION: already in $PHOENIX6_DIR"
else
  # Its own directory, not site-packages: swerve_bridge adds exactly this directory to its path,
  # so no other Python program in the container can pick up (or be broken by) this phoenix6.
  echo "phoenix6 (Python) $PHOENIX6_VERSION: installing into $PHOENIX6_DIR"
  sudo /usr/bin/python3 -m pip install --no-cache-dir --upgrade --target "$PHOENIX6_DIR" \
    "phoenix6==$PHOENIX6_VERSION"
fi

echo
echo "Done.  Next: $(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/build.sh"
