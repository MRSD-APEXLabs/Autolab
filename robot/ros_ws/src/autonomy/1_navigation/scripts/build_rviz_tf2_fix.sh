#!/usr/bin/env bash
# Build the upstream Humble tf2 release containing the waitForTransform deadlock fix.
#
# Keep this as an RViz-only overlay. Replacing libtf2 underneath an already-running navigation
# stack would mix library versions between processes and is unnecessary for the UI hang.
set -euo pipefail

TF2_VERSION=0.25.24
TF2_TAG="$TF2_VERSION"
TF2_COMMIT=404b7224d623d614f18fa9738dbf1716403d857e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="$(cd -- "$SCRIPT_DIR/../../../.." && pwd)"
FIX_ROOT="${RVIZ_TF2_FIX_ROOT:-$ROS_WS/tf2_fix/$TF2_VERSION}"
SOURCE_DIR="$FIX_ROOT/cache/geometry2"
BUILD_DIR="$FIX_ROOT/build"
INSTALL_DIR="$FIX_ROOT/install"
LOG_DIR="$FIX_ROOT/log"

if [[ -r "$INSTALL_DIR/local_setup.bash" && -r "$INSTALL_DIR/lib/libtf2.so" ]]; then
  echo "RViz tf2 $TF2_VERSION overlay already exists: $INSTALL_DIR"
  exit 0
fi

# Humble's generated setup files reference optional variables before defining them.
set +u
source /opt/ros/humble/setup.bash
set -u
mkdir -p "$FIX_ROOT/cache" "$LOG_DIR"

if [[ ! -d "$SOURCE_DIR/.git" ]]; then
  if [[ -e "$SOURCE_DIR" ]]; then
    echo "Refusing to overwrite unexpected path: $SOURCE_DIR" >&2
    exit 1
  fi
  echo "Fetching ROS geometry2 $TF2_TAG..."
  git clone --quiet --depth 1 --branch "$TF2_TAG" \
    https://github.com/ros2/geometry2.git "$SOURCE_DIR"
fi

ACTUAL_COMMIT="$(git -C "$SOURCE_DIR" rev-parse HEAD)"
if [[ "$ACTUAL_COMMIT" != "$TF2_COMMIT" ]]; then
  echo "Unexpected geometry2 commit in $SOURCE_DIR" >&2
  echo "expected: $TF2_COMMIT" >&2
  echo "actual:   $ACTUAL_COMMIT" >&2
  exit 1
fi

echo "Building RViz-only tf2 $TF2_VERSION overlay..."
colcon --log-base "$LOG_DIR" build \
  --base-paths "$SOURCE_DIR/tf2" \
  --packages-select tf2 \
  --build-base "$BUILD_DIR" \
  --install-base "$INSTALL_DIR" \
  --merge-install \
  --allow-overriding tf2 \
  --cmake-args -DBUILD_TESTING=OFF

if [[ ! -r "$INSTALL_DIR/local_setup.bash" || ! -r "$INSTALL_DIR/lib/libtf2.so" ]]; then
  echo "tf2 build completed without the expected overlay library" >&2
  exit 1
fi

echo "RViz tf2 overlay ready: $INSTALL_DIR"
