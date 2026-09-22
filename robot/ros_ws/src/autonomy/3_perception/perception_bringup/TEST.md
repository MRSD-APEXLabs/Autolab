# Testing `perception_bringup` (and the perception stack it launches)

This covers everything needed to build, unit-test, and smoke-test the perception stack on this repo's
Humble containers. It's a port of a stack originally built/tested on ROS 2 Jazzy (Python 3.12); the source
was audited for Jazzy-only syntax/API on 2026-09-22 (see `../README.md`) but had never actually been run
on Humble before that, so treat a first build here as unverified until you've done the steps below.

## 1. Start the container

```bash
autolab up robot
autolab connect robot
```

## 2. Build — do NOT use the `bws` shell alias

`bws` runs `colcon build --symlink-install`. That breaks `zedx_nano_camera`, `zedx_nano_depth`, `camera_ui`
and `camera_perception`: colcon rewrites their node scripts' shebang from `#!/usr/bin/env python3` to
`#!/usr/bin/python3`, which then runs the system Python (no CUDA torch) instead of whichever Python is
first on `PATH`. Build manually instead:

```bash
cd /home/robot/AutoLab/robot/ros_ws
colcon build --packages-select zedx_nano_camera zedx_nano_depth camera_ui camera_perception perception_bringup
source install/setup.bash
```

## 3. Install the Python deps `camera_perception` needs

The container already has `torch`, `torchvision`, `numpy`, `opencv-python` (global pip, no venv) and
`cv_bridge`/`python3-opencv` (from `ros-humble-desktop`). Missing: `pupil-apriltags` and `ultralytics`.

```bash
pip install pupil-apriltags
pip install --no-deps ultralytics 'filelock>=3.16.1' cloudpickle ultralytics-thop nvidia-ml-py
```

`--no-deps` is required on `ultralytics` — plain `pip install ultralytics` would reinstall `opencv-python`
and can shadow the system `cv2` that `cv_bridge` needs.

YOLO weights (`best_wrist.pt` etc.) are **not in this repo**. Without them, `camera_perception`'s node
still starts, logs a load error, and keeps publishing AprilTags/point cloud (see README "What is
computed" — YOLO failure is non-fatal). Copy real weights into `yolo_models_dir` if you want detections;
otherwise skip and only validate tags/cloud/hub wiring.

## 4. Unit tests first — this is what actually proves the Humble port works

These use a synthetic Xavier server and a stub robot, so they need **no real camera hardware** and are
the right way to validate the port before touching anything live:

```bash
export ROS_DOMAIN_ID=73 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # keep test traffic off the robot network
colcon test --packages-select zedx_nano_camera zedx_nano_depth camera_ui camera_perception --event-handlers=console_direct+
colcon test-result --verbose
```

For `camera_perception` specifically, if you don't have real YOLO weights available:

```bash
YOLO_OFFLINE=1 colcon test --packages-select camera_perception --event-handlers=console_direct+
```

Expected skips, not failures:
- RAFT-Stereo (neural depth) test needs a real CUDA GPU — only runs if the container has `runtime: nvidia`
  and a GPU is actually present. SGBM (CPU) backend tests always run.
- `camera_ui` page test drives headless Chrome — skipped without `google-chrome`/`chromium` +
  `websocket-client` installed in the container.
- rclpy-dependent tests are skipped if `rclpy` isn't importable (shouldn't happen here — it's ROS base).

## 5. Launch smoke test via `perception_bringup`

This is the thing we just built — verify it starts the full stack **except** the `camera_ui` web page:

```bash
ros2 launch perception_bringup perception.launch.xml
```

In another shell on the same container:

```bash
ros2 node list
# expect: /camera_hub, /zedx/zedx_camera, /zedx/zedx_depth, /zedx_nano/zedx_nano_camera, /zedx_nano/zedx_nano_depth,
#         /zedx/zedx_perception, /zedx_nano/zedx_nano_perception
# must NOT see a camera_ui / ui_node — if it's there, with_cameras wiring pulled in the wrong launch file

curl -sf http://127.0.0.1:8001/ && echo "UI PAGE IS RUNNING (should have failed to connect)"

ros2 topic echo --once /camera_hub/status
```

### Important limitation: no real cameras reach this container

SITL runs on `autolab_network` (172.31.0.0/24), not the robot's LAN, so the Xavier camera hub at
`192.168.1.101:8090` (the default `host` arg) is unreachable here. With no hub reachable:
- `camera_hub_node` logs poll errors and publishes `/camera_hub/status` with `reachable: false` — that's
  expected in SITL, not a bug.
- The camera/depth nodes will sit idle waiting for the hub; no `/zedx/...` or `/zedx_nano/...` image/depth
  topics will actually carry data.
- `camera_perception` nodes still come up and stay alive (they tolerate absent depth), but publish nothing
  useful without a live feed.

So the launch smoke test above only proves **node wiring and startup are correct**, not that images flow
end to end. For that you need the real Xavier reachable — either:
- point `host`/`port` at a reachable test server if you have one, or
- run on the `hitl`/`deploy` profile on the actual Jetson, on the robot's LAN.

## 6. What "done" looks like

- `colcon build` succeeds for all 5 packages with no `--symlink-install`.
- `colcon test` passes (or only has the expected hardware-gated skips from step 4).
- `ros2 launch perception_bringup perception.launch.xml` brings up every node in the "expect" list above,
  port 8001 refuses connections, and `/camera_hub/status` publishes (even if `reachable: false` in SITL).
- If you have hub/network access or real weights, confirm `/inspect/*`, `/servo/*` and `/zed_pointcloud`
  actually carry data — that step needs the real Xavier and is out of scope for a SITL-only check.
