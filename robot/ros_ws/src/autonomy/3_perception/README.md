# 3_perception: ZED X / ZED X Nano cameras, stereo depth, camera UI, perception and live ACT inference

This folder is self-contained. It holds the camera stack and the inference half of the teleop/ACT pipeline, ready
to move into a larger codebase:

| folder | what | depends on |
|---|---|---|
| `zedx_nano_camera/` | ROS 2 package (ament_python). Publishes a camera feed from the Xavier: JPEG, raw BGR and `CameraInfo` for each eye. Serves either camera of the camera hub (`camera` parameter). Also holds `camera_hub_node`, the ROS bridge to the hub (status topic, select services) | `rclpy`, `sensor_msgs`, `std_srvs`, `cv_bridge`, numpy, OpenCV. No ZED SDK |
| `zedx_nano_depth/` | ROS 2 package. Subscribes to the stereo pair, rectifies it, computes depth with RAFT-Stereo (GPU) or SGBM (CPU) and publishes depth plus the rectified left image | `zedx_nano_camera`; torch for the neural backend |
| `camera_ui/` | ROS 2 package. Web page on port 8001 that switches the hub between the two cameras and shows the active camera's RGB and depth. Its launch file starts the whole camera stack | `zedx_nano_camera`, `sensor_msgs`, `std_srvs`, `cv_bridge`, numpy, OpenCV |
| `camera_perception/` | ROS 2 package. AprilTags, YOLO detections and (ZED X only) a point cloud from the depth node's outputs, published under the topic names of the old Autolab `pc3.py` bridge (`/inspect/*`, `/servo/*`, `/zed_pointcloud`) | `zedx_nano_depth`, `camera_ui` (launch only), `cv_bridge`; pip: `pupil-apriltags`, `ultralytics` (torch) |
| `xavier_camera_hub/` | Not a ROS package (`COLCON_IGNORE`). HTTP server that runs **on the Xavier** (`/home/autolab/camera_hub/`) and streams one camera at a time. See [its README](xavier_camera_hub/README.md) | Python 3.8, numpy, OpenCV, pyzed (ZED X), `v4l2-ctl` (Nano) |
| `act_inference/` | Plain Python, not a ROS package (`COLCON_IGNORE`). Live ACT rollout on the xArm6. Contains a trimmed copy of the ACT/DETR model code | the ROS packages above as libraries; torch, xArm SDK |

The package names still say "nano", but `zedx_nano_camera` and `zedx_nano_depth` serve both cameras.

Relationship:

```
Xavier: xavier_camera_hub (port 8090), streams ONE camera at a time
   │  ZED X (SN 42757821, base) via pyzed   or   ZED X Nano (SN 99292912, wrist) via raw V4L2
   │  /select, /status, /cameras/<cam>/{video_feed/stereo,calibration.conf,info}, /time
   │
   ├──► camera_hub_node ──► /camera_hub/status, /camera_hub/select_{zedx,zedx_nano}, /camera_hub/release
   ├──► zedx_nano_camera (camera:=zedx)      ──► /zedx/{left,right}/…      ──► zedx_nano_depth ──► /zedx/depth/…
   ├──► zedx_nano_camera (camera:=zedx_nano) ──► /zedx_nano/{left,right}/… ──► zedx_nano_depth ──► /zedx_nano/depth/…
   │                                                                             │
   │                               camera_ui (http://<thor>:8001/) ◄─────────────┤ both namespaces + /camera_hub
   │                               camera_perception ◄───────────────────────────┤ /inspect/*, /zed_pointcloud, /servo/*
   │                               act_inference --camera-source ros ◄───────────┘
   └──► legacy root routes (Nano only): act_inference --camera-source nano-stream, teleop/nano_stream.py
```

The nodes of the camera that is not selected stay connected to the hub and wait; they start publishing within
about a second of a switch. Both camera sources of `act_inference` feed the policy identical frames. The
rectification maps, depth pipeline and ACT preprocessing are copies of `teleop/` and `act/` (see
[Verification](#verification)). The old Nano-only server (`scripts/zedx_nano_v4l2_stream.py`,
`zedx-nano-stream.service`) stays in the repo root; the hub replaces it on port 8090 and keeps its routes.

## Camera UI (ZED X and ZED X Nano)

Build the packages (see [Build](#build)), then on the Thor:

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
source /home/labx/apple_server_manip/.venv/bin/activate          # torch for the RAFT depth nodes
ros2 launch camera_ui camera_ui.launch.xml models_dir:=/home/labx/apple_server_manip/data/models
```

Open `http://<thor>:8001/`. The two buttons at the top right select the ZED X (base) or the ZED X Nano (wrist);
**Release** stops both. The left panel shows the rectified left image and the right panel shows depth (TURBO:
red near, blue far, black invalid) with its colorbar, auto range or a fixed near/far range in mm. Hovering over
either image shows the depth and the optical-frame XYZ at that pixel. **PNG** saves the RGB image, **PNG 16-bit**
saves the depth in mm. The rates, frame age, valid-pixel ratio and median depth appear under each panel.

A switch takes about 6 s to the ZED X (the ZED SDK opens the camera) and about 2 s to the Nano. The same switch
from a terminal or another node:

```bash
ros2 service call /camera_hub/select_zedx std_srvs/srv/Trigger
ros2 service call /camera_hub/select_zedx_nano std_srvs/srv/Trigger
ros2 service call /camera_hub/release std_srvs/srv/Trigger
ros2 topic echo --once /camera_hub/status                        # JSON: active camera, state, rates
curl -X POST -H 'Content-Type: application/json' -d '{"camera": "zedx"}' http://<thor>:8001/api/select
```

`/api/select` accepts only `Content-Type: application/json` and refuses cross-origin requests, so another web
page cannot switch the camera. The hub's own page, `http://192.168.1.101:8090/`, also switches cameras and shows
the raw left/right preview.

`camera_ui.launch.xml` arguments:
- `with_cameras` (true): also start `cameras.launch.xml`, i.e. the hub bridge plus a camera node and a depth node
  per camera. Use `with_cameras:=false` when the camera stack already runs.
- `ui_host` (0.0.0.0) and `ui_port` (8001): where the page is served. The page can switch cameras, so use
  `ui_host:=127.0.0.1` to keep it off the network.
- `host` and `port`: the Xavier hub (192.168.1.101, 8090).
- `backend`, `model` and `models_dir`: passed to both depth nodes.
- `ui_params_file`: the UI parameters, `config/camera_ui.yaml` by default.

`ros2 launch camera_ui cameras.launch.xml` starts the camera stack without the page.

UI HTTP API, for scripts:

| request | answer |
|---|---|
| `GET /api/status` | hub status, per-camera stream rates, depth statistics, intrinsics |
| `POST /api/select` `{"camera": "zedx" \| "zedx_nano" \| null}` | switch (null releases); waits up to 45 s for the first frame |
| `GET /api/depth_at?camera=&u=&v=` | depth (mm) and optical-frame XYZ (mm) at normalized image coordinates |
| `GET /stream/<cam>/rgb.mjpg` | rectified left image; the raw JPEG while no depth node runs |
| `GET /stream/<cam>/depth.mjpg?near=&far=&auto=1` | colorized depth |
| `GET /snapshot/<cam>/rgb.png`, `/snapshot/<cam>/depth.png` | the latest frame; depth as 16-bit PNG in mm |

## Perception: AprilTags, YOLO and the ZED X point cloud

`camera_perception` replaces the Xavier's Autolab-Camera-Edge "inspect" (ZED X) and "servo" (Nano) workers and the
`~/coding/Autolab/pc3.py` bridge that republished them. It runs on the Thor, on the rectified image and depth of the
camera stack above, and publishes under the topic names `pc3.py` used, so the planner (`move_to_pose_node`) and
MoveIt read it unchanged. There is one node per camera: `/zedx/zedx_perception` and `/zedx_nano/zedx_nano_perception`.
Like the depth nodes, the node of the camera the hub is not streaming logs once that it is idle and waits.

One-time setup, in the venv that runs the nodes (it must see the system site packages). `--no-deps` keeps pip from
installing `opencv-python`, which would shadow the system cv2 that `cv_bridge` uses:

```bash
source /home/labx/apple_server_manip/.venv/bin/activate
pip install pupil-apriltags
pip install --no-deps ultralytics 'filelock>=3.16.1' cloudpickle ultralytics-thop nvidia-ml-py
```

The YOLO weights are in `data/models/yolo/`:
- `best_wrist.pt`: oriented boxes, class `wellplate`, from Camera-Edge servo. The default for both cameras.
- `best_top.pt`: the same kind of model, from Camera-Edge inspect. It did not find the clear plate on the ZED X
  deck (below).
- `best_seg.pt`: segmentation, from `~/coding/Autolab`.

Run it next to the camera stack (the camera UI launch), or start the stack with it:

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
source /home/labx/apple_server_manip/.venv/bin/activate
ros2 launch camera_perception perception.launch.xml                          # camera stack already running
ros2 launch camera_perception perception.launch.xml with_cameras:=true \
  depth_models_dir:=/home/labx/apple_server_manip/data/models                  # also start hub bridge, camera, depth
ros2 launch camera_perception perception.launch.xml zedx_nano:=false cloud_mode:=snapshot fresh:=true
```

Use the same `ROS_DOMAIN_ID` as the camera stack and the planner.

### Topics

| `pc3.py` | camera_perception | type | contents |
|---|---|---|---|
| `/inspect/apriltags` | same | `MarkerArray` | `DELETEALL`, then one red 2 cm `SPHERE` per tag with a pose: `id` = tag id, ns `apriltags`, orientation = tag frame. Nothing else, because the planner stores every ADD marker's position under its id |
| `/inspect/wellplates` | same | `MarkerArray` | `DELETEALL`, then per detection with a pose a green `CUBE` (ns `wellplates`, id i, metric size) and a label (ns `wellplates_text`, id 1000+i). Detections are sorted by ascending confidence, so the most confident one is the last `wellplates` marker, the one the planner takes |
| `/zed_pointcloud` | same | `PointCloud2` | x, y, z float32 (+ packed `rgb`), frame `top_camera`, for MoveIt's octomap |
| `/inspect/image`, `/inspect/annotated_image` | same | `Image` bgr8 | the input image, and the image with tags, boxes and axes drawn. Published only while subscribed |
| – | `/inspect/json` | `String` | the frame's tags and detections: pixel geometry, poses, pose source, tag size measured by depth, reprojection error |
| – | `/inspect/capture_snapshot` | `std_srvs/Trigger` | snapshot mode: capture a new cloud |
| `/servo/image`, `/servo/annotated_image`, `/servo/detections` | same | as above | Nano. Detections: the best one only, ns `servo_detections` |
| – | `/servo/apriltags`, `/servo/json` | as above | Nano tags and JSON |
| `/servo/target` | **not published** | | it needs a Nano hand-eye calibration, which doesn't exist yet |

All outputs carry the image stamp. The ZED X outputs are labelled `top_camera`, the frame that the robot's TF tree
(`Chassis_1 → top_camera`) and the planner use for this camera. The launch file also publishes an identity
`top_camera → zedx_left_camera_optical_frame` (`top_camera_tf:=false` turns it off), so the `/zedx` camera
topics fit into that tree too. The Nano outputs keep `zedx_nano_left_camera_optical_frame`.

### What is computed

- **AprilTags:** `pupil_apriltags`, tag36h11, ids 0, 1, 2, 21 and 22, 62.5 mm, at full resolution.
  - Pose from `solvePnPGeneric` (IPPE_SQUARE) on the corners and the tag size. The depth inside the tag gives a
    plane that picks between the two mirror-image PnP solutions and measures the tag's actual size. The node warns
    when that size is more than 15 % off; set `apriltags.sizes` (`'21=0.075'`) for tags printed at another size.
  - `apriltags.position: depth` takes the position from the depth plane instead of the size.
  - The orientation is the AprilTag library's tag frame (x right, y down, z into the tag, in the tag's own
    orientation). The Camera-Edge code built it from image-sorted corners, so its orientation differed.
    `calibrate_top_camera.py` fits position only by default.
  - A small tag seen face-on (~40 px) has a few degrees of tilt uncertainty. Its position and in-plane angle stay
    sharp.
- **YOLO:** ultralytics, on the GPU (falls back to CPU). Oriented-box, box and segmentation models are handled.
  The ZED X keeps only the class `wellplate` (`yolo.classes`), since the planner takes the last marker on
  `/inspect/wellplates` whatever its class.
  - On the ZED X the model input gets CLAHE first (`yolo.clahe_clip` 3, `yolo.clahe_tiles` 4; contrast
    equalization of the lightness in 4×4 tiles). The deck is dim, and a clear plate on it is invisible to the
    models without it. Outputs and the annotated image use the original image.
  - Each detection is placed with a RANSAC plane fit on the depth inside its box or mask. Position: the centre ray
    meets the plane. z: the plane normal toward the camera. x: along the box's width, which is the long side for
    oriented boxes. Size: in metres.
  - Detections without valid depth appear in the JSON and the annotated image but get no marker. `pc3.py` gave
    those a pixel-scaled placeholder.
  - If the model fails to load or run, the node logs the error and continues with tags and the cloud.
- **Point cloud (ZED X only):**
  - Every 4th pixel of the depth, cropped to x, y ∈ ±0.6 m and z ∈ 0.2–1.7 m (camera frame). Points on depth
    edges (a jump over 2 % of the depth per pixel) are dropped, then one point is kept per 1 cm voxel: about
    10k points per cloud.
  - `cloud_mode:=live` (default): a new cloud up to 5 times a second, so the octomap follows the scene.
  - `cloud_mode:=snapshot`: `pc3.py`'s behaviour. 10 frames are stacked once, saved to
    `~/.ros/camera_perception/zedx_snapshot.npy` and republished at 5 Hz stamped with the current time. A cached
    cloud is loaded at start-up, even without a camera. `pc3.py`'s N×3 `.npy` files load as well.
    `/inspect/capture_snapshot` or `fresh:=true` captures a new one.
  - With `fresh:=true` or after `capture_snapshot`, the old cloud stays published until the new one is complete.
    Then the node calls `/clear_octomap` and publishes the new cloud. So MoveIt is never left without obstacles,
    even if the hub is streaming the Nano. If move_group doesn't answer within 40 timer ticks (8 s at 5 Hz), the
    node publishes anyway. Live mode with `fresh:=true` clears the same way, before its first cloud.
    A failed cache write is logged, and the cloud is still published.

The ZED X node processes at most 10 frames/s (`max_rate`). At every frame (~16/s) it would take about 1.3 CPU
cores, mostly for the full-resolution tag detection. The Nano node processes every frame.

`perception.launch.xml` arguments:
- `zedx`, `zedx_nano` (true): which cameras get a node.
- `yolo_models_dir` (`data/models/yolo`), `zedx_model` (best_wrist.pt), `zedx_nano_model` (best_wrist.pt),
  `device` (cuda:0).
- `cloud_mode` (live | snapshot) and `fresh` (false).
- `top_camera_tf` (true).
- `zedx_params_file`, `zedx_nano_params_file`: `config/zedx.yaml` and `config/zedx_nano.yaml` hold every other
  parameter, with comments.
- `with_cameras` (false), `host`, `port`, `depth_backend`, `depth_models_dir`: for starting the camera stack too.

## Live ACT inference

Prediction-only by default: robot state is read, the policy runs, the limited targets are
printed, and the arm is not enabled or commanded. Keep `policy_best.ckpt`, `config.pkl` and
`dataset_stats.pkl` together in the checkpoint folder. RGB versus RGB-D is detected from the checkpoint.

From the repo root, with the project venv:

```bash
OMP_NUM_THREADS=2 .venv/bin/python 2_manipulation/act_inference/run_act_inference.py \
  --ckpt-dir data/act_ckpts/xarm6_rgbd --duration-sec 30 --preview \
  --depth-models-dir data/models
```

`--depth-models-dir data/models` reuses the RAFT weights already in the repo. Without it the
weights go to `$ZEDX_NANO_DEPTH_MODELS` or `~/.cache/zedx_nano_depth` and are downloaded
(40 MB) on the first run.

For an operator-attended run, place the arm at the demonstrated start pose, stop any other
controller (teleop, AutoLab planning) and add `--execute`:

```bash
OMP_NUM_THREADS=2 .venv/bin/python 2_manipulation/act_inference/run_act_inference.py \
  --ckpt-dir data/act_ckpts/xarm6_rgbd --duration-sec 5 --translation-speed 0.01 \
  --depth-models-dir data/models --execute --preview
```

Ctrl-C, Enter, or `q`/Esc in the preview stops the run. Every target is speed-limited
(`--translation-speed` 0.025 m/s, `--rotation-speed` 0.1 rad/s) and must stay inside
`--workspace-radius` (0.1 m) of the start pose. The run stops if a prediction is older than
`--prediction-timeout` (0.75 s) or the camera frame is stale (`--max-frame-age` 0.25 s plus
two depth periods). Orientation is held unless `--enable-rotation` is given. There is no gripper output.

### Camera sources

- `--camera-source nano-stream` (default) reads stereo pairs from the Xavier (`--camera-host`
  192.168.1.101, `--camera-port` 8090) and computes depth in this process (`--depth-backend
  neural|sgbm`, `--depth-model`, `--depth-width`). It needs no ROS.
- `--camera-source ros --ros-namespace /zedx_nano` reads `left/image_rect_color`,
  `depth/image_rect` and `depth/camera_info` from a running `zedx_nano_depth` node, so one
  depth network serves both the robot stack and the policy. Source ROS first
  (`source /opt/ros/humble/setup.bash`) and use the same `ROS_DOMAIN_ID` as the nodes. The
  `--depth-*` options don't apply here; the depth node's parameters do. It runs a private
  rclpy context, so it can be embedded in a process that already uses rclpy.

`run_act_inference.py --help` lists all options. `act_inference/requirements.txt` lists the Python dependencies.

### Using it as a library

`act_inference/act_inference/__init__.py` adds `zedx_nano_camera/` and `zedx_nano_depth/` to
`sys.path` when they sit next to it. So once `2_manipulation/act_inference` is on the path, no
build step is needed:

```python
import sys; sys.path.insert(0, '2_manipulation/act_inference')
from pathlib import Path
from act_inference.config import InferenceConfig, RolloutConfig
from act_inference.rollout import run_act_rollout

run_act_rollout(InferenceConfig(camera_source='ros', ros_namespace='/zedx_nano'),
                RolloutConfig(ckpt_dir=Path('data/act_ckpts/xarm6_rgbd'), duration_sec=30))
```

Lower-level pieces:
- `rollout.load_policy` and `rollout.predict_chunk` for the model.
- `cameras.make_camera` for a `CameraFrame` source.
- `robot.XArm6Robot(read_only=True)` for the arm.

## ROS 2 packages

### Build

The packages were originally built and tested against ROS 2 Jazzy (Python 3.12); this repo runs Humble
(Python 3.10). Audited 2026-09-22: no Jazzy-only rclpy/launch API or Python 3.11+/3.12-only syntax found in
`zedx_nano_camera`, `zedx_nano_depth`, `camera_ui` or `camera_perception` — none was ever run on Humble though,
so treat a first build as unverified. Link or copy them into a workspace:

```bash
mkdir -p ~/ros2_ws/src && ln -s /home/labx/apple_server_manip/2_manipulation ~/ros2_ws/src/2_manipulation
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select zedx_nano_camera zedx_nano_depth camera_ui camera_perception
source install/setup.bash
```

colcon skips `act_inference` and `xavier_camera_hub` because of their `COLCON_IGNORE`. Build **without**
`--symlink-install`. The node scripts then use `#!/usr/bin/env python3`, so they run in whichever Python is first
on `PATH`. With `--symlink-install`, colcon writes `#!/usr/bin/python3` into them instead, and the depth nodes
fail because the system Python has no CUDA torch. The neural depth
backend needs torch, so activate a venv that has torch and the system site packages before
`ros2 launch`, e.g. `source /home/labx/apple_server_manip/.venv/bin/activate`. The camera node
and the `sgbm` backend need only the system Python. torch isn't a rosdep key, so `package.xml` doesn't declare it.

### Launch

The camera UI's launch file starts the full two-camera stack (see [Camera UI](#camera-ui-zed-x-and-zed-x-nano)):

```bash
ros2 launch camera_ui camera_ui.launch.xml models_dir:=/home/labx/apple_server_manip/data/models   # stack + page
ros2 launch camera_ui cameras.launch.xml models_dir:=/home/labx/apple_server_manip/data/models      # stack only
ros2 launch zedx_nano_camera camera_hub.launch.xml                                                 # hub bridge only
```

Single cameras:

```bash
# ZED X depth (also starts its camera node) in /zedx
ros2 launch zedx_nano_depth zedx_nano_depth.launch.xml camera:=zedx namespace:=zedx \
  models_dir:=/home/labx/apple_server_manip/data/models

# Nano camera only
ros2 launch zedx_nano_camera zedx_nano_camera.launch.xml host:=192.168.1.101

# depth, also starts the camera (with_camera:=true is the default)
ros2 launch zedx_nano_depth zedx_nano_depth.launch.xml models_dir:=/home/labx/apple_server_manip/data/models

# depth only, camera already running in the same namespace
ros2 launch zedx_nano_depth zedx_nano_depth.launch.xml with_camera:=false

# CPU depth, no torch needed
ros2 launch zedx_nano_depth zedx_nano_depth.launch.xml backend:=sgbm
```

Launch arguments:
- Camera: `namespace` (zedx_nano), `camera`, `name` (node name), `host`, `port`, `sensor`, `params_file`.
- Depth: `namespace`, `with_camera`, `camera`, `host`, `port`, `backend`, `model`, `models_dir`, `params_file`.
- Hub bridge: `host`, `port`, `params_file`.

`camera` selects a hub camera: `zedx` or `zedx_nano`. It also names the frames. Use a matching `namespace`.
The default, `''`, reads the hub's legacy root routes, which serve the Nano, and the old Nano-only server.
A single-camera launch does not switch the hub; select its camera in the UI or with the
`/camera_hub/select_*` services (with the hub bridge running) or `POST /select` on the Xavier.

The remaining parameters are read from `config/*.yaml`. Each file's header says which values the launch file overrides.

### Topics (namespaces `/zedx` and `/zedx_nano`)

Each camera has its own namespace with the same topics. The camera UI launch uses `/zedx` for the ZED X and
`/zedx_nano` for the Nano.

| topic | type | from | notes |
|---|---|---|---|
| `{left,right}/image_raw/compressed` | `CompressedImage` | camera | the Xavier's JPEG, forwarded without re-encoding |
| `{left,right}/image_raw` | `Image` bgr8 | camera | unrectified; decoded only while something subscribes |
| `{left,right}/camera_info` | `CameraInfo` | camera | factory K, D and stereo rectification R, P. Right `P[0,3] = -fx·B` in meters |
| `depth/image_rect` | `Image` 16UC1 | depth | depth in mm, 0 = invalid, aligned to `left/image_rect_color` |
| `depth/camera_info` | `CameraInfo` | depth | rectified pinhole model (D = 0) for `depth/image_rect` and `left/image_rect_color` |
| `left/image_rect_color` | `Image` bgr8 | depth | the rectified left image that the depth belongs to |
| `depth/colorized` | `Image` bgr8 | depth | preview: red near, blue far, black invalid. Computed only while subscribed |

With `sensor:=left` or `right`, the camera publishes only that eye; the depth node needs `stereo`.

The hub bridge, `camera_hub_node`, is not namespaced:

| name | type | notes |
|---|---|---|
| `/camera_hub/status` | `std_msgs/String` (JSON), transient local | the hub's `/status` (`active`, `state`, `error`, per-camera `fps`) plus `reachable`, `poll_error`, `hub_url`; published after every poll |
| `/camera_hub/select_zedx`, `/camera_hub/select_zedx_nano` | `std_srvs/Trigger` | switch the hub; returns when the camera streams (`success`, `message` e.g. `zedx streaming`) or fails |
| `/camera_hub/release` | `std_srvs/Trigger` | stop both cameras |

### Parameters

| node | parameter | default | |
|---|---|---|---|
| camera | `host`, `port` | 192.168.1.101, 8090 | Xavier camera hub |
| camera | `camera` | '' | zedx or zedx_nano; '' = the hub's legacy root routes (the Nano) |
| camera | `sensor` | stereo | stereo, left or right |
| camera | `timeout` | 3.0 s | HTTP timeout; longest stall before reconnecting |
| camera | `left_frame_id`, `right_frame_id` | '' | '' = `<camera>_{left,right}_camera_optical_frame` (`zedx_nano_…` for camera '') |
| camera | `clock_sync_period` | 10 s | Xavier clock re-synchronization |
| camera | `max_sync_uncertainty` | 0.05 s | reject a clock sync that is less precise than this |
| camera | `reconnect_delay` | 1.0 s | |
| camera | `inactive_poll_period` | 1.0 s | how often to retry while the hub streams the other camera |
| depth | `backend` | neural | neural (RAFT-Stereo, GPU) or sgbm (OpenCV, CPU) |
| depth | `model` | raft-realtime | raft-realtime, raft-middlebury, fast-foundation or a `.pth` path |
| depth | `models_dir` | '' | '' = `$ZEDX_NANO_DEPTH_MODELS` or `~/.cache/zedx_nano_depth`. Missing RAFT weights are downloaded |
| depth | `repo` | '' | fast-foundation only: checkout path ('' = `<models_dir>/Fast-FoundationStereo`). The repo's copy is `third_party/Fast-FoundationStereo` |
| depth | `match_width` | 640 | px; the width at which disparity is computed |
| depth | `max_depth_mm` | 4000 | farther depth is published as 0 |
| depth | `input_transport` | compressed | compressed (JPEG) or raw (`image_raw`) |
| depth | `colorize_near_mm`, `colorize_far_mm` | 100, 1000 | `depth/colorized` range |
| camera_hub | `host`, `port` | 192.168.1.101, 8090 | Xavier camera hub |
| camera_hub | `cameras` | [zedx, zedx_nano] | one `select_<camera>` service each |
| camera_hub | `poll_period`, `timeout` | 1.0 s, 3.0 s | `/status` polling |
| camera_hub | `select_timeout` | 30 s | how long a select waits for the camera's first frame |
| camera_ui | `host`, `port` | 0.0.0.0, 8001 | where the page is served (launch: `ui_host`, `ui_port`) |
| camera_ui | `cameras`, `labels` | [zedx, zedx_nano], [ZED X (base), ZED X Nano (wrist)] | topics are read from `/<camera>/…` |
| camera_ui | `max_fps`, `jpeg_quality` | 15, 80 | per browser stream |
| camera_ui | `stale_after` | 1.0 s | older frames are marked stale on the page |
| camera_ui | `hub_node` | /camera_hub | the hub bridge's status topic and services |

### Stamps, frames and clocks

- Header stamps are the capture time. The camera node maps the Xavier's clock to its own ROS clock and re-synchronizes every 10 s.
- Both eyes of a stereo pair carry the left capture stamp. The depth node pairs left and right by that exact stamp.
- The depth outputs keep the stamp and `frame_id` of the left image. As with `image_proc`, the rectified images keep the camera's frame id.
- No TF is published. The integrating stack provides the transforms to `zedx_nano_left_camera_optical_frame` (wrist) and `zedx_left_camera_optical_frame` (base).
- The ZED X images come from the ZED SDK unrectified, like the Nano's, so both go through the same rectification from the factory calibration. The ZED X baseline is 120 mm and the Nano's is 18 mm, so the ZED X is less precise up close and reaches farther.
- The depth node always works on the newest pair and drops pairs that arrive while it is busy, so latency stays at one compute period.
- The camera node reconnects on its own. While the hub streams the other camera it logs one line and waits; the depth node logs once that it is idle. The depth node exits if its backend fails, e.g. when the GPU worker process dies.
- `act_inference --camera-source ros` compares these stamps with its local clock. Run it on the same machine as the camera node, or keep both clocks synchronized (chrony/PTP).

## Tests

```bash
cd 2_manipulation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=73 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # keep test traffic off the robot network
ZEDX_NANO_DEPTH_TEST_MODELS=/home/labx/apple_server_manip/data/models \
  ../.venv/bin/python -m pytest -q -p no:cacheprovider zedx_nano_camera/test zedx_nano_depth/test act_inference/tests camera_ui/test
YOLO_OFFLINE=1 ../.venv/bin/python -m pytest -q -p no:cacheprovider camera_perception/test
../.venv/bin/python -m pytest -q -p no:cacheprovider xavier_camera_hub/tests    # the hub, with fake cameras
```

The tests use a synthetic Xavier server, a stub robot and a tiny random checkpoint. They need
no hardware and never connect to the arm. The camera UI page test drives headless Chrome and is skipped without
`google-chrome` (or chromium) and the `websocket-client` package. The RAFT test runs only with a CUDA GPU and
`ZEDX_NANO_DEPTH_TEST_MODELS`; the ROS tests are skipped when rclpy isn't importable. The perception tests render
tags and planes with exact depth; they load the real YOLO models from `data/models/yolo` (or
`CAMERA_PERCEPTION_TEST_MODELS`) when present and use a fake detector for the node. In a workspace,
`colcon test --packages-select zedx_nano_camera zedx_nano_depth camera_ui camera_perception && colcon test-result --verbose`
runs the ROS packages' tests.

## Verification

Checked on this machine (Jetson, Jazzy, torch 2.14 + CUDA) against the original `teleop/` and `act/` code:

- **Rectification maps:** bit-identical to `teleop` `StereoCalibration`, with 5-coefficient and rational distortion. This holds both from the `.conf` directly and through `CameraInfo`.
- **ACT actions:** bit-identical to the original for an RGB checkpoint and an RGB-D checkpoint.
- **Camera frames:** image, depth, headers, capture time and calibration are identical to the original `NanoStreamCamera` with SGBM and with RAFT.
- **ROS against a fake Xavier at 15 fps:** about 14 Hz on every topic, for SGBM and for RAFT.
- **Rollout, both camera sources:** with a stub robot, prediction-only rollouts run the 50 Hz loop.
- **Live cameras (2026-09-22), full camera UI stack on the Thor against the hub on the Xavier:**
  - Nano: 30 Hz raw, depth at 16-17 fps (RAFT realtime, ~42 ms per pair), 100 % valid pixels.
  - ZED X: depth at ~16 fps, 100 % valid pixels.
  - Switches both ways from the page and from the `/camera_hub/select_*` services: ~6.2 s to the ZED X,
    ~1.9 s to the Nano.
  - `teleop/nano_stream.py` still reads the Nano at 30 fps through the hub's legacy routes.
- **Perception (2026-09-22), on the running camera stack.** Outputs were kept on test topics, not the `pc3.py` names:
  - ZED X, every frame: 15 frames/s of the 16-17 Hz depth. Per frame: tags 38 ms (CPU), YOLO 22 ms (GPU).
    Clouds: 8-9 ms, ~4.3/s, 9.5-12k points in `top_camera`. About 1.3 CPU cores.
  - The 10 Hz `max_rate` cap, checked on the Nano's 14 Hz depth: 10.0 frames/s.
  - ZED X tags 1 and 21 at 0.7-0.8 m: position jitter under 1 mm. The depth measured them at 64.0 and 60.9 mm,
    so 62.5 mm fits both.
  - Nano: 13.8 frames/s, tags 26 ms, YOLO 33 ms.
  - Nodes idle while the hub streams the other camera, and exit cleanly on Ctrl-C.
  - Snapshot mode and `/clear_octomap` were tested only against fakes (unit tests).
- **Wellplate detection (2026-09-22), 8 saved ZED X frames** of a clear 96-well plate on the Opentrons deck:
  - Tried `best_top.pt` (current and two older versions from the Xavier), `best_wrist.pt` (two versions) and
    `best_seg.pt`. For each: 26 preprocessings (brightness/contrast, gamma, CLAHE, auto levels, gray world,
    sharpening) × full frame at 640 and 960, 2×2 tiles, or a deck crop.
  - `best_top.pt` scored at most 0.003 on the plate in every setting. Its only hit (0.40) was an empty deck slot.
  - `best_wrist.pt` + CLAHE: 0.90–0.92 on every frame, with no other detection above 0.01. Clip limits of 1.5–4
    all score 0.85 or more. The same model without CLAHE: nothing.
  - Pose from depth: stable to ~2 mm; size 129–135 × 91–102 mm (a standard plate is 128 × 86 mm).
  - Detection time: 33 ms with CLAHE (6 ms of it).
  - `camera_perception/test/data/zedx_clear_plate.jpg` keeps one of these frames as a regression test.
- **Not covered by these checks:** a live ACT rollout on the xArm.

## Not included

These parts are not included, and the originals remain in the repo root:
- teleoperation, SpaceMouse and recording
- the web UI
- the Camera-Edge source
- training and dataset conversion
- the `--dry-run` and `--no-depth` rollout modes

## TODO

- `package.xml` of the four ROS packages still says `TODO: License declaration`.
- The vendored code keeps its own licenses:
  - `act_inference/act_inference/act/LICENSE` (MIT)
  - `act_inference/act_inference/act/detr/LICENSE` (Apache-2.0)
  - `zedx_nano_depth/zedx_nano_depth/raft_stereo/LICENSE` (MIT)
