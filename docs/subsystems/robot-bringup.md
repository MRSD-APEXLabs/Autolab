# Robot Bringup & the Autonomy Stack

**Purpose:** the launch scaffolding that starts everything on the robot side, and the map of the seven numbered autonomy stages — including which ones are real and which are stubs.

## The launch chain

```
robot.launch.xml                    robot/ros_ws/src/robot_bringup/launch/
├── pushes namespace $ROBOT_NAME (e.g. robot_1)
├── real.launch.xml | sim.launch.xml        (arg sim:=true by default)
├── robot_state_publisher.launch.py         (URDF from $ROBOT_URDF_FILE_PATH)
├── static_transforms.launch.xml            (world → map)
├── $(env AUTONOMY_LAUNCH_PACKAGE)          → autonomy.launch.xml
└── domain_bridge                           (params/domain_bridge.yaml)

autonomy.launch.xml                 …/autonomy/autonomy_bringup/launch/
└── includes each stage via env vars from .env:
    INTERFACE_ / NAVIGATION_ / MAPPING_AND_LOCALIZATION_ / PERCEPTION_ /
    LOCAL_ / GLOBAL_ / PLANNING_ / BEHAVIOR_  _LAUNCH_{PACKAGE,FILE}
```

- `sim.launch.xml` adds `stereo_image_proc` disparity/point-cloud nodes, RViz, and rqt.
- `real.launch.xml` adds the static TF `base_link → front_stereo_camera_link` (15° pitch).

```bash
ros2 launch robot_bringup robot.launch.xml               # sim
ros2 launch robot_bringup robot.launch.xml sim:=false    # real robot
ros2 launch autonomy_bringup autonomy.launch.xml         # autonomy stages only
```

!!! warning "Two launch-chain breakers"
    1. `.env` sets `GLOBAL_LAUNCH_PACKAGE="global_bringup"`, and **no such package exists** — `autonomy.launch.xml` fails there ([known issue #2](../known-issues.md)).
    2. `robot_bringup` itself is one of two packages that don't build (`colcon build --packages-ignore robot_bringup rviz_behavior_tree_panel` per `README_planning.md`) — yet it owns `robot.launch.xml`.
    In practice, stages are launched individually as shown in [Running the System](../running.md#1-robot-stack).

## The seven stages

| Stage | Package(s) | Status | Notes |
|---|---|---|---|
| `0_interface` | `interface_bringup` | **stub** — empty launch | `ReadMe.md` there specs a robot-health interface that isn't implemented |
| `1_navigation` | `navigation_bringup`, `navigation_controller` | code exists, unused in lab routine | swerve-drive controller — see [Navigation](navigation.md) |
| `2_mapping_and_localization` | `mapping_and_localization_bringup` + vendored RTAB-Map (13 pkgs, git submodules) | vendored, **unwired** | bringup launch is empty; RTAB-Map never starts |
| `3_perception` | `perception_bringup` | **stub** — empty launch | real perception is off-board — see [Perception](perception.md) |
| `4_local` | `local_bringup` (+ `local_planner.cpp`) | **stub** | |
| `5_planning` | `base_moveit_config_2`, `base_urdf`, `global_planner`, `planning_bringup`, vendored `xarm_ros2` (11 pkgs) | **the core** | see [Planning](planning.md) |
| `6_behavior` | `behavior_tree`, `behavior_executive`, `manipulation_executive`, `lab_machine_executive`, `behavior_bringup`, rqt plugins | **active** | see [Behavior](behavior.md) |

Plus, outside the numbered stages:

- `robot_bringup` — top-level launch (this page)
- `autonomy_bringup` — the stage aggregator
- `logging/logging_bringup` — bag recording via `bag_record_pid` (`ros2 launch logging_bringup logging.launch.xml record_bag:=true`; config `config/log.yaml`: 4 GB MCAP bags, an `autolab` profile excluding `sensors/*` and a `zed` profile recording stereo/depth/IMU). Currently commented out of `robot.launch.xml`.
- `src/common/` — **empty on the host**; bind-mounted from `common/ros_packages/` inside the container (messages `behavior_tree_msgs`, `autolab_msgs`, `autolab_common`, `bag_recorder_pid`, `robot_descriptions`, `rqt_behavior_tree`, `rviz_behavior_tree_panel`). Build inside the container or these are missing.

## Container autolaunch

`robot/docker/docker-compose.yaml` starts a tmux session `robot_bringup` running `bws && sws && ros2 launch ${ROBOT_LAUNCH_PACKAGE} ${ROBOT_LAUNCH_FILE}` (the `robot-l4t` Jetson service appends `sim:="false"`). Attach with `tmux attach -t robot_bringup`. Set `AUTOLAUNCH=false` in `.env` to get an idle container instead.

## Config files

| File | Purpose |
|---|---|
| `robot/ros_ws/src/robot_bringup/params/domain_bridge.yaml` | Topics forwarded between ROS domains 0↔1 (robot side) — ⚠ included by absolute container path `/home/robot/AutoLab/...` in `robot.launch.xml` |
| `robot/ros_ws/src/robot_bringup/rviz/robot.rviz` | Robot-side RViz layout |
| `robot/docker/.bashrc` | `bws`/`sws`/`cws`/`rws` helpers; ROS_DOMAIN_ID logic (derives from robot number, then forces 0 — [known issue #6](../known-issues.md)) |
