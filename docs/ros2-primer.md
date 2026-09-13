# ROS 2 Primer (as used in this repo)

This page explains just enough ROS 2 (Robot Operating System 2, distribution **Humble**) to work on Autolab, grounding every concept in a real file from this repo. Skip it if you're already comfortable with ROS 2.

## Nodes, topics, messages

A **node** is a running program that talks ROS. A **topic** is a named, typed message bus: nodes *publish* to it and *subscribe* to it, many-to-many, without knowing about each other.

Concrete example from this repo: `manipulation_executive` (C++ node) publishes a `std_msgs/String` like `plan_wellplate` on the topic `/planning_command`; `move_to_pose_node` subscribes to `/planning_command`, plans and executes the motion, and publishes its progress (`PLANNING` → `EXECUTING` → `SUCCESS`) on `/planning_state`, which `manipulation_executive` watches. That pair of topics *is* the interface between the behavior layer and the planner.

Useful commands to poke at a live system:

```bash
ros2 node list                      # who is running
ros2 topic list                     # what buses exist
ros2 topic echo /planning_state     # watch messages live
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_home'}"
ros2 topic info /planning_command   # who publishes/subscribes, QoS
```

**Messages** are typed structs. Autolab defines its own in `common/ros_packages/behavior_tree_msgs/` — e.g. `ManipulationCommand{type, object_type, target_machine}` and `LabMachineCommand{device, action, parameters_json}`.

**Services** are request/reply calls (used sparingly here — e.g. `move_to_pose_node` calls `/controller_manager/switch_controller` to activate the arm's trajectory controller).

## QoS — a real gotcha here

Every publisher/subscriber has Quality-of-Service settings. The two that matter here: **RELIABLE** (TCP-like, default) vs **BEST_EFFORT** (lossy), and history depth. *Both sides must be compatible or messages silently don't arrive.* Autolab's behavior status topics (`behavior/<action>_status`) are **BEST_EFFORT with depth 1**, which is why `routine_executor` treats them as a live signal rather than a queue — and why `ros2 topic echo` on them needs `--qos-reliability best_effort` to see anything.

## Launch files

A **launch file** starts many nodes with parameters and remappings in one shot. Autolab uses XML launch files that include each other in a chain:

```
robot.launch.xml            (robot/ros_ws/src/robot_bringup/launch/)
└── autonomy.launch.xml     (…/autonomy/autonomy_bringup/launch/)
    ├── interface.launch.xml, navigation.launch.xml, …   (one per stage)
    ├── planning.launch.xml  → xarm6_moveit_realmove.launch.py (MoveIt + driver)
    └── behavior.launch.xml  → the executives + behavior tree
```

Run one with:

```bash
ros2 launch planning_bringup planning.launch.xml robot_ip:=192.168.1.236
```

A quirk of this repo: `autonomy.launch.xml` doesn't hardcode which package/file each stage uses — it reads them from **environment variables** (`PLANNING_LAUNCH_PACKAGE`, `BEHAVIOR_LAUNCH_FILE`, …) defined in the root `.env` and forwarded by docker compose. That indirection is also the source of [known issue #2](known-issues.md) (a stage points at a package that doesn't exist).

## Workspaces and colcon

ROS 2 code lives in a **workspace** (`robot/ros_ws`, `gcs/ros_ws`): sources under `src/`, built with **colcon** into `build/` and `install/`. You must *source* the install space in every new shell before ROS can find your packages.

Inside the Autolab containers, `robot/docker/.bashrc` and `gcs/docker/.bashrc` define shortcuts — use these instead of typing the long forms:

| Alias | Expands to | Meaning |
|---|---|---|
| `bws` | `colcon build --symlink-install …` | **b**uild **w**ork**s**pace |
| `sws` | `source install/local_setup.bash` | **s**ource **w**ork**s**pace |
| `cws` | (prompts, then deletes build/install/log) | **c**lean **w**ork**s**pace |
| `rws` | `ros2 launch robot_bringup robot.launch.xml` | **r**un — the whole robot stack |

!!! warning "Build outside Docker will fail"
    `robot/ros_ws/src/common/` is **empty on the host** — it's a bind mount of `common/ros_packages/` that only exists inside the container. Building on the host misses `behavior_tree_msgs`, `autolab_msgs`, etc. Always build inside the container (`autolab connect robot`, then `bws`).

## Namespaces and multiple robots

`robot.launch.xml` pushes everything under a namespace from `$ROBOT_NAME` (e.g. `robot_1`), so the behavior topics are really `/robot_1/behavior/manipulation_command`. The planner topics (`/planning_command`, `/planning_state`) are *not* namespaced — one of several small inconsistencies to keep in mind when echoing topics.

## ROS_DOMAIN_ID and the domain bridge

ROS 2 discovery only connects nodes with the same `ROS_DOMAIN_ID` (an integer, default 0). Autolab runs the robot container on domain **0** and (per `gcs/docker/.env`) the GCS on domain **1**, then runs a **`domain_bridge`** node on both sides to forward a curated list of topics between domains 0↔1:

- robot side: `robot/ros_ws/src/robot_bringup/params/domain_bridge.yaml`
- GCS side: `gcs/ros_ws/src/gcs_bringup/config/domain_bridge.yaml`

If a topic you need isn't crossing between robot and GCS, check those two YAML files first — the bridge only forwards what's listed. (The actual domain-ID wiring has inconsistencies; see [known issue #6](known-issues.md).)

Both containers also set `ROS_LOCALHOST_ONLY=1` and Fast DDS profiles (`fastdds.xml`), which confine DDS traffic to the local machine — inter-machine traffic is deliberately MQTT (UI↔GCS) or WebSocket/HTTP (camera-edge, lab machines) instead of DDS.

## tf2, MoveIt, RViz — the rest of the vocabulary

- **tf2** — the transform tree relating coordinate frames (`world` → `Chassis_1` → … → `end_effector_p4_1`). Inspect with `ros2 run tf2_ros tf2_echo world end_effector_p4_1`.
- **URDF/SRDF** — robot description files. Autolab's live in `robot/ros_ws/src/autonomy/5_planning/base_urdf/robot.urdf` (+ MoveIt config in `base_moveit_config_2/`).
- **MoveIt** — the motion-planning framework: maintains a *planning scene* (robot + octomap obstacles), checks collisions, time-parameterizes trajectories, and sends them to `ros2_control` controllers. Autolab wraps it with its own PRM planner — see [Planning](subsystems/planning.md).
- **RViz** — the 3D visualization tool. Configs: `gcs/ros_ws/src/gcs_bringup/rviz/gcs.rviz`, `robot/ros_ws/src/robot_bringup/rviz/robot.rviz`, `…/base_moveit_config_2/config/moveit.rviz`.
- **rosbag** — topic recording. See `record_behavior_bag.sh` at the repo root.
