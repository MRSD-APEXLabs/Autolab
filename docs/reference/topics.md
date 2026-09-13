# Topic Map

Who talks to whom. Topics on the robot side are namespaced under `/robot_1` (from `$ROBOT_NAME`) when launched via `robot.launch.xml`; the planner topics are unnamespaced.

## Mission layer (GCS)

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/routine_executor/start_routine_cmd` | `std_msgs/String` (routine JSON) | `mqtt_ros2_bridge` / `run_routine.py` → `routine_executor_node` |
| `/routine_executor/cancel`, `/pause`, `/resume` | `std_msgs/String` | same (pause/resume only reachable via ROS — see known issue #1) |
| `/routine_executor/status` | `std_msgs/String` (JSON, 1 Hz) | `routine_executor_node` → `ros2_mqtt_bridge` → UI |

## MQTT topics (Mosquitto, ws :9001 / tcp :1883)

| MQTT topic | Bridged to |
|---|---|
| `cmd/routine_executor/start_routine_cmd` | → `/routine_executor/start_routine_cmd` |
| `cmd/routine_executor/cancel` | → `/routine_executor/cancel` |
| `cmd/routine_executor/pause`, `…/resume` | ⚠ **dropped** (not in `CMD_TOPIC_MAP`) |
| `ros2/<any ros topic>` | ← auto-discovered ROS topics, `{"data": "<json>"}` envelope |

## Behavior layer

| Topic | Type | Producer → Consumer |
|---|---|---|
| `behavior/manipulation_command` | `behavior_tree_msgs/ManipulationCommand` | `routine_executor` / CLI → `manipulation_executive` |
| `behavior/lab_machine_command` | `behavior_tree_msgs/LabMachineCommand` | `routine_executor` / CLI → `lab_machine_executive` |
| `behavior/<action>_active` | `behavior_tree_msgs/Active` | behavior tree → executives |
| `behavior/<action>_status` | `behavior_tree_msgs/Status` (BEST_EFFORT, depth 1) | executives → behavior tree, `routine_executor` |
| `behavior/<condition>_success` | `std_msgs/Bool` | executives → behavior tree |
| `behavior/manipulation_phase` | `std_msgs/String` | `manipulation_executive` (debug) |

Action names follow the tree labels: `pick_base_object`, `pick_up_object`, `place_object`, `pick_wellplate` (⚠ `step_config.py` wrongly watches `pick_object_status` — #22), `execute_ot2_protocol`, `execute_shaker_protocol`.

## Planning layer

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/planning_command` | `std_msgs/String` (`plan_april_<id>` \| `plan_wellplate` \| `plan_home` \| `plan_home_offset` \| `idle`) | `manipulation_executive` / manual → `move_to_pose_node` |
| `/planning_state` | `std_msgs/String` (`IDLE\|PLANNING\|EXECUTING\|SUCCESS\|ERROR`) | `move_to_pose_node` → `manipulation_executive` |
| `/task_planner/task_type` | `std_msgs/String` (`Move`/`Grasp`/`Place`) | ⚠ **nothing publishes this** → `move_to_pose_node` (known issue #24) |
| `/display_planned_path` | `moveit_msgs/DisplayTrajectory` | `move_to_pose_node` → RViz |
| `/target_pose_marker`, `/target_points_marker` | `MarkerArray` / `Marker` | `move_to_pose_node` → RViz |

Services used by the planner: `/controller_manager/list_controllers`, `/controller_manager/switch_controller`, `/xarm/clean_error`, `/xarm/motion_enable`, `/xarm/set_state`.

## Perception

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Velodyne driver → bridge scripts |
| `/zed_pointcloud` | `PointCloud2` | bridge scripts → MoveIt octomap (`sensors_3d.yaml`), `move_to_pose_node` |
| `/inspect/apriltags`, `/inspect/wellplates` | `visualization_msgs/MarkerArray` (BEST_EFFORT) | bridge scripts → `move_to_pose_node` |
| `/inspect/image`, `/inspect/annotated_image` | `sensor_msgs/Image` | bridge scripts → RViz |
| `/servo/image`, `/servo/annotated_image`, `/servo/detections`, `/servo/target` | Image / MarkerArray / Marker | bridge scripts → RViz |
| `/vlp16_accumulated`, `/merged_pointcloud` | `PointCloud2` | bridge scripts → RViz |
| `/obstacles/markers` | `MarkerArray` | bridge scripts → RViz |

## Base / misc

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/robot_1/swerve/goal_pose` | `geometry_msgs/Pose2D` | (manual) → `navigation_controller` |
| `/robot_1/joints/joint_command` | `sensor_msgs/JointState` | `navigation_controller` → (no consumer wired) |
| `/joint_states` | `sensor_msgs/JointState` | driver → planner, recorders |

## Cross-domain forwarding

Only topics listed in the two `domain_bridge` YAMLs cross between ROS domain 0 (robot) and 1 (GCS): `gcs/ros_ws/src/gcs_bringup/config/domain_bridge.yaml` and `robot/ros_ws/src/robot_bringup/params/domain_bridge.yaml`. A topic missing from your `ros2 topic list` on one side is usually missing from these files.
