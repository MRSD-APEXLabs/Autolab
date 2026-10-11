# Launch visual servoing — from scratch

The grasp runs in the robot stack: `visual_servo_node` (`robot/ros_ws/src/autonomy/7_manipulation/visual_servoing`)
servos on the wrist ZED X Nano and drives the arm directly over the xArm SDK. Before starting:

- gripper mounted and its RS-485 cable connected
- nothing else executing on the arm (the planning stack may be up, but idle)
- a well plate under the wrist camera, e-stop in reach

## 1–3. Network, camera hub, perception

Same as [launch-planning.md](launch-planning.md) steps 1–3. The camera stack must be up: the servo reads
`/zedx_nano/*` and switches the hub with `/camera_hub/select_zedx_nano`.

## 4. Visual servo node

The robot bringup (`autonomy_bringup` → `manipulation_bringup`, real robot only) already starts it with
`execute:=true`; skip this step then (a second copy shares the node name and services). By hand, the default is a
dry run (arm only read):

```bash
ros2 launch visual_servoing visual_servo.launch.xml                          # dry run
ros2 launch visual_servoing visual_servo.launch.xml execute:=true grasp:=false   # moves, stops at the hover pose
ros2 launch visual_servoing visual_servo.launch.xml execute:=true            # full grasp
```

Watch it with `ros2 topic echo /visual_servo/status` or the `/visual_servo/annotated_image` stream.

## 5a. Trigger directly

```bash
ros2 service call /visual_servo/start std_srvs/srv/Trigger   # one run: servo to hover, descend, close, lift
ros2 service call /visual_servo/stop std_srvs/srv/Trigger    # abort at any time
```

## 5b. Trigger through the pipeline (Jetson)

Planning stack up ([launch-planning.md](launch-planning.md) step 4), then the executives:

```bash
docker exec -it autolab-robot-l4t-1 bash -lc 'export ROS_LOCALHOST_ONLY=1 FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/AutoLab/common/ros_packages/fastdds_loopback.xml; ros2 launch behavior_bringup behavior.launch.xml'
```

`behavior_tree_msgs` exists only inside the container, so these two run there:

```bash
docker exec -it autolab-robot-l4t-1 bash -lc "export ROS_LOCALHOST_ONLY=1 FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/AutoLab/common/ros_packages/fastdds_loopback.xml; ros2 topic echo /robot_1/behavior/manipulation_phase"
docker exec autolab-robot-l4t-1 bash -lc "export ROS_LOCALHOST_ONLY=1 FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/AutoLab/common/ros_packages/fastdds_loopback.xml; ros2 topic pub --once /robot_1/behavior/manipulation_command behavior_tree_msgs/msg/ManipulationCommand '{type: pick_base, object_type: well_plate, target_machine: \"\"}'"
# pick_base: PLANNING (plan_home_offset) → SERVOING → SUCCESS
# pick_up / place / pick_wellplate: SELECTING_CAMERA (ZED X) → PLANNING → … → PLANNING_SAFE
```

The servo step succeeds only when `/visual_servo/status` ends with a result in the executive's
`servo_success_results` (`grasped`, `aligned`, and `stopped` for a dry run). The executive and the servo node
must share a ROS domain; with `ROS_LOCALHOST_ONLY=1` that means the same machine.

Which AprilTag each lab machine uses (`place` / `pick_wellplate` → `plan_april_<id>`) is set in
`robot/ros_ws/src/autonomy/6_behavior/manipulation_executive/config/manipulation_executive.yaml` (`april_tags`).

## 6. Afterwards

Send `plan_home_offset` ([launch-planning.md](launch-planning.md) step 5). If the next plan won't execute, restart the planning launch.
