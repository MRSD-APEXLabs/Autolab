# Launch visual servoing — from scratch

The grasp runs on the Xavier (`~/Autolab-Camera-Edge/visual_servo.py`) and drives the arm directly over the xArm SDK. Before starting:

- gripper mounted and its RS-485 cable connected
- on the Xavier, `~/Autolab-Camera-Edge/config.py` has `DEBUG = False` (with `True` nothing moves)
- nothing else executing on the arm (the planning stack may be up, but idle)
- a well plate under the wrist camera, e-stop in reach

## 1–2. Network + camera server

Same as [launch-planning.md](launch-planning.md) steps 1–2.

## 3. Watch the wrist camera (optional)

```bash
ssh autolab@192.168.1.101 'cd ~/Autolab-Camera-Edge/example_client && python3 -m http.server 8080'
# Firefox → http://192.168.1.101:8080/test.html → host 192.168.1.101 → Connect → Wrist tile
```

## 4a. Trigger directly (Jetson)

```bash
/usr/bin/python3 ~/coding/percep/test_server.py ws://192.168.1.101:8765
> servo        # arm → start pose, gripper opens, servos down onto the plate, closes on FSR contact, lifts
> status       # worker_alive=false when finished
> stop         # abort at any time
> idle
```

## 4b. Trigger through the pipeline (Jetson)

Planning stack up ([launch-planning.md](launch-planning.md) step 5), then the executives:

```bash
docker exec -it autolab-robot-l4t-1 bash -lc 'export ROS_LOCALHOST_ONLY=1 FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/AutoLab/common/ros_packages/fastdds_loopback.xml; ros2 launch behavior_bringup behavior.launch.xml'
```

`behavior_tree_msgs` exists only inside the container, so these two run there:

```bash
docker exec -it autolab-robot-l4t-1 bash -lc "export ROS_LOCALHOST_ONLY=1 FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/AutoLab/common/ros_packages/fastdds_loopback.xml; ros2 topic echo /robot_1/behavior/manipulation_phase"
docker exec autolab-robot-l4t-1 bash -lc "export ROS_LOCALHOST_ONLY=1 FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/AutoLab/common/ros_packages/fastdds_loopback.xml; ros2 topic pub --once /robot_1/behavior/manipulation_command behavior_tree_msgs/msg/ManipulationCommand '{type: pick_base, object_type: well_plate, target_machine: \"\"}'"
# sequence: inspect → plan above the plate → servo grasp → plan_home_offset
# SUCCESS is reported when the Xavier's worker goes idle, whether or not a plate was grasped
```

## 5. Afterwards

Send `plan_home_offset` ([launch-planning.md](launch-planning.md) step 6). If the next plan won't execute, restart the planning launch.
