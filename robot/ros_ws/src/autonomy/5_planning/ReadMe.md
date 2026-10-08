# Planning: how to run it

The arm moves from step 4. Keep the E-stop in reach and the workspace clear. Nothing else may command the arm (no
joystick script, no UFactory Studio).

The point cloud, AprilTags and well plate detections come from the perception stack (`3_perception`). The planner
reads the topics it publishes: `/zed_pointcloud`, `/inspect/apriltags` and `/inspect/wellplates`.

## 1. Network (Thor)

```bash
ip -4 -br addr | grep enP2p1s0            # must be UP, 192.168.1.100
ping -c1 192.168.1.236 && echo ARM OK
ping -c1 192.168.1.101 && echo XAVIER OK
```

## 2. Camera hub (Xavier)

The hub runs at boot (`camera-hub.service`). Check that it answers:

```bash
curl -s http://192.168.1.101:8090/status   # "server": "camera_hub", "state": "streaming"
```

If it doesn't answer, restart it on the Xavier:

```bash
ssh autolab@192.168.1.101 'sudo systemctl restart camera-hub.service'
```

## 3. Perception (Thor host, not the container)

Use a new terminal. `~/.bashrc` already sets up ROS Jazzy, `ROS_DOMAIN_ID=1` and the loopback FastDDS profile.
The packages are built in `~/ros2_ws` from `~/apple_server_manip/2_manipulation`.

```bash
source ~/ros2_ws/install/setup.bash
source /home/labx/apple_server_manip/.venv/bin/activate        # torch for the depth nodes
ros2 launch camera_perception perception.launch.xml with_cameras:=true \
  depth_models_dir:=/home/labx/apple_server_manip/data/models
```

The hub streams one camera at a time, and the point cloud only comes from the ZED X. Select it (takes ~6 s):

```bash
ros2 service call /camera_hub/select_zedx std_srvs/srv/Trigger
```

Check:

```bash
ros2 topic hz /zed_pointcloud                  # up to 5 Hz
ros2 topic echo --once /inspect/wellplates     # a "wellplates" marker when a plate is in view
```

## 4. Planning stack (Thor → container): the arm enables here

After a `git pull`, rebuild in the container first: `autolab connect robot`, then `bws`.

```bash
xhost +local:
docker start autolab-robot-l4t-1                                          # skip if already running
docker exec autolab-robot-l4t-1 tmux kill-session -t robot_bringup        # stop the auto full-stack launch ("no server running" is fine)
docker exec -it autolab-robot-l4t-1 bash -lc 'export DISPLAY=:1 ROS_LOCALHOST_ONLY=1 FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/AutoLab/common/ros_packages/fastdds_loopback.xml; ros2 launch planning_bringup planning.launch.xml robot_ip:=192.168.1.236 show_rviz:=true'
# ready when "Received command: idle" repeats (~30 s); RViz opens with the planner path display
```

Always use `bash -lc`. Without it, the stack comes up on the wrong ROS domain and ignores every command.

## 5. Plan (Thor)

```bash
# terminal A
ros2 topic echo /planning_state
```

```bash
# terminal B: one at a time, wait for PLANNING → EXECUTING → SUCCESS
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_home_offset'}"   # fixed safe pose, no camera needed. Do this first
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_home'}"          # home point (via top_camera TF)
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_april_1'}"       # 25 cm above AprilTag 1 (OT-2)
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_april_2'}"       # 25 cm above AprilTag 2 (shaker)
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_wellplate'}"     # 25 cm above the detected well plate
ros2 topic pub --once /planning_command std_msgs/String "{data: 'idle'}"
```

- **The obstacle cloud is frozen at the first `plan_*` command.** The planner forwards the latest `/zed_pointcloud`
  once to `/zed_pointcloud_frozen`, which is the only cloud MoveIt reads. Every later plan uses that cloud. To take a
  new one, restart the planning launch (step 4).
- `plan_april_<id>` and `plan_wellplate` wait up to 10 s for a detection less than 1 s old, or end in `ERROR`.
- The wrist must be within ~11° of straight down before the first plan, or you get
  `Waypoint 0 FAILED ee_down constraint`.
- RViz shows two lines per plan: orange is the raw planner path, cyan is the path after shortcutting.
- The planner is chosen by `PLANNER` in `global_planner/src/move_to_pose_node.cpp` (`RRT_CONNECT` |
  `INFORMED_RRTSTAR` | `PRM`). After changing it, rebuild and relaunch step 4:

```bash
docker exec autolab-robot-l4t-1 bash -lc 'cd ~/AutoLab/robot/ros_ws && colcon build --symlink-install --packages-select global_planner'
```

## 6. Stop

Ctrl-C the planning launch (step 4), then Ctrl-C the perception launch (step 3).

## Code

The main logic is in `global_planner/src/move_to_pose_node.cpp`. The commands on `/planning_command`
(`std_msgs/String`) are `plan_april_<id>`, `plan_wellplate`, `plan_home`, `plan_home_offset` and `idle`.

Future improvements:
- [Design] Put all config params in one file.
- [Design] `move_to_pose_node.cpp` is ~1000 lines; split it into smaller pieces.
