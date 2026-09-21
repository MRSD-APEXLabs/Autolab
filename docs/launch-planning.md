# Launch planning — from scratch

The arm moves from step 5. E-stop in reach, workspace clear, nothing else commanding the arm (no joystick script, no UFactory Studio).

## 1. Network (Jetson)

```bash
ip -4 -br addr | grep enP2p1s0            # must be UP, 192.168.1.100
ping -c1 192.168.1.236 && echo ARM OK
ping -c1 192.168.1.101 && echo XAVIER OK
```

## 2. Camera server (Xavier)

Check first — if it's already running, **skip this step** (a second `main.py` fails: ports and cameras already taken):

```bash
timeout 2 bash -c '</dev/tcp/192.168.1.101/8765' && echo "ALREADY RUNNING — skip" || echo "not running — start it"
```

```bash
ssh autolab@192.168.1.101
tmux new -s edged
cd ~/Autolab-Camera-Edge && python3 main.py     # wait for the three "ws://0.0.0.0:876x" lines
# Ctrl-b d  then  exit
```
To see the camera feed go to http://192.168.1.101:8080/test.html in the browser and type 192.168.1.101 as the host


## 3. Put the camera in Inspect (Jetson)

```bash
/usr/bin/python3 ~/coding/percep/test_server.py ws://192.168.1.101:8765
> inspect
> status                                        # mode=inspect, worker_alive=true
# Ctrl-C
```

## 4. Point cloud + detections into ROS (Jetson, repo root, no venv active)

```bash
cd ~/coding/Autolab && /usr/bin/python3 pc3.py   # "Connected to ws://192.168.1.101:8766"
# other terminal:  ros2 topic hz /zed_pointcloud   (~5 Hz)
```

## 5. Robot stack (Jetson → container) — the arm enables here

```bash
xhost +local:
docker start autolab-robot-l4t-1                                          # skip if already running
docker exec autolab-robot-l4t-1 tmux kill-session -t robot_bringup        # stop the auto full-stack launch ("no server running" is fine)
docker exec -it autolab-robot-l4t-1 bash -lc 'export DISPLAY=:1 ROS_LOCALHOST_ONLY=1 FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/AutoLab/common/ros_packages/fastdds_loopback.xml; ros2 launch planning_bringup planning.launch.xml robot_ip:=192.168.1.236 show_rviz:=true'
# ready when "Received command: idle" repeats (~30 s); RViz opens with the planner path display
```

Always `bash -lc` — without it the stack comes up on the wrong ROS domain and ignores every command.

## 6. Plan (Jetson)

```bash
# terminal A
ros2 topic echo /planning_state
```

```bash
# terminal B — one at a time, wait for PLANNING → EXECUTING → SUCCESS
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_home_offset'}"   # fixed safe pose, no camera needed — do this first
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_home'}"          # home point (via top_camera TF)
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_april_1'}"       # 25 cm above AprilTag 1 (OT-2)
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_april_2'}"       # 25 cm above AprilTag 2 (shaker)
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_wellplate'}"     # 25 cm above the detected well plate
ros2 topic pub --once /planning_command std_msgs/String "{data: 'idle'}"
```

- Wrist must be within ~11° of straight down before the first plan, else `Waypoint 0 FAILED ee_down constraint`.
- RViz shows two lines per plan: orange = raw planner path, cyan = after shortcutting.
- Planner choice: `PLANNER` in `robot/ros_ws/src/autonomy/5_planning/global_planner/src/move_to_pose_node.cpp` (`RRT_CONNECT` | `INFORMED_RRTSTAR` | `PRM`), then rebuild and relaunch step 5:

```bash
docker exec autolab-robot-l4t-1 bash -lc 'cd ~/AutoLab/robot/ros_ws && colcon build --symlink-install --packages-select global_planner'
```

## 7. Stop

Ctrl-C the launch (step 5) · Ctrl-C `pc3.py` · `test_server.py` → `idle`
