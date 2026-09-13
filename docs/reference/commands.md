# Command Cheatsheet

Every run/launch command in the project, grouped by layer. Commands marked ⚠ move the real arm. For the ordered, copy-paste version of the planning bringup see the [Planning Quickstart](../planning-quickstart.md).

## Xavier (camera-edge)

```bash
ssh autolab@192.168.1.101
cd ~/Autolab-Camera-Edge && python3 main.py                                   # server (system python, in tmux)
cd ~/Autolab-Camera-Edge/example_client && python3 -m http.server 8080        # dashboard → http://192.168.1.101:8080/test.html
```

## `autolab` CLI (host)

```bash
./autolab setup                # once: install the shell function
autolab install                # once: install docker etc.
autolab up                     # start everything in the root compose (currently: robot only)
autolab up robot               # robot container
autolab up --build             # rebuild images first
autolab connect robot          # shell into a container
autolab status                 # container status
autolab logs <container>       # tail logs
autolab down                   # stop all (all profiles)
```

## Docker compose (stacks not in the root compose)

Run from the repo root. **Docs and Isaac Sim** can run standalone but need `--env-file .env` (compose only auto-loads the `.env` beside the compose file, and these files take `PROJECT_NAME`/`DOCKER_IMAGE_TAG`/… from the root `.env`). **The GCS/UI stack cannot run standalone at all** — it uses the `autolab_network` defined only in the root compose — so un-comment its `include:` line in the root `docker-compose.yaml` and use `autolab up <service>` (see [Setup](../setup.md#the-autolab-cli)).

```bash
# one-time: in the ROOT docker-compose.yaml, un-comment the include line
#     # - gcs/docker/docker-compose.yaml
# (gcs/docker/docker-compose.yaml can't run standalone with -f: it uses the
#  autolab_network that only the root compose defines)
autolab up gcs-real                        # GCS + Mosquitto (host-networked; plain `gcs` can't reach the robot — #25)
COMPOSE_PROFILES=ui-dev autolab up ui-dev # UI dev :3000
autolab up ui                      # UI prod :80
autolab up elasticsearch kibana filebeat
docker compose --env-file .env -f simulation/isaac-sim/docker/docker-compose.yaml up isaac-sim-gui
docker compose --env-file .env -f docs/docker/docker-compose.yaml up docs                   # this site :8000
```

## Build & source (inside containers)

```bash
docker start autolab-robot-1 && autolab connect robot && tmux kill-session -t robot_bringup   # start existing container idle, no rebuild
bws        # colcon build --symlink-install
sws        # source install/local_setup.bash
cws        # clean workspace (prompts)
rws        # ros2 launch robot_bringup robot.launch.xml
# planning workspace, per README_planning.md:
colcon build --packages-ignore robot_bringup rviz_behavior_tree_panel
source install/setup.bash
```

## Whole-stack bringup (robot container)

```bash
ros2 launch robot_bringup robot.launch.xml                # ⚠ sim:=true default; see known issue #2
ros2 launch robot_bringup robot.launch.xml sim:=false     # ⚠ real robot
ros2 launch autonomy_bringup autonomy.launch.xml          # autonomy stages only
```

## Per-stage launches

```bash
ros2 launch interface_bringup interface.launch.xml                        # stub
ros2 launch navigation_bringup navigation.launch.xml
ros2 launch mapping_and_localization_bringup mapping_and_localization.launch.xml  # stub
ros2 launch perception_bringup perception.launch.xml                      # stub
ros2 launch local_bringup local.launch.xml                                # stub
ros2 launch planning_bringup planning.launch.xml robot_ip:=192.168.1.236  # ⚠ MoveIt + driver
ros2 launch behavior_bringup behavior.launch.xml
ros2 launch manipulation_executive manipulation_executive.launch.xml
ros2 launch lab_machine_executive lab_machine_executive.launch.xml
ros2 launch logging_bringup logging.launch.xml record_bag:=true
```

## Planning / arm

```bash
# move_to_pose_node is auto-started by planning.launch.xml (15 s later) — do not start it again (#26);
# `ros2 run` is absent in the robot image (#31); binary: install/global_planner/lib/global_planner/move_to_pose_node
ros2 launch base_moveit_config_2 demo.launch.py                           # fake hardware demo
ros2 launch base_moveit_config_2 move_group.launch.py
ros2 launch base_moveit_config_2 moveit_rviz.launch.py
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.236 dof:=6 robot_type:=xarm hw_ns:=xarm no_gui_ctrl:=false   # ⚠
ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py
ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.236       # ⚠ driver only
ros2 launch xarm_planner xarm6_planner_realmove.launch.py robot_ip:=192.168.1.236   # ⚠
ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.236  # ⚠
```

## GCS

```bash
ros2 launch gcs_bringup gcs.launch.xml            # RViz + rqt console + domain bridge
ros2 run routine_executor routine_executor_node   # ← manual, no launch file
ros2 run gcs_monitoring mqtt_ros2_bridge          # ← manual
ros2 run gcs_monitoring ros2_mqtt_bridge          # ← manual
python3 gcs/ros_ws/src/routine_executor/scripts/run_routine.py \
    pick_base "place:target_machine=ot2" ot2 pick_up "shaker:pwm=200"     # ⚠ full routine
rqt --force-discover                              # find the custom rqt panels
```

## Sensors

```bash
cd ~/coding/vlp16 && source install/setup.bash && ros2 launch vlp16_rviz vlp16_rviz.launch.py start_rviz:=false   # VLP-16 → /velodyne_points (host, Jazzy)
python3 ~/coding/percep/test_server.py             # Xavier mode switcher: servo | inspect | idle | status | stop
ros2 launch zed_wrapper zed_dual_camera.launch.py \
    pose_cam_serial:='41591402' wire_cam_serial:='44405253' \
    camera_name:="robot_1/sensors" node_name:="front_stereo"
```

## Host-side scripts (⚠ machine-local, not in git)

```bash
cd ~/coding/Autolab
python3 pointcloud_lidar_accumulated.py            # perception bridge (accumulating)
python3 pointcloud_lidar.py                        # perception bridge (latest-frame)
python3 pc4.py                                     # camera-only bridge
/home/labx/coding/Autolab/.venv_jog/bin/python joy_cartesian_jog.py   # ⚠⚠ teleop — see Teleop page
```

## Waypoint / roadmap tools

```bash
python3 robot/ros_ws/src/autonomy/5_planning/global_planner/src/record_waypoints.py       # ⚠ teach mode
python3 robot/ros_ws/src/autonomy/5_planning/global_planner/src/record_waypoints_ros2.py
python3 robot/ros_ws/src/autonomy/5_planning/global_planner/src/inject_waypoints_node.py
python3 robot/add_waypoints_to_roadmap.py          # broken — known issue #4
python3 robot/ros_ws/src/autonomy/5_planning/global_planner/src/visualize_roadmap.py      # host paths
```

## Tests & debugging

```bash
./test_full_routine.sh                             # ⚠ OT-2-only smoke test (steps 4–5 missing — #23)
bash record_behavior_bag.sh robot_1                # rosbag of behavior topics
ros2 topic echo /planning_state
ros2 topic echo /behavior/manipulation_phase
ros2 topic echo /routine_executor/status
ros2 run tf2_ros tf2_echo world end_effector_p4_1
ros2 control list_controllers
mosquitto_sub -t 'ros2/#' -v                       # watch the MQTT side
```

## Manual command injection (from `test commands.txt` / `test_full_routine.sh`)

```bash
ros2 topic pub --once /behavior/manipulation_command behavior_tree_msgs/msg/ManipulationCommand \
  "{'type': 'pick_base', 'object_type': 'well_plate', 'target_machine': ''}"          # ⚠
ros2 topic pub --once /behavior/manipulation_command behavior_tree_msgs/msg/ManipulationCommand \
  "{'type': 'place', 'object_type': 'well_plate', 'target_machine': 'ot2'}"           # ⚠
ros2 topic pub --once /behavior/lab_machine_command behavior_tree_msgs/msg/LabMachineCommand \
  "{device: 'shaker', action: 'protocol', parameters_json: '{\"pwm\": 100}'}"
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_wellplate'}"    # ⚠
```
