# Simulation (Isaac Sim)

**Purpose:** an NVIDIA Isaac Sim scene of the robot for development without hardware. **Status: partially wired** — the scene and container exist, but the sim include is commented out of the root compose, and the ros2_control sim plumbing is commented out of the MoveIt config.

Paths: `simulation/isaac-sim/`.

## Running

```bash
# The root docker-compose include for isaac-sim is commented out, so start it directly:
docker compose --env-file .env -f simulation/isaac-sim/docker/docker-compose.yaml up isaac-sim-gui
```

The container starts a tmux session `isaac` and loads either a USD scene or a standalone Python script, controlled from `.env`:

| `.env` variable | Value | Meaning |
|---|---|---|
| `ISAAC_SIM_GUI` | `/root/Autolab/simulation/isaac-sim/working2.usd` | Scene to open |
| `ISAAC_SIM_USE_STANDALONE_SCRIPT` | `false` | Use a script from `launch_scripts/` instead of a USD file |
| `ISAAC_SIM_SCRIPT_NAME` | `robot_init.py` | Which script |
| `PLAY_SIM_ON_START` | `false` | Auto-press play |

A ZED streamer port (30000) is exposed for `zed-isaac-sim` (git submodule) integration. Robot USD variants live in `robot/ros_ws/src/autonomy/5_planning/base_urdf/robot/` and `robot_isaac/`.

## Connecting ROS 2 control to the sim

From `README_planning.md`, the Isaac integration flow:

```bash
sudo apt update
sudo apt install ros-humble-topic-based-ros2-control

ros2 run controller_manager ros2_control_node --ros-args -p use_sim_time:=True
ros2 control list_controllers
ros2 run tf2_ros tf2_echo world end_effector_p4_1
```

The `base_moveit_config_2/config/ros2_controllers.yaml` contains a **commented-out** `topic_based_ros2_control` block that maps controller I/O to `/isaac/joint_commands` and `/isaac/joint_states` — swap that in (and the matching block in `demo_bot.urdf.xacro`) to drive the simulated robot instead of the real `UFRobotSystemHardware` interface.

## Sim mode of the robot container

Independently of Isaac, `robot.launch.xml` defaults to `sim:=true`, which starts `stereo_image_proc` + RViz + rqt instead of real-hardware pieces (see [Robot Bringup](robot-bringup.md)). The `sitl` compose profile is the x86 development path.
