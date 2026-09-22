# 1_navigation — swerve navigation for Autolab

Localise in a saved map and drive to goals given over ROS topics (or clicked in RViz).
Runs in the robot container (ROS 2 Humble) and on the host (ROS 2 Jazzy) from the same source.

```
1_navigation/
  swerve_navigation/    ament_python: swerve_bridge, cmd_vel_mux, joy_teleop, cloud_to_scan,
                        point_to_goal, location_markers, phoenix_sim, nav_api
  navigation_bringup/   ament_cmake: launch/, config/, urdf/, rviz/, behavior_trees/, maps/,
                        phoenix6/tuner_constants.py + the custom Smac-2D planner plugin (C++)
  scripts/install_deps.sh   container dependencies
  build.sh              build (container -> ros_ws/install, host -> ros_ws_nav/install)
  run.sh                safety-gated runner (dry run by default)
```

## Quick start (all ROS processes in Docker/Humble)

```bash
cd /home/robot/AutoLab/robot/ros_ws       # inside autolab-robot-l4t-1
./src/autonomy/1_navigation/build.sh
./src/autonomy/1_navigation/run.sh                    # dry run: the robot cannot move
./src/autonomy/1_navigation/run.sh --hardware         # REAL ROBOT
```

In a host terminal, start the UI as a separate **Humble process inside the same container**:

```bash
cd ~/coding/Autolab/robot/ros_ws
./src/autonomy/1_navigation/rviz_docker.sh
```

The window uses the host X display, but its ROS participant is Humble. Closing or restarting it
does not stop navigation. Docker navigation defaults to isolated ROS domain 42 because domain 0 on
this Jetson already contains unrelated Jazzy processes. The stack also defaults to Cyclone DDS:
Fast DDS repeatedly froze only `controller_server`'s TF reader while fresh subscribers continued
to receive current transforms. Set the same `NAVIGATION_DOMAIN_ID` for both commands only when a
different isolated domain is required.

Then, in a second terminal inside the container (`sws` is enough, the packages are in
`ros_ws/install`):

```bash
ros2 topic echo /nav/locations --once
ros2 topic echo /nav/state
ros2 topic pub --once /nav/goal_location std_msgs/String '{data: ot2}'
ros2 topic pub --once /nav/cancel std_msgs/Empty '{}'
```

## Changes to the rest of Autolab

`.env` already pointed at this package, so nothing there changed:

```
NAVIGATION_LAUNCH_PACKAGE="navigation_bringup"
NAVIGATION_LAUNCH_FILE="navigation.launch.xml"
```

`robot/docker/Dockerfile.robot` gained the dependencies the stack needs beyond the image:
`ros-humble-velodyne`, `ros-humble-rmw-cyclonedds-cpp`, and the phoenix6 **Python** package in
`/opt/phoenix6_python` (the image's apt `phoenix6` is the C++ one, which swerve_bridge cannot use).
Until the image is rebuilt, `scripts/install_deps.sh` installs the same things into a running
container.

Nothing else in Autolab was touched.

### The autonomy bringup does not start navigation by default

`autonomy.launch.xml` includes `navigation.launch.xml`, which starts the stack only when
`NAVIGATION_ENABLED=true`. Navigation owns `/cmd_vel`, `/odom` and the `odom -> base_footprint`
transform, and with `hardware:=false` those come from the phoenix6 *simulator*, which would fight
a real drivetrain brought up beside it. Enable it deliberately:

```bash
export NAVIGATION_ENABLED=true     # and NAVIGATION_HARDWARE=true to let it drive
```

`run.sh` starts navigation directly and ignores both variables.

## Humble and Jazzy from one source

Everything distro-specific is selected at launch time by `ROS_DISTRO`:

| | Humble (container) | Jazzy (host) |
|---|---|---|
| Nav2 params | `config/nav2_params.yaml` + `config/nav2_humble_overlay.yaml` | `config/nav2_params.yaml` |
| behaviour tree | `navigate_to_pose_cautious_humble.xml` (BT.CPP v3) | `navigate_to_pose_cautious.xml` (BT.CPP v4) |
| build output | `ros_ws/install` (with every other package) | `ros_ws_nav/install` |

The overlay carries only what Humble spells differently: behaviour plugins are
`nav2_behaviors/Spin` (not `::`), and collision-monitor polygons take a flat number array with
`max_points` instead of a `"[[x, y], ...]"` string with `min_points`. The C++ planner guards the
`cancel_checker` argument of `nav2_core::GlobalPlanner::createPlan()` behind
`NAV2_PLANNER_CANCEL_CHECKER`, which `CMakeLists.txt` defines everywhere except Humble. In Python,
`swerve_bridge` only asks rclpy for message-reception timestamps where they exist (Iron+), and
`location_markers` / `point_to_goal` tolerate Humble's `NavigateToPose` result, which has no
`error_code`. Where Humble cannot tell a failed staging leg from one preempted by another client,
the approach stops instead of driving to the goal on its own.

If you edit the Jazzy tree or the Nav2 parameters, keep the Humble copies in step.

## Topic API

`nav_api` (`api_namespace:=/` by default) is the whole contract.

In:

| topic | type | meaning |
|---|---|---|
| `/nav/goal_pose` | `geometry_msgs/PoseStamped` | go to a pose in `map` |
| `/nav/goal_location` | `std_msgs/String` | go to a saved place by name |
| `/nav/cancel` | `std_msgs/Empty` | cancel the current goal |

Out (latched, so a late subscriber gets the current value, except where noted):

| topic | type | meaning |
|---|---|---|
| `/nav/state` | `std_msgs/String` | `IDLE`/`SENDING`/`NAVIGATING`/`SUCCEEDED`/`CANCELED`/`FAILED` |
| `/nav/busy` | `std_msgs/Bool` | true while a goal is active |
| `/nav/goal` | `geometry_msgs/PoseStamped` | goal currently being pursued |
| `/nav/locations` | `std_msgs/String` | saved places as JSON |
| `/nav/result` | `std_msgs/String` | terminal state of the last goal (an event: not latched) |
| `/nav/pose` | `geometry_msgs/PoseStamped` | AMCL pose in `map` (not latched) |
| `/nav/distance_remaining`, `/nav/time_remaining` | `std_msgs/Float32` | Nav2 feedback (not latched) |

Goals are republished on `/goal_pose_staged`, so a goal from the API takes exactly the same
staged-approach path as an RViz click. State is *derived* from `/navigate_to_pose/_action/status`
and its feedback rather than from an action client of its own, so it reports what the robot is
really doing no matter who commanded it. A 1.2 s settle keeps the gap between the two staged legs
from looking like arrival.

`api_namespace:=/robot_1` moves the whole API under that namespace.

## run.sh

Refuses `joy:=` (the Xbox pad is the e-stop), refuses a second stack on the same domain, and asks
for a typed `YES` before arming the drivetrain. Extra launch arguments pass through, e.g.
`map:=/path/to/other.yaml`, `rviz:=true`, `x:=1.0 y:=0.5 yaw:=1.57`, `api_namespace:=/robot_1`,
`nav_motion:=omni`, `approach_distance:=0`.

In the Humble robot container, RViz defaults to off even though the launch-file default is on;
run `rviz_docker.sh` from the host. On Jazzy, or when `rviz:=true` is explicit, the launch-file
setting is used unchanged.

The default map is `navigation_bringup/maps/map.yaml` with the places in `map.locations.yaml`.
`navigation_bringup/phoenix6/tuner_constants.py` is the Tuner X output for this drivetrain,
copied from `~/nav_nav/Navigation/generated`: re-copy it after re-tuning.

## Safety

The lidar is blind behind and to the right of the robot over ~136°, so the stack drives
nose-first (`nav_motion:=forward`) by default. On hardware the pad is the only e-stop: `B`
latches stop, holding `LT` overrides Nav2, `X` cancels the goal.

Hardware runs confirmed that the bridge publishes live odometry and the full Nav2 stack reaches
the active lifecycle state. The container's Humble RViz was the unstable component, so it is now
kept out of the robot process by default. A dry-run goal can still eventually fail because the
simulated base moves while the real lidar shows a stationary world and the pose estimate walks
into mapped obstacles; that happens on both distros and says nothing about the real robot.
