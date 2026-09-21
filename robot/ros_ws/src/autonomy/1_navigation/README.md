# 1_navigation — swerve navigation for Autolab

Localise in a saved map and drive to goals given over ROS topics (or clicked in RViz).
This is the `5_navigation` stack from `~/nav_nav` merged into the Autolab tree, renamed to the
package names Autolab's `.env` already expects.

```
1_navigation/
  swerve_navigation/    ament_python: swerve_bridge, cmd_vel_mux, joy_teleop, cloud_to_scan,
                        point_to_goal, location_markers, phoenix_sim, nav_api
  navigation_bringup/   ament_cmake: launch/, config/, urdf/, rviz/, behavior_trees/, maps/
                        + the custom Smac-2D planner plugin (C++)
  build.sh              host build  -> robot/ros_ws_nav/
  run.sh                safety-gated runner (dry run by default)
  scripts/canivore_up.sh
```

## Changes to the rest of Autolab: none

`.env` already says

```
NAVIGATION_LAUNCH_PACKAGE="navigation_bringup"
NAVIGATION_LAUNCH_FILE="navigation.launch.xml"
```

so the package is named `navigation_bringup` and ships `launch/navigation.launch.xml`.
No existing Autolab file was edited.

`navigation.launch.xml` is a thin wrapper over `navigation.launch.py` and defaults to
`hardware:=false`, `rviz:=false`. Inside the robot container it finds none of the packages it
needs (see below), logs `[navigation] NOT STARTED: ...` and starts nothing — `autonomy.launch.xml`
comes up normally either way.

## Host, not container

The container is ROS 2 **Humble** / Python 3.10; this stack is ROS 2 **Jazzy** / Python 3.12.
The container also has no `velodyne_driver`, no `joy`, and only the apt **C++** phoenix6 — the
swerve bridge needs the Python phoenix6 wheel from the Jazzy venv. So navigation runs on the
host and reaches the rest of Autolab over DDS like any other participant.

Two portability concessions were made so the code still *builds* under Humble if that ever
becomes useful: `latest_frame_smac_planner.cpp` guards the `cancel_checker` argument of
`nav2_core::GlobalPlanner::createPlan()` behind `NAV2_PLANNER_CANCEL_CHECKER`, which
`CMakeLists.txt` defines for every distro except Humble.

Builds go to `robot/ros_ws_nav/`, never `robot/ros_ws/{build,install}` — those are bind-mounted
into the container and `bws` writes into them.

## Topic API

`nav_api` (started by the launch file, `api_namespace:=/` by default) is the whole contract.

In:

| topic | type | meaning |
|---|---|---|
| `/nav/goal_pose` | `geometry_msgs/PoseStamped` | go to a pose in `map` |
| `/nav/goal_location` | `std_msgs/String` | go to a saved place by name |
| `/nav/cancel` | `std_msgs/Empty` | cancel the current goal |

Out (all latched except pose/feedback):

| topic | type | meaning |
|---|---|---|
| `/nav/state` | `std_msgs/String` | `IDLE`/`SENDING`/`NAVIGATING`/`SUCCEEDED`/`CANCELED`/`FAILED` |
| `/nav/busy` | `std_msgs/Bool` | true while a goal is active |
| `/nav/result` | `std_msgs/String` | terminal state of the last goal |
| `/nav/goal` | `geometry_msgs/PoseStamped` | goal currently being pursued |
| `/nav/pose` | `geometry_msgs/PoseStamped` | AMCL pose in `map` |
| `/nav/distance_remaining`, `/nav/time_remaining` | `std_msgs/Float32` | Nav2 feedback, forwarded as-is |
| `/nav/locations` | `std_msgs/String` | saved places as JSON |

Goals are republished on `/goal_pose_staged`, so a goal from the API takes exactly the same
staged-approach path as an RViz click. State is *derived* from
`/navigate_to_pose/_action/status` + feedback rather than from an action client of its own, so
it reports what the robot is really doing no matter who commanded it (RViz, a saved place, the
API). A 1.2 s settle keeps the gap between the two staged legs from looking like arrival.

`api_namespace:=/robot_1` moves the whole API under that namespace for multi-robot integration.

## Build and run

```bash
autonomy/1_navigation/build.sh                  # host, Jazzy -> robot/ros_ws_nav
autonomy/1_navigation/run.sh                    # dry run: phoenix6 simulator, cannot move
autonomy/1_navigation/run.sh --hardware         # REAL ROBOT, drives by itself
```

`run.sh` refuses `joy:=` (the Xbox pad is the e-stop), refuses a second stack on the same
domain, and asks for a typed `YES` before arming the drivetrain. Extra launch arguments pass
through, e.g. `map:=/path/to/other.yaml`, `rviz:=true`, `x:=1.0 y:=0.5 yaw:=1.57`,
`api_namespace:=/robot_1`, `nav_motion:=omni`, `approach_distance:=0`.

The default map is `navigation_bringup/maps/map.yaml` with places in `map.locations.yaml`.

## Safety

The lidar is blind behind and to the right of the robot over ~136°, so the stack drives
nose-first (`nav_motion:=forward`) by default. On hardware the pad is the only e-stop: `B`
latches stop, holding `LT` overrides Nav2, `X` cancels the goal.
