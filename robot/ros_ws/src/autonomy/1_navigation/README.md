# 1_navigation — swerve navigation for Autolab

Localise in a saved map and drive to goals given over ROS topics (or clicked in RViz).
Runs in the robot container (ROS 2 Humble) and on the host (ROS 2 Jazzy) from the same source.

```
1_navigation/
  swerve_navigation/    ament_python: swerve_bridge, cmd_vel_mux, joy_teleop, cloud_to_scan,
                        point_to_goal, location_markers, nav_api
  navigation_bringup/   ament_cmake: launch/, config/, urdf/, rviz/, behavior_trees/, maps/,
                        phoenix6/tuner_constants.py + the custom Smac-2D planner plugin (C++)
  scripts/install_deps.sh   container dependencies
  build.sh              build (container -> ros_ws/install, host -> ros_ws_nav/install)
  run.sh                safety-gated runner (dry run by default)
```

## Current defaults

Navigation now defaults to **best**, **omni** and **fast**. `best.yaml`, `best.pgm`,
`best.locations.yaml` and the SLAM archive were copied from the working reference.
Saved stations are `OT-2`, `shaker`, and `home`; legacy `ot2` API requests also match
`OT-2`. The previous `map.*` files remain available with an explicit map argument.

Fast transit is capped at 0.90 m/s forward, 0.40 m/s lateral and 1.20 rad/s for open
staging alignment. `nav_speed:=normal` restores 0.45 / 0.20 / 0.60. Gentle acceleration,
onboard azimuth profiling, fitted drift correction and slow translation-only station
parking/withdrawal are retained. Large heading changes happen in checked open staging
space. `/nav/state` reports `SUCCEEDED` only after whole-mission verification, never
when the child staging action completes.

All port changes stay in this `1_navigation` directory. The outer autonomy launcher,
behavior executives, Dockerfiles and `.env` do not need edits. Rebuild the two packages
inside the Humble container with `build.sh`, then restart navigation. Check the initial
pose against the new map before sending a goal.

Validation (2026-11-08): both packages compiled in an isolated ARM64 Humble
container. 591 port regression checks passed under Humble/Python 3.10 and under
Jazzy/Python 3.12. The actual Humble Nav2 stack completed OT-2 -> shaker -> home ->
OT-2 -> undock through the public `/nav` API with zero simulated collisions and
no staging-as-arrival reports. A separate undock test verified the final public
`SUCCEEDED` state. The working reference passed its 817 checks. These tests used
a kinematic base and simulated lidar; the physical Autolab stack was not started.

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
this Jetson already contains unrelated Jazzy processes. It uses Fast DDS with shared memory off;
both the stack and Docker RViz communicate over UDP. Set the same `NAVIGATION_DOMAIN_ID` for both
commands only when a different isolated domain is required.

The Humble apt repository on this robot supplies an affected tf2 release whose asynchronous
transform callbacks can deadlock both RViz and Nav2 after they have been running for a while.
`run.sh` and `rviz_docker.sh` detect affected versions, build the upstream 0.25.24 fix once under
`ros_ws/tf2_fix`, and load the same overlay into every navigation process. The first launch may
therefore need network access and take a minute; subsequent launches reuse the overlay. With the
deadlock fixed at its source, the controller path stays identical to `~/nav_nav`: the behavior tree
sends `FollowPath` directly to Nav2's stock MPPI controller.

Then, in a second terminal inside the container (`sws` is enough, the packages are in
`ros_ws/install`):

```bash
ros2 topic echo /nav/locations --once
ros2 topic echo /nav/state
ros2 topic pub --once /nav/goal_location std_msgs/String '{data: OT-2}'
ros2 topic pub --once /nav/cancel std_msgs/Empty '{}'
```

## Existing outer-stack integration (unchanged by this update)

`.env` already pointed at this package, so nothing there changed:

```
NAVIGATION_LAUNCH_PACKAGE="navigation_bringup"
NAVIGATION_LAUNCH_FILE="navigation.launch.xml"
```

`robot/docker/Dockerfile.robot` gained the dependencies the stack needs beyond the image:
`ros-humble-velodyne` and the phoenix6 **Python** package in
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

On the real robot, `robot_bringup`'s `robot.launch.xml` (`sim:=false`) runs `run.sh --hardware --yes` itself, as a
child process, so the separate terminal is no longer needed: `navigation:=false` leaves it out and
`navigation_hardware:=false` runs it on the phoenix6 simulator. Everything `run.sh` does still applies (domain 42,
tf2 overlay, CANivore bring-up); only the typed YES is skipped. The Xbox pad may be connected later.

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
`error_code`. Station recovery uses action status: an aborted staging leg can replan
to the same staging pose; a canceled/preempted leg stops and cannot restart itself.

Additional Humble compatibility in the current port:

- Humble behavior-server `global_frame` is `odom`, matching its local collision checker.
  Jazzy separates `local_frame=odom` from `global_frame=map`.
- Station actions tolerate Humble results without `error_code`; aborted translation/staging
  actions receive bounded recovery, while canceled actions still stop.
- The staging tree has a separate BT.CPP v3 version without Jazzy error-code ports.
- Humble overlays load after motion/speed overlays, preserving flat polygon points,
  `max_points`, slash-style behavior plugin IDs and the singular progress-checker setting.
- Humble has no VelocityPolygon. Fast mode retains the proven low-speed slowdown polygon
  and adds the native predictive Approach action at 0.55 s. At DockPath's maximum
  translation speed, this prediction stays inside the existing 8 cm stop margin.
- The tf2 0.25.24 overlay, disabled Phoenix diagnostics, CAN-clock monitoring and
  Humble message-callback adapter are preserved.

If you edit the Jazzy tree or the Nav2 parameters, keep the Humble copies in step.

## Station approach and recovery

`best.locations.yaml` now uses a 0.30 m final docking leg and a separate 0.80 m
staging/withdrawal distance. The robot aligns in checked open space at staging,
translates with its heading held to the 30 cm approach point, then starts final
parking with a fresh odometry heading anchor. Large alignment turns and withdrawal
stay at 80 cm. Small heading corrections are also allowed at the approach point
and after the final docking leg, before reporting success.

Fine corrections are limited to 0.25 rad (14.3 degrees) and 0.35 rad/s,
with two correction attempts per request. The downstream module-speed compensation
sets a practical in-place speed floor of about 0.334 rad/s; this remains enabled. A dedicated lifecycle-managed
Nav2 behavior server supplies this slow Spin action independently of fast staging
alignment. Both costmaps must clear the footprint throughout the requested angular
sweep; clearance is checked again while turning. The rotation lock is released
only for the correction action and restored during stopping/final verification.

Final station verification allows 0.12 m position error and 0.12 rad (6.9 degrees)
heading error. The controller aims for 0.10 m, leaving braking allowance. A stopped
base inside the verification limits may finish after the follower is canceled
and its terminal result is received. Final map/odometry agreement is still checked.

The shared Humble progress checker counts 3 cm of translation within 20 seconds.
For parking/approach translation, 8 seconds without 2 cm of displacement triggers
a stopped retry before that deadline. Aborted staging actions replan to staging;
small final position errors retry translation; heading errors first attempt a
checked small correction, otherwise withdraw along the current body axis and
realign in checked open space. Recovery waits for the
previous action to terminate and the base to stop. The same public request stays
`NAVIGATING` throughout recovery, with two retries total and a 360 second mission
limit. `/station_docking/status` describes each retry.

Cancellation, manual takeover, e-stop, stale perception, blocked withdrawal or
exhausted retries still stop the mission. Recovery does not bypass collision
monitoring or claim success outside the station limits. Stop navigation, run
`./src/autonomy/1_navigation/build.sh` in the Humble container, then restart
`./src/autonomy/1_navigation/run.sh --hardware` to load these changes.

Validation (2026-11-10): 632 checks passed under Humble/Python 3.10; the docking,
Humble compatibility and public API checks also passed under Jazzy. Tests cover
the observed 9.1/10.4 cm stalls, heading recovery through withdrawal/re-alignment,
retry exhaustion, cancellation during recovery, stale perception and blocked exits.
The isolated Humble Nav2 simulation completed shaker -> home -> OT-2 -> undock
with an injected 0.16 rad physical approach disturbance. It used a local heading
correction and completed with zero collisions and no failed requests. The measured
fine-turn output peaked at 0.3343 rad/s, within the 0.35 rad/s limit, including
downstream stiction compensation. Tests also cover post-docking heading correction,
changing obstacles during a turn and the physical excursion limit.
These are software/simulation checks;
the running physical navigation stack was neither rebuilt nor restarted.

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

Goals are republished on `/goal_pose_staged`, using the same station coordinator as RViz.
For maps with docking enabled, `/station_docking/result` owns the public outcome across
withdrawal, transit, alignment and final approach. Child NavigateToPose success does not
end the mission. Whole-mission ETA is unavailable (`time_remaining = -1`); distance is
measured against the final goal. For maps without docking enabled, direct Nav2 goals
retain the previous action-status/feedback behavior.

`api_namespace:=/robot_1` moves the whole API under that namespace.

## run.sh

Refuses `joy:=` (the Xbox pad is the e-stop), refuses a second stack on the same domain, and asks
for a typed `YES` before arming the drivetrain. Extra launch arguments pass through, e.g.
`map:=/path/to/other.yaml`, `rviz:=true`, `x:=1.0 y:=0.5 yaw:=1.57`, `api_namespace:=/robot_1`,
`nav_motion:=omni`, `approach_distance:=0`.

In the Humble robot container, RViz defaults to off even though the launch-file default is on;
run `rviz_docker.sh` from the host. On Jazzy, or when `rviz:=true` is explicit, the launch-file
setting is used unchanged.

The default map is `navigation_bringup/maps/best.yaml` with places in `best.locations.yaml`.
`navigation_bringup/phoenix6/tuner_constants.py` is the Tuner X output for this drivetrain,
copied from `~/nav_nav/Navigation/generated`: re-copy it after re-tuning.

## Safety

The lidar is blind behind and to the right of the robot over ~136°, so the stack drives
nose-first (`nav_motion:=forward`) by default. On hardware the pad is the only e-stop: `B`
latches stop, holding `LT` overrides Nav2, `X` cancels the goal.

Recovery matches `~/nav_nav`: after clearing costmaps it may spin 1.57 rad to recover visibility,
wait, or back up at 0.05 m/s for at most 0.15 m. The backup limit reduces exposure to the lidar's
rear/right blind area but does not make contact safe.

Hardware runs confirmed that the bridge publishes live odometry and the full Nav2 stack reaches
the active lifecycle state. The container's Humble RViz was the unstable component, so it is now
kept out of the robot process by default. A dry-run goal can still eventually fail because the
simulated base moves while the real lidar shows a stationary world and the pose estimate walks
into mapped obstacles; that happens on both distros and says nothing about the real robot.

## Wireless controller

The reader is `swerve_navigation/joy_auto`, launched with node name `joy_node`.
It prefers the Bluetooth 8BitDo Pro 2 in X mode (`Xbox One S Controller` in SDL)
and falls back to `Xbox 360 Controller` over USB. Only one reader publishes `/joy`.
Switching controllers requires releasing LT, centring sticks and releasing buttons;
B remains available during that handover. E-stop stays latched across reconnects.
Only the selected controller supplies controls. Losing both releases teleop; the
existing mux may then return to Nav2, as when LT is released.

Docker's privileged-device snapshot can miss a Bluetooth pad connected after the
container started. The controller may appear in `/sys/class/input` while its
`/dev/input/event*` node is missing inside Docker. Keyboard event nodes alone do
not prove the gamepad works. `run.sh --hardware` does not require the pad at start: it may be connected
after launch, and joy_auto selects it when it appears. Until then there is no pad e-stop.
`scripts/check_controller.py` opens the named controller with SDL as a read-only check, without publishing
ROS messages.

The default robot Compose configuration now bind-mounts the live host `/dev/input`
directory read-only for hotplug/reconnect. Containers created before that change
need to be recreated once. **First stop navigation and any other robot work: the
following command recreates the robot container.** Run on the host:

```bash
cd ~/coding/Autolab
docker compose --profile deploy -f docker-compose.yaml up -d --no-deps robot-l4t
```

The mount retains the existing service's input group. The included
`docker-input.override.yaml` remains available for older checkouts that lack the
default mount.
Inside the Humble container, rebuild and then use the usual runner:

```bash
cd /home/robot/AutoLab/robot/ros_ws
./src/autonomy/1_navigation/build.sh
python3 ./src/autonomy/1_navigation/scripts/check_controller.py
./src/autonomy/1_navigation/run.sh --hardware
```

Inspect the selected pad with `ros2 topic echo /joy/selected_device --once` in the
same navigation domain (42 by default). Normalisation, hotplug preference,
disconnection, held-LT handover and latched e-stop behavior have regression tests.
The port does not start a second reader in an already running navigation stack.

Wireless-port validation (2026-11-08): 599 Humble regression checks and a fresh
Humble package build passed. In a network-isolated Humble container with only the
Bluetooth event device exposed, the real SDL selector chose `Xbox One S Controller`
and published 20 correctly shaped Joy samples at about 20 Hz on private probe
topics. No teleop, velocity mux or hardware bridge was started in that test. The
live robot container was inspected but was not restarted or remounted.
