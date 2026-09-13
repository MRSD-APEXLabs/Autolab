# Navigation (Swerve Base)

**Purpose:** drive the 4-wheel swerve base. **Status: code exists but is not part of the lab routine** — in the current deployment the robot is effectively stationary and only the arm moves.

Paths: `robot/ros_ws/src/autonomy/1_navigation/`.

## What exists

- `navigation_controller` (C++, `src/navigation_controller.cpp`) — converts a base goal pose into per-module steer/wheel commands for the swerve drive:
    - **Subscribes:** `/robot_1/joints/joint_states` (JointState), `/robot_1/swerve/goal_pose` (Pose2D), `/robot_1/odom` (Odometry)
    - **Publishes:** `/robot_1/joints/joint_command` (JointState)
- `navigation_bringup` — `launch/navigation.launch.xml` (runs the controller under namespace `navigation_controller`)

```bash
ros2 launch navigation_bringup navigation.launch.xml
# or
ros2 run navigation_controller navigation_controller
```

## Related but dormant

- The swerve hardware joints (`Az_{Front,Back}_{Left,Right}_1` steering + `Wheel_*_1`) are modeled in `base_urdf/robot.urdf` and appear as *passive* joints in the MoveIt SRDF.
- `2_mapping_and_localization` vendored RTAB-Map is unwired (empty bringup launch); `4_local` (`local_planner.cpp`) is a stub.
- There is no odometry source or base hardware driver wired up in this repo — publishing to `/robot_1/swerve/goal_pose` moves nothing unless a base driver consumes `/robot_1/joints/joint_command`.
