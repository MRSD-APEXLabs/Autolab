# Navigation Controller

## Overview

This ROS 2 node implements a goal-based navigation controller for a four-wheel swerve-drive mobile robot. It converts a 2D goal pose and odometry feedback into wheel drive velocities and azimuth angles using a proportional control law and swerve kinematics.

## Node

**Name:** `navigation_controller`

## Subscribed Topics

* `/robot_1/joints/joint_states` (`sensor_msgs/JointState`)
  Used to track current joint states (stored, not directly used in control).

* `/robot_1/swerve/goal_pose` (`geometry_msgs/Pose2D`)
  Navigation goal.

  * `x`, `y`: position in meters
  * `theta`: yaw in degrees

* `/robot_1/odom` (`nav_msgs/Odometry`)
  Robot pose estimate used for feedback control.

## Published Topics

* `/robot_1/joints/joint_command` (`sensor_msgs/JointState`)
  Commands for wheel drive velocities and azimuth joint positions.

## Parameters

* `wheel_radius` (double, default: `0.0508` (2 inches))
  Wheel radius in meters.

* `kp_xy` (double, default: `1.0`)
  Proportional gain for planar position error.

* `kp_yaw` (double, default: `2.0`)
  Proportional gain for yaw error.

* `loop_hz` (double, default: `20.0`)
  Control loop frequency.

* `x_offset` (double, default: `0.25`)
  Longitudinal wheelbase offset used in swerve kinematics.

## Joint Mapping

The controller assumes the following joint naming and ordering:

### Drive Joints

1. `Revolute_1` – front right
2. `Revolute_2` – front left
3. `Revolute_4` – rear right
4. `Revolute_3` – rear left

### Azimuth Joints

1. `Revolute_5` – front right
2. `Revolute_6` – front left
3. `Revolute_8` – rear right
4. `Revolute_7` – rear left

## Control Logic

1. Compute position and yaw error between current odometry and goal.
2. Transform position error into the robot frame.
3. Apply proportional control to generate linear and angular velocity commands.
4. Use swerve-drive kinematics to compute:

   * Wheel angular velocities
   * Wheel steering (azimuth) angles
5. Publish joint commands at the configured control rate.

## Angle Conventions

* Internal yaw values are in radians.
* Goal yaw is provided in degrees and converted internally.
* All angles are normalized to `[-π, π]`.

## Usage

- Publish a `geometry_msgs/Pose2D` message to `/robot_1/swerve/goal_pose`.


## Future Improvements

* Limit velocity or acceleration.
* Controller assumes odometry from Base, it should have been from Mapping and Localization module.
* Include Kd