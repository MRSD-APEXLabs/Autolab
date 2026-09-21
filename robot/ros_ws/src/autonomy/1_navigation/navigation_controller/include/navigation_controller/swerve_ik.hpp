#pragma once

namespace navigation_controller {

struct WheelCommand {
  double velocity{0.0};
  double position{0.0};
};

struct SwerveCommand {
  WheelCommand fr;
  WheelCommand fl;
  WheelCommand rr;
  WheelCommand rl;
};

// Reuses the swerve inverse-kinematics arithmetic that used to live inline
// in NavigationController::control_loop(). linear_x_cmd/linear_y_cmd are
// in the robot (base_link) frame, angular_cmd is yaw rate. wheel_radius is
// in meters, x_offset is the module spacing term used by the original
// P-control loop's wheel-vector formula.
SwerveCommand compute_swerve_command(
  double linear_x_cmd, double linear_y_cmd, double angular_cmd,
  double wheel_radius, double x_offset);

}  // namespace navigation_controller
