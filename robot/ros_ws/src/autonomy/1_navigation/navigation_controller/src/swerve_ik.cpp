#include "navigation_controller/swerve_ik.hpp"

#include <cmath>

namespace navigation_controller {

SwerveCommand compute_swerve_command(
  double linear_x_cmd, double linear_y_cmd, double angular_cmd,
  double wheel_radius, double x_offset)
{
  const double a = -linear_y_cmd - angular_cmd * x_offset / 2.0;
  const double b = -linear_y_cmd + angular_cmd * x_offset / 2.0;
  const double c_ = linear_x_cmd - angular_cmd * x_offset / 2.0;
  const double d = linear_x_cmd + angular_cmd * x_offset / 2.0;

  const double fl_vel = -std::hypot(b, d) / (wheel_radius * M_PI);
  const double fr_vel =  std::hypot(b, c_) / (wheel_radius * M_PI);
  const double rl_vel = -std::hypot(a, d) / (wheel_radius * M_PI);
  const double rr_vel =  std::hypot(a, c_) / (wheel_radius * M_PI);

  const double fl_pos = -std::atan2(b, d);
  const double fr_pos = -std::atan2(b, c_);
  const double rl_pos = -std::atan2(a, d);
  const double rr_pos = -std::atan2(a, c_);

  SwerveCommand cmd;
  cmd.fl.velocity = std::isnan(fl_vel) ? 0.0 : fl_vel;
  cmd.fr.velocity = std::isnan(fr_vel) ? 0.0 : fr_vel;
  cmd.rl.velocity = std::isnan(rl_vel) ? 0.0 : rl_vel;
  cmd.rr.velocity = std::isnan(rr_vel) ? 0.0 : rr_vel;
  cmd.fl.position = std::isnan(fl_pos) ? 0.0 : fl_pos;
  cmd.fr.position = std::isnan(fr_pos) ? 0.0 : fr_pos;
  cmd.rl.position = std::isnan(rl_pos) ? 0.0 : rl_pos;
  cmd.rr.position = std::isnan(rr_pos) ? 0.0 : rr_pos;
  return cmd;
}

}  // namespace navigation_controller
