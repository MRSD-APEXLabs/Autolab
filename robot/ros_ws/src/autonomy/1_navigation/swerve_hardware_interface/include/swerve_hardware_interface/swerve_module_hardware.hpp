#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANBus.hpp>

#include "swerve_hardware_interface/module_mapping.hpp"

namespace swerve_hardware_interface
{

struct SharedModuleParams
{
  double drive_gear_ratio;
  double steer_gear_ratio;
  double coupling_gear_ratio;
  double steer_kp, steer_ki, steer_kd, steer_ks, steer_kv;
  double drive_kp, drive_ki, drive_kd, drive_ks, drive_kv;
  double steer_stator_current_limit_a;
};

class SwerveModuleHardware
{
public:
  SwerveModuleHardware(
    const ModuleConfig & cfg,
    const SharedModuleParams & params,
    ctre::phoenix6::CANBus & bus);

  bool configure();

  void set_drive_velocity_rotps(double wheel_rotps);
  void set_steer_position_rad(double rad);

  double get_drive_velocity_rotps();
  double get_steer_position_rad();

  bool is_healthy();

private:
  ModuleConfig cfg_;
  SharedModuleParams params_;
  ctre::phoenix6::hardware::TalonFX drive_;
  ctre::phoenix6::hardware::TalonFX steer_;
  ctre::phoenix6::hardware::CANcoder cancoder_;
};

}  // namespace swerve_hardware_interface
