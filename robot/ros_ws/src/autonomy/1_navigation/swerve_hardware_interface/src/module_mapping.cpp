#include "swerve_hardware_interface/module_mapping.hpp"

#include <cmath>

namespace swerve_hardware_interface
{

std::vector<ModuleConfig> default_module_configs()
{
  return {
    {"FR", "Revolute_2", "Revolute_6", 12, 32, 22, -0.009521484375},
    {"FL", "Revolute_1", "Revolute_5", 10, 30, 20, -0.199462890625},
    {"RR", "Revolute_4", "Revolute_8", 13, 33, 23, -0.082275390625},
    {"RL", "Revolute_3", "Revolute_7", 11, 31, 21, -0.36376953125},
  };
}

const ModuleConfig * find_module_for_joint(
  const std::vector<ModuleConfig> & configs,
  const std::string & joint_name)
{
  for (const auto & cfg : configs) {
    if (cfg.drive_joint == joint_name || cfg.azimuth_joint == joint_name) {
      return &cfg;
    }
  }
  return nullptr;
}

double wheel_rotps_to_rotor_rotps(double wheel_rotps, double drive_gear_ratio)
{
  return wheel_rotps * drive_gear_ratio;
}

double rotor_rotps_to_wheel_rotps(double rotor_rotps, double drive_gear_ratio)
{
  return rotor_rotps / drive_gear_ratio;
}

double rad_to_rotor_rotations(double rad, double steer_gear_ratio)
{
  return (rad / (2.0 * M_PI)) * steer_gear_ratio;
}

double rotor_rotations_to_rad(double rotations, double steer_gear_ratio)
{
  return (rotations / steer_gear_ratio) * 2.0 * M_PI;
}

}  // namespace swerve_hardware_interface
