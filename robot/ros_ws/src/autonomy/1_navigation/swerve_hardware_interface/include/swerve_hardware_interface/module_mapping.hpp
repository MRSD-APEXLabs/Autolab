#pragma once

#include <string>
#include <vector>

namespace swerve_hardware_interface
{

struct ModuleConfig
{
  std::string label;          // "FR", "FL", "RR", "RL"
  std::string drive_joint;    // e.g. "Revolute_1"
  std::string azimuth_joint;  // e.g. "Revolute_5"
  int drive_can_id;
  int steer_can_id;
  int cancoder_id;
  double cancoder_offset_rot;
};

std::vector<ModuleConfig> default_module_configs();

const ModuleConfig * find_module_for_joint(
  const std::vector<ModuleConfig> & configs,
  const std::string & joint_name);

double wheel_rotps_to_rotor_rotps(double wheel_rotps, double drive_gear_ratio);
double rotor_rotps_to_wheel_rotps(double rotor_rotps, double drive_gear_ratio);
double rad_to_rotor_rotations(double rad, double steer_gear_ratio);
double rotor_rotations_to_rad(double rotations, double steer_gear_ratio);

}  // namespace swerve_hardware_interface
