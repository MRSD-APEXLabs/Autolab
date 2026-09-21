#include <cmath>
#include <set>
#include <vector>

#include <gtest/gtest.h>
#include "swerve_hardware_interface/module_mapping.hpp"

using swerve_hardware_interface::default_module_configs;
using swerve_hardware_interface::find_module_for_joint;
using swerve_hardware_interface::ModuleConfig;

TEST(ModuleMapping, ReturnsFourModules) {
  auto configs = default_module_configs();
  ASSERT_EQ(configs.size(), 4u);
}

TEST(ModuleMapping, FrontRightMatchesSpec) {
  auto configs = default_module_configs();
  const ModuleConfig* fr = find_module_for_joint(configs, "Revolute_1");
  ASSERT_NE(fr, nullptr);
  EXPECT_EQ(fr->label, "FR");
  EXPECT_EQ(fr->azimuth_joint, "Revolute_5");
  EXPECT_EQ(fr->drive_can_id, 12);
  EXPECT_EQ(fr->steer_can_id, 32);
  EXPECT_EQ(fr->cancoder_id, 22);
  EXPECT_DOUBLE_EQ(fr->cancoder_offset_rot, -0.009521484375);
}

TEST(ModuleMapping, FrontLeftMatchesSpec) {
  auto configs = default_module_configs();
  const ModuleConfig* fl = find_module_for_joint(configs, "Revolute_6");
  ASSERT_NE(fl, nullptr);
  EXPECT_EQ(fl->label, "FL");
  EXPECT_EQ(fl->drive_joint, "Revolute_2");
  EXPECT_EQ(fl->drive_can_id, 10);
  EXPECT_EQ(fl->steer_can_id, 30);
  EXPECT_EQ(fl->cancoder_id, 20);
  EXPECT_DOUBLE_EQ(fl->cancoder_offset_rot, -0.199462890625);
}

TEST(ModuleMapping, RearRightMatchesSpec) {
  auto configs = default_module_configs();
  const ModuleConfig* rr = find_module_for_joint(configs, "Revolute_4");
  ASSERT_NE(rr, nullptr);
  EXPECT_EQ(rr->label, "RR");
  EXPECT_EQ(rr->azimuth_joint, "Revolute_8");
  EXPECT_EQ(rr->drive_can_id, 13);
  EXPECT_EQ(rr->steer_can_id, 33);
  EXPECT_EQ(rr->cancoder_id, 23);
  EXPECT_DOUBLE_EQ(rr->cancoder_offset_rot, -0.082275390625);
}

TEST(ModuleMapping, RearLeftMatchesSpec) {
  auto configs = default_module_configs();
  const ModuleConfig* rl = find_module_for_joint(configs, "Revolute_7");
  ASSERT_NE(rl, nullptr);
  EXPECT_EQ(rl->label, "RL");
  EXPECT_EQ(rl->drive_joint, "Revolute_3");
  EXPECT_EQ(rl->drive_can_id, 11);
  EXPECT_EQ(rl->steer_can_id, 31);
  EXPECT_EQ(rl->cancoder_id, 21);
  EXPECT_DOUBLE_EQ(rl->cancoder_offset_rot, -0.36376953125);
}

TEST(ModuleMapping, AllCanIdsAreUnique) {
  auto configs = default_module_configs();
  std::set<int> ids;
  for (const auto & cfg : configs) {
    ids.insert(cfg.drive_can_id);
    ids.insert(cfg.steer_can_id);
    ids.insert(cfg.cancoder_id);
  }
  EXPECT_EQ(ids.size(), 12u);
}

TEST(ModuleMapping, UnknownJointReturnsNull) {
  auto configs = default_module_configs();
  EXPECT_EQ(find_module_for_joint(configs, "not_a_joint"), nullptr);
}

TEST(GearRatioConversion, WheelToRotorRoundTrips) {
  double rotor = swerve_hardware_interface::wheel_rotps_to_rotor_rotps(2.0, 7.03125);
  EXPECT_DOUBLE_EQ(rotor, 14.0625);
  EXPECT_DOUBLE_EQ(
    swerve_hardware_interface::rotor_rotps_to_wheel_rotps(rotor, 7.03125), 2.0);
}

TEST(GearRatioConversion, RadToRotorRoundTrips) {
  double rotations = swerve_hardware_interface::rad_to_rotor_rotations(M_PI, 26.09090909090909);
  EXPECT_NEAR(rotations, 13.04545454545, 1e-6);
  EXPECT_NEAR(
    swerve_hardware_interface::rotor_rotations_to_rad(rotations, 26.09090909090909),
    M_PI, 1e-9);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
