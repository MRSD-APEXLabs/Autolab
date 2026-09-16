#include <cmath>

#include <gtest/gtest.h>
#include "navigation_controller/swerve_ik.hpp"
#include "navigation_controller/cmd_vel_watchdog.hpp"

using navigation_controller::compute_swerve_command;

static constexpr double kWheelRadius = 0.0508;
static constexpr double kXOffset = 0.25;

TEST(SwerveIk, ZeroCommandIsAllZero) {
  auto cmd = compute_swerve_command(0.0, 0.0, 0.0, kWheelRadius, kXOffset);
  EXPECT_DOUBLE_EQ(cmd.fl.velocity, 0.0);
  EXPECT_DOUBLE_EQ(cmd.fr.velocity, 0.0);
  EXPECT_DOUBLE_EQ(cmd.rl.velocity, 0.0);
  EXPECT_DOUBLE_EQ(cmd.rr.velocity, 0.0);
  EXPECT_DOUBLE_EQ(cmd.fl.position, 0.0);
  EXPECT_DOUBLE_EQ(cmd.fr.position, 0.0);
  EXPECT_DOUBLE_EQ(cmd.rl.position, 0.0);
  EXPECT_DOUBLE_EQ(cmd.rr.position, 0.0);
}

TEST(SwerveIk, PureForwardPointsAllWheelsStraight) {
  auto cmd = compute_swerve_command(1.0, 0.0, 0.0, kWheelRadius, kXOffset);
  EXPECT_NEAR(cmd.fl.position, 0.0, 1e-9);
  EXPECT_NEAR(cmd.fr.position, 0.0, 1e-9);
  EXPECT_NEAR(cmd.rl.position, 0.0, 1e-9);
  EXPECT_NEAR(cmd.rr.position, 0.0, 1e-9);
  // FR/RR are mounted mirrored relative to FL/RL, so drive sign flips
  // for the same forward command (matches original inline formula).
  EXPECT_GT(cmd.fl.velocity, 0.0);
  EXPECT_NEAR(cmd.fr.velocity, -cmd.fl.velocity, 1e-9);
  EXPECT_NEAR(cmd.rl.velocity, cmd.fl.velocity, 1e-9);
  EXPECT_NEAR(cmd.rr.velocity, -cmd.fl.velocity, 1e-9);
}

TEST(SwerveIk, PureStrafeTurnsWheelsSideways) {
  auto cmd = compute_swerve_command(0.0, 1.0, 0.0, kWheelRadius, kXOffset);
  EXPECT_NEAR(std::fabs(cmd.fl.position), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(std::fabs(cmd.fr.position), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(std::fabs(cmd.rl.position), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(std::fabs(cmd.rr.position), M_PI / 2.0, 1e-9);
}

TEST(SwerveIk, PureRotationProducesXPattern) {
  auto cmd = compute_swerve_command(0.0, 0.0, 1.0, kWheelRadius, kXOffset);
  EXPECT_NEAR(cmd.fl.position, -M_PI / 4.0, 1e-9);
  EXPECT_NEAR(cmd.fr.position, -3.0 * M_PI / 4.0, 1e-9);
  EXPECT_NEAR(cmd.rl.position, M_PI / 4.0, 1e-9);
  EXPECT_NEAR(cmd.rr.position, 3.0 * M_PI / 4.0, 1e-9);
}

TEST(CmdVelWatchdog, NotStaleWithinTimeout) {
  EXPECT_FALSE(navigation_controller::is_cmd_vel_stale(0.1, 0.3));
  EXPECT_FALSE(navigation_controller::is_cmd_vel_stale(0.299, 0.3));
}

TEST(CmdVelWatchdog, StaleAtOrPastTimeout) {
  EXPECT_TRUE(navigation_controller::is_cmd_vel_stale(0.3, 0.3));
  EXPECT_TRUE(navigation_controller::is_cmd_vel_stale(1.0, 0.3));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
