#include <gtest/gtest.h>
#include "manipulation_executive/manipulation_executive.hpp"

using ManipPhase = ManipulationExecutive::ManipPhase;

TEST(CameraModeCmdGuard, AllowedWhenIdle) {
  EXPECT_TRUE(ManipulationExecutive::camera_mode_cmd_allowed(ManipPhase::IDLE));
}

TEST(CameraModeCmdGuard, RejectedWhenActivatingInspect) {
  EXPECT_FALSE(ManipulationExecutive::camera_mode_cmd_allowed(ManipPhase::ACTIVATING_INSPECT));
}

TEST(CameraModeCmdGuard, RejectedWhenPlanning) {
  EXPECT_FALSE(ManipulationExecutive::camera_mode_cmd_allowed(ManipPhase::PLANNING));
}

TEST(CameraModeCmdGuard, RejectedWhenWaitingPlace) {
  EXPECT_FALSE(ManipulationExecutive::camera_mode_cmd_allowed(ManipPhase::WAITING_PLACE));
}

TEST(CameraModeCmdGuard, RejectedWhenActivatingServo) {
  EXPECT_FALSE(ManipulationExecutive::camera_mode_cmd_allowed(ManipPhase::ACTIVATING_SERVO));
}

TEST(CameraModeCmdGuard, RejectedWhenPlanningSafe) {
  EXPECT_FALSE(ManipulationExecutive::camera_mode_cmd_allowed(ManipPhase::PLANNING_SAFE));
}
