#include <gtest/gtest.h>
#include "manipulation_executive/manipulation_logic.hpp"

using namespace manipulation_logic;

// ── april_command ───────────────────────────────────────────────────────────

TEST(AprilCommand, MapsConfiguredMachineToTagCommand) {
  const std::map<std::string, int64_t> tags{{"ot2", 1}, {"shaker", 2}};
  EXPECT_EQ(april_command(tags, "ot2"), std::optional<std::string>("plan_april_1"));
  EXPECT_EQ(april_command(tags, "shaker"), std::optional<std::string>("plan_april_2"));
}

TEST(AprilCommand, UsesWhateverIdIsConfigured) {
  const std::map<std::string, int64_t> tags{{"ot2", 21}};
  EXPECT_EQ(april_command(tags, "ot2"), std::optional<std::string>("plan_april_21"));
}

TEST(AprilCommand, UnknownMachineIsNullopt) {
  const std::map<std::string, int64_t> tags{{"ot2", 1}};
  EXPECT_FALSE(april_command(tags, "centrifuge").has_value());
  EXPECT_FALSE(april_command(tags, "").has_value());
}

TEST(AprilCommand, NegativeIdIsRejected) {
  const std::map<std::string, int64_t> tags{{"ot2", -1}};
  EXPECT_FALSE(april_command(tags, "ot2").has_value());
}

TEST(MachineNames, JoinsConfiguredNames) {
  const std::map<std::string, int64_t> tags{{"ot2", 1}, {"shaker", 2}};
  EXPECT_EQ(machine_names(tags), "ot2, shaker");
  EXPECT_EQ(machine_names({}), "<none>");
}

// ── parse_servo_status ──────────────────────────────────────────────────────

TEST(ParseServoStatus, ReadsRunningResultAndStart) {
  auto s = parse_servo_status(
      R"({"phase": "aligning", "running": true, "result": null, "started": 12.5})");
  ASSERT_TRUE(s.has_value());
  EXPECT_TRUE(s->running);
  EXPECT_EQ(s->result, "");
  EXPECT_DOUBLE_EQ(s->started, 12.5);
  EXPECT_EQ(s->phase, "aligning");
}

TEST(ParseServoStatus, FinishedRunCarriesResult) {
  auto s = parse_servo_status(
      R"({"phase": "idle", "running": false, "result": "grasped", "message": "ok", "started": 3.0})");
  ASSERT_TRUE(s.has_value());
  EXPECT_FALSE(s->running);
  EXPECT_EQ(s->result, "grasped");
  EXPECT_EQ(s->message, "ok");
}

TEST(ParseServoStatus, MissingStartedIsMinusOne) {
  auto s = parse_servo_status(R"({"phase": "idle", "running": false, "result": null})");
  ASSERT_TRUE(s.has_value());
  EXPECT_DOUBLE_EQ(s->started, -1.0);
}

TEST(ParseServoStatus, InvalidJsonIsNullopt) {
  EXPECT_FALSE(parse_servo_status("not json").has_value());
  EXPECT_FALSE(parse_servo_status("[1, 2]").has_value());
}

// ── ServoRunTracker ─────────────────────────────────────────────────────────

static ServoStatus st(bool running, const std::string& result, double started) {
  ServoStatus s;
  s.running = running;
  s.result  = result;
  s.started = started;
  return s;
}

TEST(ServoRunTracker, IgnoresStaleFinishedStatusBeforeRunStarts) {
  // transient-local status from the previous run arrives first
  ServoRunTracker t;
  t.update(st(false, "grasped", 1.0));
  EXPECT_FALSE(t.finished());
}

TEST(ServoRunTracker, FinishesAfterRunningThenStopped) {
  ServoRunTracker t;
  t.update(st(false, "failed", 1.0));   // stale
  t.update(st(true, "", 5.0));
  EXPECT_FALSE(t.finished());
  t.update(st(true, "", 5.0));
  EXPECT_FALSE(t.finished());
  t.update(st(false, "grasped", 5.0));
  EXPECT_TRUE(t.finished());
  EXPECT_EQ(t.result(), "grasped");
}

TEST(ServoRunTracker, IgnoresFinishOfADifferentRun) {
  ServoRunTracker t;
  t.update(st(true, "", 5.0));
  t.update(st(false, "failed", 4.0));   // older run's status, out of order
  EXPECT_FALSE(t.finished());
  t.update(st(false, "aligned", 5.0));
  EXPECT_TRUE(t.finished());
  EXPECT_EQ(t.result(), "aligned");
}

TEST(ServoRunTracker, StaysFinishedOnceFinished) {
  ServoRunTracker t;
  t.update(st(true, "", 5.0));
  t.update(st(false, "grasped", 5.0));
  t.update(st(true, "", 9.0));          // someone else started a new run
  EXPECT_TRUE(t.finished());
  EXPECT_EQ(t.result(), "grasped");
}

// ── servo_succeeded ─────────────────────────────────────────────────────────

TEST(ServoSucceeded, OnlyConfiguredResultsSucceed) {
  const std::vector<std::string> ok{"grasped", "aligned", "stopped"};
  EXPECT_TRUE(servo_succeeded("grasped", ok));
  EXPECT_TRUE(servo_succeeded("aligned", ok));
  EXPECT_TRUE(servo_succeeded("stopped", ok));
  EXPECT_FALSE(servo_succeeded("failed", ok));
  EXPECT_FALSE(servo_succeeded("aborted", ok));
  EXPECT_FALSE(servo_succeeded("", ok));
}
