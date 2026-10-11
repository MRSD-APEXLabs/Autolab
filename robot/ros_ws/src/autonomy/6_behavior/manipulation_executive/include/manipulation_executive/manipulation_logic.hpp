#pragma once

// Pure decision logic for ManipulationExecutive — no ROS, so it is unit-testable
// on its own (test/test_manipulation_logic.cpp).

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace manipulation_logic {

// ─────────────────────────────────────────────────────────────────────────────
// Lab machine → AprilTag planning command
// ─────────────────────────────────────────────────────────────────────────────

// "plan_april_<id>" for a machine in the april_tags map, nullopt if the machine
// is not configured (or its id is negative).
inline std::optional<std::string> april_command(
    const std::map<std::string, int64_t>& april_tags, const std::string& machine)
{
    auto it = april_tags.find(machine);
    if (it == april_tags.end() || it->second < 0) return std::nullopt;
    return "plan_april_" + std::to_string(it->second);
}

// Configured machine names for log messages: "ot2, shaker".
inline std::string machine_names(const std::map<std::string, int64_t>& april_tags)
{
    if (april_tags.empty()) return "<none>";
    std::string out;
    for (const auto& [name, id] : april_tags) {
        if (!out.empty()) out += ", ";
        out += name;
    }
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// visual_servo/status (std_msgs/String JSON from visual_servoing's servo_node)
// ─────────────────────────────────────────────────────────────────────────────

struct ServoStatus {
    bool        running{false};
    std::string result;          // "" while running; grasped|aligned|failed|aborted|stopped|done after
    std::string message;
    std::string phase;
    double      started{-1.0};   // servo node's monotonic start time — identifies the run
};

inline std::optional<ServoStatus> parse_servo_status(const std::string& text)
{
    try {
        auto doc = nlohmann::json::parse(text);
        if (!doc.is_object()) return std::nullopt;
        ServoStatus s;
        s.running = doc.value("running", false);
        if (doc.contains("result")  && doc["result"].is_string())  s.result  = doc["result"];
        if (doc.contains("message") && doc["message"].is_string()) s.message = doc["message"];
        if (doc.contains("phase")   && doc["phase"].is_string())   s.phase   = doc["phase"];
        if (doc.contains("started") && doc["started"].is_number()) s.started = doc["started"];
        return s;
    } catch (const nlohmann::json::exception&) {
        return std::nullopt;
    }
}

// Follows one servo run through its status messages. The status topic is
// transient-local, so the previous run's final status arrives first: a run only
// counts once it has been seen running, and it finishes when that same run
// (same `started`) reports running == false.
class ServoRunTracker {
public:
    void update(const ServoStatus& s)
    {
        if (finished_) return;
        if (!seen_running_) {
            if (s.running) {
                seen_running_ = true;
                started_      = s.started;
            }
            return;
        }
        if (!s.running && s.started == started_) {
            finished_ = true;
            result_   = s.result;
            message_  = s.message;
        }
    }

    bool               seen_running() const { return seen_running_; }
    bool               finished()     const { return finished_; }
    const std::string& result()       const { return result_; }
    const std::string& message()      const { return message_; }

private:
    bool        seen_running_{false};
    bool        finished_{false};
    double      started_{-1.0};
    std::string result_;
    std::string message_;
};

inline bool servo_succeeded(const std::string& result,
                            const std::vector<std::string>& success_results)
{
    if (result.empty()) return false;
    return std::find(success_results.begin(), success_results.end(), result)
           != success_results.end();
}

}  // namespace manipulation_logic
