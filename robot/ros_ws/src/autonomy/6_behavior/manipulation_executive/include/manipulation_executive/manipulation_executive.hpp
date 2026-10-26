#pragma once

#include <atomic>
#include <cstdint>
#include <future>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <behavior_tree/behavior_tree.hpp>
#include <behavior_tree_msgs/msg/manipulation_command.hpp>
#include <manipulation_executive/manipulation_logic.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class ManipulationExecutive : public rclcpp::Node {
public:
    ManipulationExecutive();

    enum class ManipType  { PICK_UP, PLACE, PICK_BASE, PICK_WELLPLATE };
    enum class ManipPhase {
        IDLE,
        SELECTING_CAMERA,
        PLANNING,
        WAITING_PLACE,
        SERVOING,
        PLANNING_SAFE,
    };

private:
    // ── BT nodes ──────────────────────────────────────────────────────────────
    bt::Condition* pick_up_condition_;
    bt::Condition* place_condition_;
    bt::Condition* pick_base_condition_;
    bt::Condition* pick_wellplate_condition_;
    std::vector<bt::Condition*> conditions_;

    bt::Action* pick_up_action_;
    bt::Action* place_action_;
    bt::Action* pick_base_action_;
    bt::Action* pick_wellplate_action_;
    std::vector<bt::Action*> actions_;

    // ── Manipulation state (shared; only one action active at a time) ──────────
    ManipType  manip_type_    {ManipType::PICK_UP};
    ManipPhase manip_phase_   {ManipPhase::IDLE};
    bool       manip_terminal_{false};

    std::string object_type_;    // e.g. "well_plate"
    std::string target_machine_; // (place / pick_wellplate) e.g. "ot2"

    std::future<bool> pending_;
    bool              in_flight_{false};
    // Set when the BT deactivates the action mid-flight; the async waits bail out.
    std::atomic<bool> abort_{false};

    // ── ROS2 params ───────────────────────────────────────────────────────────
    double                         place_wait_s_;
    std::map<std::string, int64_t> april_tags_;            // target_machine → AprilTag id
    std::string                    camera_select_service_;
    double                         camera_select_timeout_s_;
    std::string                    servo_start_service_;
    std::string                    servo_stop_service_;
    std::string                    servo_status_topic_;
    std::vector<std::string>       servo_success_results_;

    // ── ROS2 infrastructure ───────────────────────────────────────────────────
    rclcpp::CallbackGroup::SharedPtr io_group_;   // service clients + servo status
    rclcpp::Subscription<behavior_tree_msgs::msg::ManipulationCommand>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  planning_cmd_pub_;
    // 0=unknown/waiting, 1=SUCCESS, 2=ERROR
    std::atomic<int> planning_result_{0};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planning_state_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr camera_select_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_stop_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr servo_status_sub_;
    std::mutex                          servo_mutex_;
    manipulation_logic::ServoRunTracker servo_tracker_;   // guarded by servo_mutex_
    rclcpp::TimerBase::SharedPtr timer_;

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void command_callback(const behavior_tree_msgs::msg::ManipulationCommand::SharedPtr msg);
    void servo_status_callback(const std_msgs::msg::String::SharedPtr msg);
    void timer_callback();

    // ── State machine ─────────────────────────────────────────────────────────
    void tick_manip(bt::Action* action, ManipType type);
    void reset_manip();
    void fail_manip(bt::Action* action);
    void start_select_camera();

    // ── Async operations (blocking; run in std::async threads) ────────────────
    // Calls the camera hub's select service (the base ZED X, which the planner's
    // /inspect/* topics come from) and waits for the camera to stream.
    bool select_camera();

    // Starts a visual_servoing run and waits for it to finish; true if its result
    // is one of servo_success_results.
    bool run_servo();

    // Publishes to /planning_command and waits for /planning_state → SUCCESS or ERROR.
    // pose_type: "inspection" | "placement" | "wellplate_placement" | "safe" | "home_offset"
    bool run_planning(const std::string& pose_type);

    // Sends a Trigger request and waits up to timeout_s; false (with `message` set)
    // if the service is missing, does not answer or answers success=false.
    bool call_trigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
                      double timeout_s, std::string& message);
};
