#pragma once

#include <nlohmann/json.hpp>

#include <atomic>
#include <future>
#include <string>
#include <vector>

#include <behavior_tree/behavior_tree.hpp>
#include <behavior_tree_msgs/msg/manipulation_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class ManipulationExecutive : public rclcpp::Node {
public:
    ManipulationExecutive();

    enum class ManipType  { PICK_UP, PLACE };
    enum class ManipPhase {
        IDLE,
        ACTIVATING_INSPECT,
        PLANNING,
        ACTIVATING_SERVO,
        PLANNING_SAFE,
    };

private:
    // ── BT nodes ──────────────────────────────────────────────────────────────
    bt::Condition* pick_up_condition_;
    bt::Condition* place_condition_;
    std::vector<bt::Condition*> conditions_;

    bt::Action* pick_up_action_;
    bt::Action* place_action_;
    std::vector<bt::Action*> actions_;

    // ── Manipulation state (shared; only one action active at a time) ──────────
    ManipType  manip_type_    {ManipType::PICK_UP};
    ManipPhase manip_phase_   {ManipPhase::IDLE};
    bool       manip_terminal_{false};

    std::string object_type_;    // e.g. "well_plate"
    std::string target_machine_; // (place only) e.g. "ot2"

    std::future<bool> pending_;
    bool              in_flight_{false};

    // ── ROS2 params ───────────────────────────────────────────────────────────
    std::string camera_edge_host_;
    int         camera_edge_port_;

    // ── ROS2 infrastructure ───────────────────────────────────────────────────
    rclcpp::Subscription<behavior_tree_msgs::msg::ManipulationCommand>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  planning_cmd_pub_;
    std::atomic<int> planning_done_flag_{-1}; // -1=pending, 0=failed, 1=success
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planning_done_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void command_callback(const behavior_tree_msgs::msg::ManipulationCommand::SharedPtr msg);
    void timer_callback();

    // ── State machine ─────────────────────────────────────────────────────────
    void tick_manip(bt::Action* action, ManipType type);
    void reset_manip();
    void fail_manip(bt::Action* action);

    // ── Async operations (blocking; run in std::async threads) ────────────────
    // Send { cmd: mode, mode: <mode> } then poll { cmd: status }.
    // wait_complete=false: return true once worker_alive==true (inspect — just confirm active).
    // wait_complete=true:  return true once mode returns to idle (servo — wait for finish).
    bool activate_camera_mode(const std::string& mode, bool wait_complete);

    // Publishes to /planning_command and polls /planning_done (via planning_done_flag_).
    // pose_type: "inspection" | "placement" | "safe"
    bool run_planning(const std::string& pose_type);

    // ── WebSocket helper (synchronous, blocking) ───────────────────────────────
    // Connects to camera_edge_host_:camera_edge_port_, sends msg, reads one
    // response, closes. Returns response string or empty on error.
    std::string ws_round_trip(const std::string& msg);
};
