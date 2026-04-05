#pragma once

#include <nlohmann/json.hpp>

#include <future>
#include <string>
#include <vector>

#include <behavior_tree/behavior_tree.hpp>
#include <behavior_tree_msgs/msg/manipulation_command.hpp>
#include <rclcpp/rclcpp.hpp>

class ManipulationExecutive : public rclcpp::Node {
public:
    ManipulationExecutive();

private:
    // ── BT nodes ──────────────────────────────────────────────────────────────
    bt::Condition* pick_up_condition_;
    bt::Condition* place_condition_;
    std::vector<bt::Condition*> conditions_;

    bt::Action* pick_up_action_;
    bt::Action* place_action_;
    std::vector<bt::Action*> actions_;

    // ── Manipulation state (shared; only one action active at a time) ──────────
    enum class ManipType  { PICK_UP, PLACE };
    enum class ManipPhase {
        IDLE,
        ACTIVATING_INSPECT,  // send inspect mode; poll until active
        PLANNING,            // TODO: real planning service; placeholder: sleep
        ACTIVATING_SERVO,    // send servo mode; poll until complete
        PLANNING_SAFE,       // TODO: real planning service; placeholder: sleep
    };

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
    double      planning_placeholder_s_;  // seconds to simulate planning

    // ── ROS2 infrastructure ───────────────────────────────────────────────────
    rclcpp::Subscription<behavior_tree_msgs::msg::ManipulationCommand>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void command_callback(const behavior_tree_msgs::msg::ManipulationCommand::SharedPtr msg);
    void timer_callback();

    // ── State machine ─────────────────────────────────────────────────────────
    void tick_manip(bt::Action* action, ManipType type);
    void reset_manip();
    void fail_manip(bt::Action* action);

    // ── Async operations (blocking; run in std::async threads) ────────────────
    // Send { cmd: mode, mode: <mode> } then poll { cmd: status } until the
    // reported state matches expect_state. Returns false on timeout/error.
    bool activate_camera_mode(const std::string& mode, const std::string& expect_state);

    // Placeholder planning: logs intent and sleeps for planning_placeholder_s_.
    bool run_planning(const std::string& pose_type);

    // ── WebSocket helper (synchronous, blocking) ───────────────────────────────
    // Connects to camera_edge_host_:camera_edge_port_, sends msg, reads one
    // response, closes. Returns response string or empty on error.
    std::string ws_round_trip(const std::string& msg);
};
