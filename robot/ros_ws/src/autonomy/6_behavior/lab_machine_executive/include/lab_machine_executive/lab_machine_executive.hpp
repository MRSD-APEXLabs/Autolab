// Copyright (c) 2024 Carnegie Mellon University
// MIT License — see LICENSE for details.

#pragma once

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <future>
#include <string>
#include <vector>

#include <behavior_tree/behavior_tree.hpp>
#include <behavior_tree_msgs/msg/lab_machine_command.hpp>
#include <rclcpp/rclcpp.hpp>

class LabMachineExecutive : public rclcpp::Node {
   public:
    LabMachineExecutive();

   private:
    // ── BT conditions (published every tick; set via LabMachineCommand) ──────
    bt::Condition* command_lab_machine_condition_;
    bt::Condition* ot2_condition_;
    bt::Condition* shaker_condition_;
    std::vector<bt::Condition*> conditions_;

    // ── BT actions ────────────────────────────────────────────────────────────
    bt::Action* ot2_action_;
    bt::Action* shaker_action_;
    std::vector<bt::Action*> actions_;

    // ── Stored protocols ─────────────────────────────────────────────────────
    std::string ot2_protocol_json_;    // latest LabMachineCommand for ot2
    std::string shaker_protocol_json_; // latest LabMachineCommand for shaker

    // ── OT-2 step-machine state ───────────────────────────────────────────────
    enum class OT2Phase { IDLE, CONNECTING, STEPPING, HOMING, DISCONNECTING };
    nlohmann::json ot2_steps_;     // parsed steps array
    int ot2_current_step_;         // index into ot2_steps_
    OT2Phase ot2_phase_;
    bool ot2_terminal_;            // true once success/failure set; cleared on deactivation
    std::future<bool> pending_http_; // in-flight HTTP call
    bool http_in_flight_;

    // ── ROS2 params ───────────────────────────────────────────────────────────
    std::string ot2_base_url_;
    std::string ot2_host_;         // OT-2 robot IP passed to /robot/connect
    std::string shaker_base_url_;
    std::string shaker_endpoint_;  // e.g. "/pwm"

    // ── ROS2 infrastructure ───────────────────────────────────────────────────
    rclcpp::Subscription<behavior_tree_msgs::msg::LabMachineCommand>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ── Callbacks ────────────────────────────────────────────────────────────
    void command_callback(const behavior_tree_msgs::msg::LabMachineCommand::SharedPtr msg);
    void timer_callback();

    // ── Helpers ───────────────────────────────────────────────────────────────
    void tick_ot2();
    void tick_shaker();
    bool dispatch_ot2_step(const nlohmann::json& step);

    // Static HTTP helper — runs in std::async thread
    bool connect_ot2();
    bool home_ot2();
    bool disconnect_ot2();
    static bool http_post(const std::string& url,
                          const std::string& body,
                          const std::string& content_type);
    static std::string action_to_endpoint(const std::string& action);
};
