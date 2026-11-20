// Copyright (c) 2024 Carnegie Mellon University
// MIT License — see LICENSE for details.

#pragma once

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <future>
#include <string>
#include <vector>

#include <behavior_tree/behavior_tree.hpp>
#include <lab_machine_executive/tip_check.hpp>
#include <behavior_tree_msgs/msg/lab_machine_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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
    std::string ot2_run_id_;       // names this protocol run's tip-check results

    // ── ROS2 params ───────────────────────────────────────────────────────────
    std::string ot2_base_url_;
    std::string ot2_host_;         // OT-2 robot IP passed to /robot/connect
    std::string shaker_base_url_;
    std::string shaker_endpoint_;  // e.g. "/pwm"
    lab_machine::TipCheckConfig tip_check_cfg_;  // d3:4b tip check after each aspirate

    // ── ROS2 infrastructure ───────────────────────────────────────────────────
    rclcpp::Subscription<behavior_tree_msgs::msg::LabMachineCommand>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    // One tip-check result (JSON) per aspirate; routine_executor attaches it to the run
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tip_check_pub_;
    // Live OT-2 progress (JSON, lab_machine::ot2_progress): command, tip-check pass, phase
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ot2_progress_pub_;

    // ── Callbacks ────────────────────────────────────────────────────────────
    void command_callback(const behavior_tree_msgs::msg::LabMachineCommand::SharedPtr msg);
    void timer_callback();

    // ── Helpers ───────────────────────────────────────────────────────────────
    void tick_ot2();
    void tick_shaker();
    bool dispatch_ot2_step(const nlohmann::json& step, int index);
    nlohmann::json run_tip_check(const std::string& label, int index);
    // Thread-safe (called from the timer and from the HTTP worker during a tip check)
    void publish_ot2_progress(const std::string& phase, int command = -1,
                              const nlohmann::json& tip_check = nlohmann::json());

    // Static HTTP helper — runs in std::async thread
    bool connect_ot2();
    bool home_ot2();
    bool disconnect_ot2();
    static bool http_post(const std::string& url,
                          const std::string& body,
                          const std::string& content_type);
    // Same as http_post, but stores the response body in *response when non-null.
    static bool http_post_capture(const std::string& url,
                                  const std::string& body,
                                  const std::string& content_type,
                                  std::string* response);
    static std::string action_to_endpoint(const std::string& action);
};
