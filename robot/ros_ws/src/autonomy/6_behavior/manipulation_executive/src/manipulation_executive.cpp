#include <manipulation_executive/manipulation_executive.hpp>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <thread>
#include <chrono>

using namespace std::chrono_literals;

namespace beast     = boost::beast;
namespace websocket = beast::websocket;
namespace net       = boost::asio;
using     tcp       = net::ip::tcp;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

ManipulationExecutive::ManipulationExecutive()
    : Node("manipulation_executive")
{
    this->declare_parameter<std::string>("camera_edge_host",       "localhost");
    this->declare_parameter<int>        ("camera_edge_port",       8765);

    this->get_parameter("camera_edge_host",       camera_edge_host_);
    this->get_parameter("camera_edge_port",       camera_edge_port_);

    RCLCPP_INFO(this->get_logger(), "Camera-edge: %s:%d",
                camera_edge_host_.c_str(), camera_edge_port_);

    // BT conditions
    pick_up_condition_ = new bt::Condition("Pick Up Commanded", this);
    place_condition_   = new bt::Condition("Place Commanded",   this);
    conditions_.push_back(pick_up_condition_);
    conditions_.push_back(place_condition_);

    // BT actions
    pick_up_action_ = new bt::Action("Pick Up Object", this);
    place_action_   = new bt::Action("Place Object",   this);
    actions_.push_back(pick_up_action_);
    actions_.push_back(place_action_);

    // Phase publisher — monitor with: ros2 topic echo /behavior/manipulation_phase
    phase_pub_ = this->create_publisher<std_msgs::msg::String>("manipulation_phase", 10);

    // Planning command publisher
    planning_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/planning_command", 10);

    // Planning done subscriber — sets flag read by run_planning() in its async thread
    planning_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/planning_done", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            planning_done_flag_ = msg->data ? 1 : 0;
        });

    // Command subscription
    cmd_sub_ = this->create_subscription<behavior_tree_msgs::msg::ManipulationCommand>(
        "manipulation_command", 10,
        std::bind(&ManipulationExecutive::command_callback, this, std::placeholders::_1));

    // 20 Hz timer
    timer_ = rclcpp::create_timer(
        this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / 20.0),
        std::bind(&ManipulationExecutive::timer_callback, this));
}

// ─────────────────────────────────────────────────────────────────────────────
// Command callback — arms the BT conditions so the tree activates the action
// ─────────────────────────────────────────────────────────────────────────────

void ManipulationExecutive::command_callback(
    const behavior_tree_msgs::msg::ManipulationCommand::SharedPtr msg)
{
    if (manip_phase_ != ManipPhase::IDLE) {
        RCLCPP_WARN(this->get_logger(),
                    "Manipulation in progress — ignoring new command");
        return;
    }

    object_type_    = msg->object_type;
    target_machine_ = msg->target_machine;

    if (msg->type == "pick_up") {
        pick_up_condition_->set(true);
        place_condition_->set(false);
        RCLCPP_INFO(this->get_logger(), "Received pick_up command (object: %s)",
                    object_type_.c_str());
    } else if (msg->type == "place") {
        place_condition_->set(true);
        pick_up_condition_->set(false);
        RCLCPP_INFO(this->get_logger(), "Received place command (object: %s → machine: %s)",
                    object_type_.c_str(), target_machine_.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown manipulation type '%s' — ignoring",
                    msg->type.c_str());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Timer callback — 20 Hz
// ─────────────────────────────────────────────────────────────────────────────

static const char* phase_name(ManipulationExecutive::ManipPhase p)
{
    switch (p) {
        case ManipulationExecutive::ManipPhase::IDLE:               return "IDLE";
        case ManipulationExecutive::ManipPhase::ACTIVATING_INSPECT: return "ACTIVATING_INSPECT";
        case ManipulationExecutive::ManipPhase::PLANNING:           return "PLANNING";
        case ManipulationExecutive::ManipPhase::ACTIVATING_SERVO:   return "ACTIVATING_SERVO";
        case ManipulationExecutive::ManipPhase::PLANNING_SAFE:      return "PLANNING_SAFE";
        default:                                                     return "UNKNOWN";
    }
}

void ManipulationExecutive::timer_callback()
{
    tick_manip(pick_up_action_, ManipType::PICK_UP);
    tick_manip(place_action_,   ManipType::PLACE);

    // Publish current phase for monitoring
    std_msgs::msg::String phase_msg;
    phase_msg.data = phase_name(manip_phase_);
    phase_pub_->publish(phase_msg);

    for (auto* c : conditions_) c->publish();
    for (auto* a : actions_)    a->publish();
}

// ─────────────────────────────────────────────────────────────────────────────
// Shared manipulation state machine
// ─────────────────────────────────────────────────────────────────────────────

void ManipulationExecutive::tick_manip(bt::Action* action, ManipType type)
{
    if (!action->is_active()) {
        if (action->active_has_changed()) reset_manip();
        return;
    }

    if (manip_terminal_) return;

    // ── New activation ────────────────────────────────────────────────────────
    if (action->active_has_changed()) {
        manip_type_ = type;
        RCLCPP_INFO(this->get_logger(), "Manipulation activated (%s)",
                    type == ManipType::PICK_UP ? "pick_up" : "place");

        manip_phase_ = ManipPhase::ACTIVATING_INSPECT;
        in_flight_   = true;
        pending_ = std::async(std::launch::async,
                              &ManipulationExecutive::activate_camera_mode,
                              this, "inspect", false);
        action->set_running();
        return;
    }

    // ── ACTIVATING_INSPECT ────────────────────────────────────────────────────
    if (manip_phase_ == ManipPhase::ACTIVATING_INSPECT) {
        if (pending_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            in_flight_ = false;
            if (!pending_.get()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to activate inspect mode — FAILURE");
                fail_manip(action);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Inspect mode active — triggering planning");
            manip_phase_ = ManipPhase::PLANNING;
            in_flight_   = true;
            // pick_up → inspection pose; place → placement pose
            std::string pose_type = (manip_type_ == ManipType::PICK_UP) ? "inspection" : "placement";
            pending_ = std::async(std::launch::async,
                                  &ManipulationExecutive::run_planning,
                                  this, pose_type);
        }
        action->set_running();
        return;
    }

    // ── PLANNING ──────────────────────────────────────────────────────────────
    if (manip_phase_ == ManipPhase::PLANNING) {
        if (pending_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            in_flight_ = false;
            if (!pending_.get()) {
                RCLCPP_ERROR(this->get_logger(), "Planning failed — FAILURE");
                fail_manip(action);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Planning complete — activating servo mode");
            manip_phase_ = ManipPhase::ACTIVATING_SERVO;
            in_flight_   = true;
            pending_ = std::async(std::launch::async,
                                  &ManipulationExecutive::activate_camera_mode,
                                  this, "servo", true);
        }
        action->set_running();
        return;
    }

    // ── ACTIVATING_SERVO ──────────────────────────────────────────────────────
    if (manip_phase_ == ManipPhase::ACTIVATING_SERVO) {
        if (pending_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            in_flight_ = false;
            if (!pending_.get()) {
                RCLCPP_ERROR(this->get_logger(), "Servo mode did not complete — FAILURE");
                fail_manip(action);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Servo complete — planning safe position");
            manip_phase_ = ManipPhase::PLANNING_SAFE;
            in_flight_   = true;
            pending_ = std::async(std::launch::async,
                                  &ManipulationExecutive::run_planning,
                                  this, "safe");
        }
        action->set_running();
        return;
    }

    // ── PLANNING_SAFE ─────────────────────────────────────────────────────────
    if (manip_phase_ == ManipPhase::PLANNING_SAFE) {
        if (pending_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            in_flight_ = false;
            if (!pending_.get()) {
                RCLCPP_ERROR(this->get_logger(), "Safe planning failed — FAILURE");
                fail_manip(action);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Manipulation complete — SUCCESS");
            bt::Condition* cond = (manip_type_ == ManipType::PICK_UP)
                                  ? pick_up_condition_ : place_condition_;
            cond->set(false);
            manip_phase_    = ManipPhase::IDLE;
            manip_terminal_ = true;
            action->set_success();
        }
        action->set_running();
        return;
    }
}

void ManipulationExecutive::reset_manip()
{
    manip_phase_    = ManipPhase::IDLE;
    manip_terminal_ = false;
    in_flight_      = false;
}

void ManipulationExecutive::fail_manip(bt::Action* action)
{
    bt::Condition* cond = (manip_type_ == ManipType::PICK_UP)
                          ? pick_up_condition_ : place_condition_;
    cond->set(false);
    manip_phase_    = ManipPhase::IDLE;
    manip_terminal_ = true;
    action->set_failure();
}

// ─────────────────────────────────────────────────────────────────────────────
// Camera-edge WebSocket helper
// ─────────────────────────────────────────────────────────────────────────────

std::string ManipulationExecutive::ws_round_trip(const std::string& msg)
{
    try {
        net::io_context ioc;
        tcp::resolver resolver{ioc};
        websocket::stream<tcp::socket> ws{ioc};

        auto const results = resolver.resolve(camera_edge_host_,
                                              std::to_string(camera_edge_port_));
        net::connect(ws.next_layer(), results.begin(), results.end());
        ws.handshake(camera_edge_host_, "/");

        // Server sends a greeting on connect ("connected") — drain it before sending
        beast::flat_buffer greeting;
        ws.read(greeting);

        ws.write(net::buffer(msg));

        beast::flat_buffer buffer;
        ws.read(buffer);
        ws.close(websocket::close_code::normal);

        return beast::buffers_to_string(buffer.data());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "WebSocket error: %s", e.what());
        return "";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// activate_camera_mode — runs in std::async thread
//
// Sends { "cmd": "mode", "mode": <mode> }, then polls { "cmd": "status" }.
//
// Response format: { "mode": "<current_mode>", "worker_alive": <bool> }
//   worker_alive == true  AND mode == sent_mode  →  mode is running (active)
//   worker_alive == false OR  mode == "idle"     →  not running (failed/idle)
//
// wait_complete == false (inspect):
//   Return true once worker_alive==true && mode==sent_mode (confirmed active).
//   Return false immediately if worker_alive==false (failed to start).
//
// wait_complete == true (servo):
//   First confirm activation (worker_alive==true), then wait until the mode
//   returns to idle (worker_alive==false), which signals servo has finished.
// ─────────────────────────────────────────────────────────────────────────────

bool ManipulationExecutive::activate_camera_mode(const std::string& mode,
                                                  bool wait_complete)
{
    nlohmann::json set_cmd    = {{"cmd", "mode"},   {"mode", mode}};
    nlohmann::json status_cmd = {{"cmd", "status"}};

    constexpr int MAX_POLLS        = 60;   // 60 × 500 ms = 30 s timeout
    constexpr int POLL_INTERVAL_MS = 500;

    ws_round_trip(set_cmd.dump());  // fire mode command; confirm via status polls

    bool confirmed_active = false;

    for (int i = 0; i < MAX_POLLS; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(POLL_INTERVAL_MS));

        std::string resp = ws_round_trip(status_cmd.dump());

        RCLCPP_INFO(this->get_logger(),
                    "[camera-edge status] poll %d/%d  raw='%s'",
                    i + 1, MAX_POLLS, resp.empty() ? "<empty>" : resp.c_str());

        if (resp.empty()) continue;

        try {
            auto doc = nlohmann::json::parse(resp);
            std::string resp_mode  = doc.value("mode",         "");
            bool        resp_alive = doc.value("worker_alive", false);

            RCLCPP_INFO(this->get_logger(),
                        "[camera-edge status] mode='%s' worker_alive=%s  (want mode='%s' wait_complete=%d confirmed_active=%d)",
                        resp_mode.c_str(), resp_alive ? "true" : "false",
                        mode.c_str(), wait_complete, confirmed_active);

            if (!wait_complete) {
                // Inspect: succeed once active; keep polling if not yet active
                if (resp_mode == mode && resp_alive) return true;
            } else {
                // Servo: wait for activation then completion (returns to idle)
                if (!confirmed_active) {
                    confirmed_active = (resp_mode == mode && resp_alive);
                } else if (!resp_alive || resp_mode == "idle") {
                    return true;  // servo finished
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(),
                        "[camera-edge status] failed to parse response: %s  raw='%s'",
                        e.what(), resp.c_str());
        }
    }

    RCLCPP_ERROR(this->get_logger(),
                 "Timed out waiting for camera-edge mode='%s' (wait_complete=%d)",
                 mode.c_str(), wait_complete);
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// run_planning — runs in std::async thread
//
// Publishes to /planning_command and polls /planning_done (via planning_done_flag_).
// pose_type: "inspection" | "placement" | "safe"
// ─────────────────────────────────────────────────────────────────────────────

bool ManipulationExecutive::run_planning(const std::string& pose_type)
{
    // Capture shared members on the calling thread before the async thread reads them
    const std::string object_type    = object_type_;
    const std::string target_machine = target_machine_;

    // ── Map pose_type + object_type → planning command ───────────────────────
    std::string cmd;
    if (pose_type == "inspection") {
        if (object_type == "well_plate") {
            cmd = "plan_wellplate";
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "run_planning: unsupported object_type '%s' for inspection — aborting",
                         object_type.c_str());
            return false;
        }
    } else if (pose_type == "placement") {
        cmd = "plan_april";
    } else if (pose_type == "safe") {
        cmd = "plan_home";
    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "run_planning: unknown pose_type '%s' — aborting",
                     pose_type.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Planning: pose_type=%s object=%s target=%s → /planning_command=%s",
                pose_type.c_str(), object_type.c_str(), target_machine.c_str(), cmd.c_str());

    // ── Reset flag before publishing (prevents stale result from prior plan) ──
    planning_done_flag_ = -1;

    // ── Publish command ───────────────────────────────────────────────────────
    std_msgs::msg::String cmd_msg;
    cmd_msg.data = cmd;
    planning_cmd_pub_->publish(cmd_msg);

    // ── Poll for result (100 ms intervals, 60 s timeout) ─────────────────────
    constexpr int TIMEOUT_MS = 60000;
    constexpr int POLL_MS    = 100;
    for (int elapsed = 0; elapsed < TIMEOUT_MS; elapsed += POLL_MS) {
        std::this_thread::sleep_for(std::chrono::milliseconds(POLL_MS));
        int flag = planning_done_flag_.load();
        if (flag != -1) {
            return flag == 1;
        }
    }

    RCLCPP_ERROR(this->get_logger(),
                 "run_planning: 60 s timeout waiting for /planning_done (pose_type=%s)",
                 pose_type.c_str());
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulationExecutive>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
