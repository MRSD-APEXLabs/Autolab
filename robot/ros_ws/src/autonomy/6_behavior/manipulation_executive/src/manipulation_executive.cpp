#include <manipulation_executive/manipulation_executive.hpp>

#include <chrono>
#include <functional>
#include <thread>

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

// april_tags.<machine> are free-form keys, so they are declared from the
// parameter file (automatically_declare_parameters_from_overrides).
ManipulationExecutive::ManipulationExecutive()
    : Node("manipulation_executive",
           rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true))
{
    auto param = [this](const std::string& name, const rclcpp::ParameterValue& def) {
        if (!this->has_parameter(name)) this->declare_parameter(name, def);
        return this->get_parameter(name);
    };

    camera_select_service_   = param("camera_select_service",   rclcpp::ParameterValue("/camera_hub/select_zedx")).as_string();
    camera_select_timeout_s_ = param("camera_select_timeout_s", rclcpp::ParameterValue(45.0)).as_double();

    servo_.name            = "Visual servo";
    servo_.node            = "visual_servo_node";
    servo_.start_service   = param("servo_start_service",   rclcpp::ParameterValue("/visual_servo/start")).as_string();
    servo_.stop_service    = param("servo_stop_service",    rclcpp::ParameterValue("/visual_servo/stop")).as_string();
    servo_.status_topic    = param("servo_status_topic",    rclcpp::ParameterValue("/visual_servo/status")).as_string();
    servo_.success_results = param("servo_success_results",
                                   rclcpp::ParameterValue(std::vector<std::string>{"grasped", "aligned", "stopped"}))
                                 .as_string_array();

    act_.name            = "ACT inference";
    act_.node            = "act_inference_node";
    act_start_prefix_    = param("act_start_service_prefix", rclcpp::ParameterValue("/act_inference/start")).as_string();
    act_.stop_service    = param("act_stop_service",    rclcpp::ParameterValue("/act_inference/stop")).as_string();
    act_.status_topic    = param("act_status_topic",    rclcpp::ParameterValue("/act_inference/status")).as_string();
    act_.success_results = param("act_success_results",
                                 rclcpp::ParameterValue(std::vector<std::string>{"done"}))
                               .as_string_array();

    this->get_parameters("april_tags", april_tags_);
    if (april_tags_.empty()) {
        april_tags_ = {{"ot2", 1}, {"shaker", 2}};
        RCLCPP_WARN(this->get_logger(),
                    "No april_tags.* parameters — using defaults (ot2=1, shaker=2)");
    }
    for (const auto& [machine, id] : april_tags_) {
        RCLCPP_INFO(this->get_logger(), "AprilTag map: %s → plan_april_%ld",
                    machine.c_str(), static_cast<long>(id));
    }

    // BT conditions
    place_condition_         = new bt::Condition("Place Commanded",          this);
    pick_base_condition_     = new bt::Condition("Pick Base Commanded",      this);
    pick_condition_          = new bt::Condition("Pick Commanded",           this);
    place_base_condition_    = new bt::Condition("Place Base Commanded",     this);
    conditions_.push_back(place_condition_);
    conditions_.push_back(pick_base_condition_);
    conditions_.push_back(pick_condition_);
    conditions_.push_back(place_base_condition_);

    // BT actions
    place_action_          = new bt::Action("Place Object",      this);
    pick_base_action_      = new bt::Action("Pick Base Object",  this);
    pick_action_           = new bt::Action("Pick Object",       this);
    place_base_action_     = new bt::Action("Place Base Object", this);
    actions_.push_back(place_action_);
    actions_.push_back(pick_base_action_);
    actions_.push_back(pick_action_);
    actions_.push_back(place_base_action_);

    // Phase publisher — monitor with: ros2 topic echo /behavior/manipulation_phase
    phase_pub_ = this->create_publisher<std_msgs::msg::String>("manipulation_phase", 10);

    // Planning command publisher
    planning_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/planning_command", 10);

    // Planning state subscriber — sets result flag read by run_planning() in its async thread.
    // Only SUCCESS and ERROR update the flag; IDLE/PLANNING/EXECUTING are ignored.
    planning_state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/planning_state", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            if      (msg->data == "SUCCESS") planning_result_.store(1);
            else if (msg->data == "ERROR")   planning_result_.store(2);
        });

    // Command subscription
    cmd_sub_ = this->create_subscription<behavior_tree_msgs::msg::ManipulationCommand>(
        "manipulation_command", 10,
        std::bind(&ManipulationExecutive::command_callback, this, std::placeholders::_1));

    // Camera hub, visual servo and ACT inference. Their own callback group, so replies
    // are handled while the timer runs; the async threads below wait on them.
    io_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    camera_select_client_ = this->create_client<Trigger>(
        camera_select_service_, rmw_qos_profile_services_default, io_group_);
    setup_remote(servo_);
    setup_remote(act_);
    // <prefix>/<target>: base for place_base, a lab machine for place
    std::vector<std::string> act_targets{"base"};
    for (const auto& [machine, id] : april_tags_) act_targets.push_back(machine);
    for (const auto& target : act_targets) {
        act_starts_[target] = this->create_client<Trigger>(
            act_start_prefix_ + "/" + target, rmw_qos_profile_services_default, io_group_);
    }

    RCLCPP_INFO(this->get_logger(), "Camera select: %s | servo: %s, %s, %s | ACT: %s/<target>, %s, %s",
                camera_select_service_.c_str(), servo_.start_service.c_str(),
                servo_.stop_service.c_str(), servo_.status_topic.c_str(),
                act_start_prefix_.c_str(), act_.stop_service.c_str(), act_.status_topic.c_str());

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

    const std::map<std::string, bt::Condition*> by_type{
        {"place",      place_condition_},
        {"pick_base",  pick_base_condition_},
        {"pick",       pick_condition_},
        {"place_base", place_base_condition_},
    };
    auto it = by_type.find(msg->type);
    if (it == by_type.end()) {
        RCLCPP_WARN(this->get_logger(), "Unknown manipulation type '%s' — ignoring",
                    msg->type.c_str());
        return;
    }
    for (auto* c : conditions_) c->set(c == it->second);
    RCLCPP_INFO(this->get_logger(), "Received %s command (object: %s, machine: %s)",
                msg->type.c_str(), object_type_.c_str(), target_machine_.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Remote run status — feeds the tracker that run_remote() polls
// ─────────────────────────────────────────────────────────────────────────────

void ManipulationExecutive::setup_remote(RemoteRun& run)
{
    if (!run.start_service.empty()) {  // ACT has one start client per target (act_starts_)
        run.start = this->create_client<Trigger>(
            run.start_service, rmw_qos_profile_services_default, io_group_);
    }
    run.stop = this->create_client<Trigger>(
        run.stop_service, rmw_qos_profile_services_default, io_group_);

    rclcpp::SubscriptionOptions io_options;
    io_options.callback_group = io_group_;
    // visual_servoing and act_inference publish their status transient-local
    run.status_sub = this->create_subscription<std_msgs::msg::String>(
        run.status_topic, rclcpp::QoS(10).reliable().transient_local(),
        [this, &run](const std_msgs::msg::String::SharedPtr msg) { status_callback(run, msg); },
        io_options);
}

void ManipulationExecutive::status_callback(RemoteRun& run, const std_msgs::msg::String::SharedPtr msg)
{
    auto status = manipulation_logic::parse_run_status(msg->data);
    if (!status) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Unparseable %s: '%s'", run.status_topic.c_str(), msg->data.c_str());
        return;
    }
    std::lock_guard<std::mutex> lock(run.mutex);
    run.has_status   = true;
    run.last_started = status->started;
    run.tracker.update(*status);
}

// ─────────────────────────────────────────────────────────────────────────────
// Timer callback — 20 Hz
// ─────────────────────────────────────────────────────────────────────────────

static const char* phase_name(ManipulationExecutive::ManipPhase p)
{
    switch (p) {
        case ManipulationExecutive::ManipPhase::IDLE:             return "IDLE";
        case ManipulationExecutive::ManipPhase::SELECTING_CAMERA: return "SELECTING_CAMERA";
        case ManipulationExecutive::ManipPhase::PLANNING:         return "PLANNING";
        case ManipulationExecutive::ManipPhase::RUNNING_ACT:      return "RUNNING_ACT";
        case ManipulationExecutive::ManipPhase::SERVOING:         return "SERVOING";
        case ManipulationExecutive::ManipPhase::PLANNING_SAFE:    return "PLANNING_SAFE";
        default:                                                   return "UNKNOWN";
    }
}

void ManipulationExecutive::timer_callback()
{
    tick_manip(place_action_,          ManipType::PLACE);
    tick_manip(pick_base_action_,      ManipType::PICK_BASE);
    tick_manip(pick_action_,           ManipType::PICK);
    tick_manip(place_base_action_,     ManipType::PLACE_BASE);

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

void ManipulationExecutive::start_select_camera()
{
    manip_phase_ = ManipPhase::SELECTING_CAMERA;
    in_flight_   = true;
    pending_ = std::async(std::launch::async, &ManipulationExecutive::select_camera, this);
}

bt::Condition* ManipulationExecutive::condition_for(ManipType type)
{
    switch (type) {
        case ManipType::PLACE:      return place_condition_;
        case ManipType::PICK_BASE:  return pick_base_condition_;
        case ManipType::PICK:       return pick_condition_;
        case ManipType::PLACE_BASE: return place_base_condition_;
    }
    return pick_condition_;  // unreachable
}

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
        // A wait left over from a deactivated run sees abort_ and returns
        // within ~100 ms; drain it before clearing the flag for this run.
        if (pending_.valid()) pending_.wait();
        abort_.store(false);
        const char* type_name =
            (type == ManipType::PLACE)      ? "place"      :
            (type == ManipType::PICK_BASE)  ? "pick_base"  :
            (type == ManipType::PICK)       ? "pick"       :
                                              "place_base";
        RCLCPP_INFO(this->get_logger(), "Manipulation activated (%s)", type_name);

        if (type == ManipType::PICK_BASE || type == ManipType::PLACE_BASE) {
            // base: no vision needed for home_offset; the servo / ACT node selects its own camera
            manip_phase_ = ManipPhase::PLANNING;
            in_flight_   = true;
            pending_ = std::async(std::launch::async,
                                  &ManipulationExecutive::run_planning,
                                  this, "home_offset");
        } else {
            // every other plan reads /inspect/* → base camera first
            start_select_camera();
        }
        action->set_running();
        return;
    }

    // ── SELECTING_CAMERA ──────────────────────────────────────────────────────
    if (manip_phase_ == ManipPhase::SELECTING_CAMERA) {
        if (pending_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            in_flight_ = false;
            if (!pending_.get()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to select the base camera — FAILURE");
                fail_manip(action);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Base camera streaming — triggering planning");
            manip_phase_ = ManipPhase::PLANNING;
            in_flight_   = true;
            // place and pick both go to target_machine's AprilTag
            pending_ = std::async(std::launch::async,
                                  &ManipulationExecutive::run_planning,
                                  this, "machine_tag");
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
            if (manip_type_ == ManipType::PLACE || manip_type_ == ManipType::PLACE_BASE) {
                const std::string target =
                    (manip_type_ == ManipType::PLACE_BASE) ? "base" : target_machine_;
                auto start = act_starts_.find(target);
                if (start == act_starts_.end()) {
                    RCLCPP_ERROR(this->get_logger(), "No ACT start service for target '%s' — FAILURE",
                                 target.c_str());
                    fail_manip(action);
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Planning complete — starting ACT inference (%s)",
                            target.c_str());
                manip_phase_ = ManipPhase::RUNNING_ACT;
                in_flight_   = true;
                pending_ = std::async(std::launch::async,
                                      &ManipulationExecutive::run_remote, this, std::ref(act_),
                                      start->second);
            } else {
                RCLCPP_INFO(this->get_logger(), "Planning complete — starting visual servo");
                manip_phase_ = ManipPhase::SERVOING;
                in_flight_   = true;
                pending_ = std::async(std::launch::async,
                                      &ManipulationExecutive::run_remote, this, std::ref(servo_), servo_.start);
            }
        }
        action->set_running();
        return;
    }

    // ── RUNNING_ACT ───────────────────────────────────────────────────────────
    if (manip_phase_ == ManipPhase::RUNNING_ACT) {
        if (pending_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            in_flight_ = false;
            if (!pending_.get()) {
                RCLCPP_ERROR(this->get_logger(), "ACT inference did not succeed — FAILURE");
                fail_manip(action);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "ACT complete — planning safe position");
            manip_phase_ = ManipPhase::PLANNING_SAFE;
            in_flight_   = true;
            pending_ = std::async(std::launch::async,
                                  &ManipulationExecutive::run_planning,
                                  this, "safe");
        }
        action->set_running();
        return;
    }


    // ── SERVOING ──────────────────────────────────────────────────────────────
    if (manip_phase_ == ManipPhase::SERVOING) {
        if (pending_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            in_flight_ = false;
            if (!pending_.get()) {
                RCLCPP_ERROR(this->get_logger(), "Visual servo did not succeed — FAILURE");
                fail_manip(action);
                return;
            }
            if (manip_type_ == ManipType::PICK_BASE) {
                // pick_base ends here — no safe-home planning needed
                RCLCPP_INFO(this->get_logger(), "Servo complete — pick_base SUCCESS");
                pick_base_condition_->set(false);
                manip_phase_    = ManipPhase::IDLE;
                manip_terminal_ = true;
                action->set_success();
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "Servo complete — planning safe position");
                manip_phase_ = ManipPhase::PLANNING_SAFE;
                in_flight_   = true;
                pending_ = std::async(std::launch::async,
                                      &ManipulationExecutive::run_planning,
                                      this, "safe");
            }
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
            condition_for(manip_type_)->set(false);
            manip_phase_    = ManipPhase::IDLE;
            manip_terminal_ = true;
            action->set_success();
            return;
        }
        action->set_running();
        return;
    }
}

void ManipulationExecutive::reset_manip()
{
    // Deactivated by the BT: stop the arm if the servo or ACT is driving it, and
    // let the async waits return so the next activation isn't blocked on them.
    RemoteRun* driving =
        (manip_phase_ == ManipPhase::SERVOING)    ? &servo_ :
        (manip_phase_ == ManipPhase::RUNNING_ACT) ? &act_   : nullptr;
    if (driving && in_flight_) {
        RCLCPP_WARN(this->get_logger(), "Deactivated mid-run — stopping %s", driving->name.c_str());
        send_stop(*driving);
    }
    if (in_flight_) abort_.store(true);

    manip_phase_    = ManipPhase::IDLE;
    manip_terminal_ = false;
    in_flight_      = false;
}

void ManipulationExecutive::fail_manip(bt::Action* action)
{
    condition_for(manip_type_)->set(false);
    manip_phase_    = ManipPhase::IDLE;
    manip_terminal_ = true;
    action->set_failure();
}

// ─────────────────────────────────────────────────────────────────────────────
// call_trigger — blocking std_srvs/Trigger call, for the async threads.
// The reply is handled by the executor (io_group_) while this thread waits.
// ─────────────────────────────────────────────────────────────────────────────

bool ManipulationExecutive::call_trigger(
    const rclcpp::Client<Trigger>::SharedPtr& client, double timeout_s, std::string& message)
{
    if (!client->wait_for_service(2s)) {
        message = std::string(client->get_service_name()) + " is not available";
        return false;
    }
    if (abort_.load()) {
        message = "aborted";
        return false;
    }

    auto request = client->async_send_request(std::make_shared<Trigger::Request>());
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(timeout_s));

    while (request.future.wait_for(100ms) != std::future_status::ready) {
        if (abort_.load() || !rclcpp::ok()) {
            client->remove_pending_request(request.request_id);
            message = "aborted";
            return false;
        }
        if (std::chrono::steady_clock::now() > deadline) {
            client->remove_pending_request(request.request_id);
            message = std::string("no reply from ") + client->get_service_name() +
                      " within " + std::to_string(timeout_s) + " s";
            return false;
        }
    }

    auto response = request.future.get();
    message = response->message;
    return response->success;
}

// ─────────────────────────────────────────────────────────────────────────────
// select_camera — runs in std::async thread
//
// The camera hub streams one camera at a time. The planner's AprilTag and
// wellplate poses (/inspect/*) come from the base ZED X, so it is selected
// before every plan that reads them. The service returns once the camera
// streams (~6 s for the ZED X, immediately if it already is).
// ─────────────────────────────────────────────────────────────────────────────

bool ManipulationExecutive::select_camera()
{
    std::string message;
    bool ok = call_trigger(camera_select_client_, camera_select_timeout_s_, message);
    if (ok) {
        RCLCPP_INFO(this->get_logger(), "%s: %s",
                    camera_select_service_.c_str(), message.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "%s failed: %s",
                     camera_select_service_.c_str(), message.c_str());
    }
    return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
// run_remote — runs in std::async thread
//
// Calls `start` (visual servo, ACT inference for its target), then follows the run's
// status topic until that run ends. Both nodes switch the camera hub to the
// wrist camera themselves. Success only for results in the run's
// success_results (servo: grasped, aligned, stopped; ACT: done by default).
// No timeout is applied — the node is expected to always publish a terminal
// result; the run fails only if the status topic loses its publisher.
// The previous run's `started` (from the transient-local status) identifies
// this run even if it ends before it is ever published running.
// ─────────────────────────────────────────────────────────────────────────────

void ManipulationExecutive::send_stop(RemoteRun& run)
{
    if (run.stop->service_is_ready()) {
        run.stop->async_send_request(std::make_shared<Trigger::Request>());
    }
}

bool ManipulationExecutive::run_remote(RemoteRun& run, rclcpp::Client<Trigger>::SharedPtr start)
{
    {
        std::lock_guard<std::mutex> lock(run.mutex);
        run.tracker = run.has_status ? manipulation_logic::RunTracker(run.last_started)
                                     : manipulation_logic::RunTracker{};
    }

    std::string message;
    if (!call_trigger(start, 5.0, message)) {
        // deactivated while the start request was in flight: reset_manip()'s stop may
        // have reached the node before the start did, so stop again
        if (abort_.load()) {
            send_stop(run);
            return false;
        }
        RCLCPP_ERROR(this->get_logger(),
                     "%s did not start (%s) — is %s running?",
                     run.name.c_str(), message.c_str(), run.node.c_str());
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "%s started: %s", run.name.c_str(), message.c_str());

    while (rclcpp::ok()) {
        std::this_thread::sleep_for(100ms);

        {
            std::lock_guard<std::mutex> lock(run.mutex);
            if (run.tracker.finished()) {
                const std::string& result = run.tracker.result();
                bool ok = manipulation_logic::run_succeeded(result, run.success_results);
                if (ok) {
                    RCLCPP_INFO(this->get_logger(), "%s finished: %s (%s)", run.name.c_str(),
                                result.c_str(), run.tracker.message().c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "%s finished: %s (%s)", run.name.c_str(),
                                 result.empty() ? "<no result>" : result.c_str(),
                                 run.tracker.message().c_str());
                }
                return ok;
            }
        }

        if (abort_.load()) {   // reset_manip() sent stop; again, in case it beat the start
            send_stop(run);
            return false;
        }

        if (run.status_sub->get_publisher_count() == 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "%s has no publisher — %s is gone, failing",
                         run.status_topic.c_str(), run.node.c_str());
            return false;
        }
    }
    return false;  // rclcpp shutdown
}

// ─────────────────────────────────────────────────────────────────────────────
// run_planning — runs in std::async thread
//
// Publishes to /planning_command and waits for /planning_state → SUCCESS or ERROR.
// The planner publishes PLANNING then EXECUTING as intermediate states, and either
// SUCCESS or ERROR as terminal states. No timeout is applied — the planner is
// expected to always emit a terminal state.
// pose_type: "machine_tag" | "safe" | "home_offset"
// ─────────────────────────────────────────────────────────────────────────────

bool ManipulationExecutive::run_planning(const std::string& pose_type)
{
    // Capture shared members on the calling thread before the async thread reads them
    const std::string object_type    = object_type_;
    const std::string target_machine = target_machine_;

    // ── Map pose_type + object_type → planning command ───────────────────────
    std::string cmd;
    if (pose_type == "machine_tag") {
        // target_machine → AprilTag id, from the april_tags.* parameters
        auto april_cmd = manipulation_logic::april_command(april_tags_, target_machine);
        if (!april_cmd) {
            RCLCPP_ERROR(this->get_logger(),
                         "run_planning: target_machine '%s' has no AprilTag (configured: %s) — aborting",
                         target_machine.c_str(),
                         manipulation_logic::machine_names(april_tags_).c_str());
            return false;
        }
        cmd = *april_cmd;
    } else if (pose_type == "safe") {
        cmd = "plan_home_offset";
    } else if (pose_type == "home_offset") {
        cmd = "plan_home_offset";
    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "run_planning: unknown pose_type '%s' — aborting",
                     pose_type.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Planning: pose_type=%s object=%s target=%s → /planning_command=%s",
                pose_type.c_str(), object_type.c_str(), target_machine.c_str(), cmd.c_str());

    // ── Reset result before publishing (prevents stale result from prior plan) ─
    planning_result_.store(0);

    // ── Publish command ───────────────────────────────────────────────────────
    std_msgs::msg::String cmd_msg;
    cmd_msg.data = cmd;
    planning_cmd_pub_->publish(cmd_msg);

    // ── Wait for terminal state (SUCCESS=1 or ERROR=2) ────────────────────────
    // The planner publishes PLANNING → EXECUTING → SUCCESS|ERROR. We wait here
    // with no timeout; the planner always emits a terminal state. A BT
    // deactivation (abort_) stops the wait, not the plan.
    constexpr int POLL_MS = 100;
    while (rclcpp::ok() && !abort_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(POLL_MS));
        int result = planning_result_.load();
        if (result == 1) return true;   // SUCCESS
        if (result == 2) return false;  // ERROR
    }

    return false;  // aborted or rclcpp shutdown
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
