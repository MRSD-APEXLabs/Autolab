// Copyright (c) 2024 Carnegie Mellon University
// MIT License — see LICENSE for details.

#include <lab_machine_executive/lab_machine_executive.hpp>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

LabMachineExecutive::LabMachineExecutive()
    : Node("lab_machine_executive"),
      ot2_current_step_(0),
      ot2_phase_(OT2Phase::IDLE),
      ot2_terminal_(false),
      http_in_flight_(false)
{
    // ROS2 parameters
    this->declare_parameter<std::string>("ot2_base_url",     "http://192.168.1.100:8000");
    this->declare_parameter<std::string>("ot2_host",         "192.168.1.100");
    this->declare_parameter<std::string>("shaker_base_url",  "http://192.168.1.101:8080");
    this->declare_parameter<std::string>("shaker_endpoint",  "/pwm");

    this->get_parameter("ot2_base_url",    ot2_base_url_);
    this->get_parameter("ot2_host",        ot2_host_);
    this->get_parameter("shaker_base_url", shaker_base_url_);
    this->get_parameter("shaker_endpoint", shaker_endpoint_);

    RCLCPP_INFO(this->get_logger(), "OT-2 base URL: %s (robot host: %s)",
                ot2_base_url_.c_str(), ot2_host_.c_str());
    RCLCPP_INFO(this->get_logger(), "Shaker base URL: %s%s",
                shaker_base_url_.c_str(), shaker_endpoint_.c_str());

    // BT conditions
    command_lab_machine_condition_ = new bt::Condition("Command Lab Machine", this);
    ot2_condition_                 = new bt::Condition("OT2 Liquid Handler", this);
    shaker_condition_              = new bt::Condition("Custom Shaker Module", this);
    conditions_.push_back(command_lab_machine_condition_);
    conditions_.push_back(ot2_condition_);
    conditions_.push_back(shaker_condition_);

    // BT actions
    ot2_action_    = new bt::Action("Execute OT2 Protocol", this);
    shaker_action_ = new bt::Action("Execute Shaker Protocol", this);
    actions_.push_back(ot2_action_);
    actions_.push_back(shaker_action_);

    // Subscription for incoming lab machine commands
    cmd_sub_ = this->create_subscription<behavior_tree_msgs::msg::LabMachineCommand>(
        "lab_machine_command", 10,
        std::bind(&LabMachineExecutive::command_callback, this, std::placeholders::_1));

    // Timer at 20 Hz to match behavior_executive cadence
    timer_ = rclcpp::create_timer(
        this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / 20.0),
        std::bind(&LabMachineExecutive::timer_callback, this));
}

// ─────────────────────────────────────────────────────────────────────────────
// LabMachineCommand subscription callback
// ─────────────────────────────────────────────────────────────────────────────

void LabMachineExecutive::command_callback(
    const behavior_tree_msgs::msg::LabMachineCommand::SharedPtr msg)
{
    if (msg->device == "ot2") {
        if (ot2_phase_ != OT2Phase::IDLE) {
            RCLCPP_WARN(this->get_logger(),
                        "OT-2 run in progress — ignoring new protocol");
            return;
        }
        ot2_protocol_json_ = msg->parameters_json;
        command_lab_machine_condition_->set(true);
        ot2_condition_->set(true);
        shaker_condition_->set(false);
        RCLCPP_INFO(this->get_logger(), "Received OT-2 protocol (%zu bytes)",
                    ot2_protocol_json_.size());
    } else if (msg->device == "shaker") {
        if (ot2_phase_ != OT2Phase::IDLE) {

            RCLCPP_WARN(this->get_logger(),
                        "OT-2 run in progress — ignoring shaker command");
            return;
        }
        if (shaker_condition_->get()) {
            RCLCPP_WARN(this->get_logger(),
                        "Shaker run in progress — ignoring new command");
            return;
        }
        shaker_protocol_json_ = msg->parameters_json;
        command_lab_machine_condition_->set(true);
        shaker_condition_->set(true);
        ot2_condition_->set(false);
        RCLCPP_INFO(this->get_logger(), "Received shaker protocol (%zu bytes)",
                    shaker_protocol_json_.size());
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown device '%s' — ignoring", msg->device.c_str());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Static helpers
// ─────────────────────────────────────────────────────────────────────────────

// Discard HTTP response body — required by libcurl
static size_t discard_response(char*, size_t size, size_t nmemb, void*) {
    return size * nmemb;
}

bool LabMachineExecutive::http_post(const std::string& url,
                                    const std::string& body,
                                    const std::string& content_type)
{
    CURL* curl = curl_easy_init();
    if (!curl) return false;

    struct curl_slist* headers = nullptr;
    std::string ct_header = "Content-Type: " + content_type;
    headers = curl_slist_append(headers, ct_header.c_str());

    curl_easy_setopt(curl, CURLOPT_URL,           url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST,           1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS,     body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE,  static_cast<long>(body.size()));
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER,     headers);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT,        30L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,  discard_response);

    CURLcode res  = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    return (res == CURLE_OK) && (http_code >= 200) && (http_code < 300);
}

bool LabMachineExecutive::home_ot2() {
    return http_post(ot2_base_url_ + "/robot/home", "{}", "application/json");
}

bool LabMachineExecutive::disconnect_ot2() {
    return http_post(ot2_base_url_ + "/robot/disconnect", "{}", "application/json");
}

// Converts OT-2 step action name to REST endpoint path.
// e.g. "pick_up_tips" → "/commands/pick-up-tips"
std::string LabMachineExecutive::action_to_endpoint(const std::string& action) {
    std::string slug = action;
    std::replace(slug.begin(), slug.end(), '_', '-');
    return "/commands/" + slug;
}

// POST /robot/connect {"host": ot2_host_}
// Returns true on 200 (connected) or 400 (already connected).
bool LabMachineExecutive::connect_ot2() {
    nlohmann::json body = {{"host", ot2_host_}};
    std::string body_str = body.dump();

    CURL* curl = curl_easy_init();
    if (!curl) return false;

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    std::string url = ot2_base_url_ + "/robot/connect";
    curl_easy_setopt(curl, CURLOPT_URL,           url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST,           1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS,     body_str.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE,  static_cast<long>(body_str.size()));
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER,     headers);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT,        60L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,  discard_response);

    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    // 200 = connected, 400 = already connected (both are fine)
    return (res == CURLE_OK) && (http_code == 200 || http_code == 400);
}

// ─────────────────────────────────────────────────────────────────────────────
// Timer callback — runs at 20 Hz
// ─────────────────────────────────────────────────────────────────────────────

void LabMachineExecutive::timer_callback() {
    tick_ot2();
    tick_shaker();

    for (auto* c : conditions_) c->publish();
    for (auto* a : actions_)    a->publish();
}

// ─────────────────────────────────────────────────────────────────────────────
// OT-2 step machine
// ─────────────────────────────────────────────────────────────────────────────

void LabMachineExecutive::tick_ot2() {
    if (!ot2_action_->is_active()) {
        if (ot2_action_->active_has_changed()) {
            ot2_current_step_ = 0;
            ot2_phase_        = OT2Phase::IDLE;
            ot2_terminal_     = false;
            http_in_flight_   = false;
        }
        return;
    }

    if (ot2_terminal_) return;

    // ── New activation: parse protocol then connect ───────────────────────────
    if (ot2_action_->active_has_changed()) {
        if (ot2_phase_ != OT2Phase::IDLE) {
            // BT re-activated mid-run — ignore, keep running
            ot2_action_->set_running();
            return;
        }
        if (ot2_protocol_json_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "OT-2 action activated but no protocol received — FAILURE");
            command_lab_machine_condition_->set(false);
            ot2_condition_->set(false);
            ot2_terminal_ = true;
            ot2_action_->set_failure();
            return;
        }
        try {
            auto doc   = nlohmann::json::parse(ot2_protocol_json_);
            ot2_steps_ = doc.at("steps");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Malformed OT-2 protocol JSON: %s", e.what());
            command_lab_machine_condition_->set(false);
            ot2_condition_->set(false);
            ot2_terminal_ = true;
            ot2_action_->set_failure();
            return;
        }
        ot2_current_step_ = 0;
        ot2_phase_        = OT2Phase::CONNECTING;
        http_in_flight_   = true;
        RCLCPP_INFO(this->get_logger(), "OT-2 connecting to %s…", ot2_host_.c_str());
        pending_http_ = std::async(std::launch::async,
                                   &LabMachineExecutive::connect_ot2, this);
        ot2_action_->set_running();
        return;
    }

    // ── CONNECTING ────────────────────────────────────────────────────────────
    if (ot2_phase_ == OT2Phase::CONNECTING) {
        if (pending_http_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            bool ok = pending_http_.get();
            http_in_flight_ = false;
            if (!ok) {
                RCLCPP_ERROR(this->get_logger(), "OT-2 connect failed — FAILURE");
                command_lab_machine_condition_->set(false);
                ot2_condition_->set(false);
                ot2_terminal_ = true;
                ot2_action_->set_failure();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "OT-2 connected — starting %zu steps",
                        ot2_steps_.size());
            ot2_phase_ = OT2Phase::STEPPING;
        }
        ot2_action_->set_running();
        return;
    }

    // ── STEPPING ──────────────────────────────────────────────────────────────
    if (ot2_phase_ == OT2Phase::STEPPING) {
        if (!http_in_flight_) {
            if (ot2_current_step_ >= static_cast<int>(ot2_steps_.size())) {
                RCLCPP_INFO(this->get_logger(), "OT-2 steps complete — homing…");
                ot2_phase_      = OT2Phase::HOMING;
                http_in_flight_ = true;
                pending_http_   = std::async(std::launch::async,
                                             &LabMachineExecutive::home_ot2, this);
                ot2_action_->set_running();
                return;
            }
            const auto& step = ot2_steps_[ot2_current_step_];
            RCLCPP_INFO(this->get_logger(), "OT-2 step %d: %s",
                        ot2_current_step_, step.value("action", "?").c_str());
            pending_http_   = std::async(std::launch::async,
                                         &LabMachineExecutive::dispatch_ot2_step,
                                         this, step);
            http_in_flight_ = true;
            ot2_action_->set_running();
            return;
        }
        if (pending_http_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            bool ok = pending_http_.get();
            http_in_flight_ = false;
            if (!ok) {
                RCLCPP_ERROR(this->get_logger(),
                             "OT-2 step %d failed — FAILURE", ot2_current_step_);
                command_lab_machine_condition_->set(false);
                ot2_condition_->set(false);
                ot2_terminal_ = true;
                ot2_action_->set_failure();
                return;
            }
            ot2_current_step_++;
        }
        ot2_action_->set_running();
        return;
    }

    // ── HOMING ────────────────────────────────────────────────────────────────
    if (ot2_phase_ == OT2Phase::HOMING) {
        if (pending_http_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            bool ok = pending_http_.get();
            http_in_flight_ = false;
            if (!ok) {
                RCLCPP_ERROR(this->get_logger(), "OT-2 home failed — FAILURE");
                command_lab_machine_condition_->set(false);
                ot2_condition_->set(false);
                ot2_terminal_ = true;
                ot2_action_->set_failure();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "OT-2 homed — disconnecting…");
            ot2_phase_      = OT2Phase::DISCONNECTING;
            http_in_flight_ = true;
            pending_http_   = std::async(std::launch::async,
                                         &LabMachineExecutive::disconnect_ot2, this);
        }
        ot2_action_->set_running();
        return;
    }

    // ── DISCONNECTING ─────────────────────────────────────────────────────────
    if (ot2_phase_ == OT2Phase::DISCONNECTING) {
        if (pending_http_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            bool ok = pending_http_.get();
            http_in_flight_ = false;
            if (!ok) {
                RCLCPP_ERROR(this->get_logger(), "OT-2 disconnect failed — FAILURE");
                command_lab_machine_condition_->set(false);
                ot2_condition_->set(false);
                ot2_terminal_ = true;
                ot2_action_->set_failure();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "OT-2 disconnected — SUCCESS");
            command_lab_machine_condition_->set(false);
            ot2_condition_->set(false);
            ot2_terminal_ = true;
            ot2_action_->set_success();
        }
        ot2_action_->set_running();
        return;
    }
}

bool LabMachineExecutive::dispatch_ot2_step(const nlohmann::json& step) {
    std::string action = step.value("action", "");
    if (action.empty()) return false;

    // Build body: step object minus the "action" key
    nlohmann::json body = step;
    body.erase("action");

    std::string url      = ot2_base_url_ + action_to_endpoint(action);
    std::string body_str = body.dump();

    return http_post(url, body_str, "application/json");
}

// ─────────────────────────────────────────────────────────────────────────────
// Shaker action (single HTTP call)
// ─────────────────────────────────────────────────────────────────────────────

void LabMachineExecutive::tick_shaker() {
    if (!shaker_action_->is_active()) {
        if (shaker_action_->active_has_changed()) {
            http_in_flight_ = false;
        }
        return;
    }

    if (shaker_action_->active_has_changed()) {
        // New activation — fire single HTTP call asynchronously
        if (shaker_protocol_json_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "Shaker action activated but no command received — FAILURE");
            shaker_action_->set_failure();
            return;
        }

        int pwm = 0;
        try {
            auto doc = nlohmann::json::parse(shaker_protocol_json_);
            pwm = doc.at("pwm").get<int>();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Malformed shaker JSON: %s", e.what());
            shaker_action_->set_failure();
            return;
        }

        std::string url  = shaker_base_url_ + shaker_endpoint_;
        std::string body = std::to_string(pwm);

        RCLCPP_INFO(this->get_logger(), "Shaker: POST %s body=%s", url.c_str(), body.c_str());

        // Fire-and-report: single blocking call in async thread, check next tick
        pending_http_   = std::async(std::launch::async,
                                     &LabMachineExecutive::http_post,
                                     url, body, std::string("text/plain"));
        http_in_flight_ = true;
        shaker_action_->set_running();
        return;
    }

    // Poll future
    if (http_in_flight_ &&
        pending_http_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
        bool ok = pending_http_.get();
        http_in_flight_ = false;

        if (ok) {
            RCLCPP_INFO(this->get_logger(), "Shaker command succeeded — SUCCESS");
            command_lab_machine_condition_->set(false);
            shaker_condition_->set(false);
            shaker_action_->set_success();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Shaker command failed — FAILURE");
            command_lab_machine_condition_->set(false);
            shaker_condition_->set(false);
            shaker_action_->set_failure();
        }
        return;
    }

    shaker_action_->set_running();
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LabMachineExecutive>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
