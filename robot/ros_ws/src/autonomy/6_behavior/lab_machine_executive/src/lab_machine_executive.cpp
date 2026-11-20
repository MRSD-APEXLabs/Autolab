// Copyright (c) 2024 Carnegie Mellon University
// MIT License — see LICENSE for details.

#include <lab_machine_executive/lab_machine_executive.hpp>

#include <algorithm>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace {

// libcurl client for the tip-check service (JSON in, JSON out).
class CurlHttp : public lab_machine::HttpClient {
   public:
    explicit CurlHttp(long timeout_s) : timeout_s_(timeout_s) {}
    lab_machine::HttpResponse post(const std::string& url, const std::string& body) override {
        return request(url, &body);
    }
    lab_machine::HttpResponse get(const std::string& url) override { return request(url, nullptr); }

   private:
    static size_t collect(char* ptr, size_t size, size_t nmemb, void* userdata) {
        static_cast<std::string*>(userdata)->append(ptr, size * nmemb);
        return size * nmemb;
    }
    lab_machine::HttpResponse request(const std::string& url, const std::string* body) {
        lab_machine::HttpResponse r{false, 0, "", ""};
        CURL* curl = curl_easy_init();
        if (!curl) { r.error = "curl init failed"; return r; }
        struct curl_slist* headers = curl_slist_append(nullptr, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        if (body) {
            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body->c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body->size()));
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        }
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_s_);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, collect);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &r.body);
        CURLcode res = curl_easy_perform(curl);
        if (res == CURLE_OK) {
            r.sent = true;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &r.code);
        } else {
            r.error = curl_easy_strerror(res);
        }
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        return r;
    }
    long timeout_s_;
};

}  // namespace

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
    // d3:4b tip-check service, run after each aspirate. Empty disables the check.
    this->declare_parameter<std::string>("tip_check_base_url", "");
    this->declare_parameter<int>("tip_check_passes", 5);
    this->declare_parameter<double>("tip_check_timeout_s", 900.0);

    this->get_parameter("ot2_base_url",     ot2_base_url_);
    this->get_parameter("ot2_host",         ot2_host_);
    this->get_parameter("shaker_base_url",  shaker_base_url_);
    this->get_parameter("shaker_endpoint",  shaker_endpoint_);
    this->get_parameter("tip_check_base_url", tip_check_cfg_.base_url);
    int passes = 5;
    this->get_parameter("tip_check_passes", passes);
    tip_check_cfg_.passes = std::max(1, passes);
    this->get_parameter("tip_check_timeout_s", tip_check_cfg_.timeout_s);

    RCLCPP_INFO(this->get_logger(), "OT-2 base URL: %s (robot host: %s)",
                ot2_base_url_.c_str(), ot2_host_.c_str());
    RCLCPP_INFO(this->get_logger(), "Shaker base URL: %s%s",
                shaker_base_url_.c_str(), shaker_endpoint_.c_str());
    if (tip_check_cfg_.base_url.empty())
        RCLCPP_INFO(this->get_logger(), "Tip check disabled (tip_check_base_url empty)");
    else
        RCLCPP_INFO(this->get_logger(), "Tip check after each aspirate via %s (%d passes)",
                    tip_check_cfg_.base_url.c_str(), tip_check_cfg_.passes);

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

    tip_check_pub_ = this->create_publisher<std_msgs::msg::String>(
        "tip_check_result", rclcpp::QoS(10).reliable());
    ot2_progress_pub_ = this->create_publisher<std_msgs::msg::String>(
        "ot2_progress", rclcpp::QoS(10).reliable());

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

// Append HTTP response body to a std::string
static size_t collect_response(char* ptr, size_t size, size_t nmemb, void* userdata) {
    static_cast<std::string*>(userdata)->append(ptr, size * nmemb);
    return size * nmemb;
}

bool LabMachineExecutive::http_post(const std::string& url,
                                    const std::string& body,
                                    const std::string& content_type)
{
    return http_post_capture(url, body, content_type, nullptr);
}

bool LabMachineExecutive::http_post_capture(const std::string& url,
                                            const std::string& body,
                                            const std::string& content_type,
                                            std::string* response)
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
    if (response) {
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, collect_response);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA,     response);
    } else {
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, discard_response);
    }

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
        ot2_run_id_       = std::to_string(
            std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));
        ot2_phase_        = OT2Phase::CONNECTING;
        http_in_flight_   = true;
        RCLCPP_INFO(this->get_logger(), "OT-2 connecting to %s…", ot2_host_.c_str());
        publish_ot2_progress("connecting");
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
                publish_ot2_progress("failed");
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
                publish_ot2_progress("homing");
                http_in_flight_ = true;
                pending_http_   = std::async(std::launch::async,
                                             &LabMachineExecutive::home_ot2, this);
                ot2_action_->set_running();
                return;
            }
            const auto& step = ot2_steps_[ot2_current_step_];
            RCLCPP_INFO(this->get_logger(), "OT-2 step %d: %s",
                        ot2_current_step_, step.value("action", "?").c_str());
            publish_ot2_progress("command", ot2_current_step_);
            pending_http_   = std::async(std::launch::async,
                                         &LabMachineExecutive::dispatch_ot2_step,
                                         this, step, ot2_current_step_);
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
                publish_ot2_progress("failed", ot2_current_step_);
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
                publish_ot2_progress("failed", ot2_current_step_);
                ot2_action_->set_failure();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "OT-2 homed — disconnecting…");
            ot2_phase_      = OT2Phase::DISCONNECTING;
            publish_ot2_progress("disconnecting");
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
                publish_ot2_progress("failed", ot2_current_step_);
                ot2_action_->set_failure();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "OT-2 disconnected — SUCCESS");
            publish_ot2_progress("done");
            command_lab_machine_condition_->set(false);
            ot2_condition_->set(false);
            ot2_terminal_ = true;
            ot2_action_->set_success();
            return;
        }
        ot2_action_->set_running();
        return;
    }
}

// Runs one protocol step on the wrapper. After a successful aspirate the tip check runs here, in
// the same worker thread, so the protocol waits for it; its result never fails the step.
bool LabMachineExecutive::dispatch_ot2_step(const nlohmann::json& step, int index) {
    auto dispatch = [this](const nlohmann::json& s) {
        std::string action = s.value("action", "");
        if (action.empty()) return false;
        nlohmann::json body = s;  // step object minus the "action" key
        body.erase("action");
        return http_post(ot2_base_url_ + action_to_endpoint(action), body.dump(),
                         "application/json");
    };
    lab_machine::TipCheckFn check;
    if (!tip_check_cfg_.base_url.empty())
        check = [this, index](const std::string& label) { return run_tip_check(label, index); };
    auto publish = [this](const nlohmann::json& result) {
        std_msgs::msg::String msg;
        msg.data = result.dump();
        tip_check_pub_->publish(msg);
        if (result.value("status", "") == "done")
            RCLCPP_INFO(this->get_logger(), "Tip check: %s",
                        result.value("summary", "").c_str());
        else
            RCLCPP_WARN(this->get_logger(), "Tip check failed, protocol continues: %s",
                        result.value("error", "").c_str());
    };
    return lab_machine::run_protocol_step(step, index, ot2_run_id_, dispatch, check, publish);
}

void LabMachineExecutive::publish_ot2_progress(const std::string& phase, int command,
                                               const nlohmann::json& tip_check) {
    std_msgs::msg::String msg;
    msg.data = lab_machine::ot2_progress(ot2_run_id_, phase, ot2_steps_, command, tip_check).dump();
    ot2_progress_pub_->publish(msg);
}

nlohmann::json LabMachineExecutive::run_tip_check(const std::string& label, int index) {
    RCLCPP_INFO(this->get_logger(), "Tip check: %d passes via %s", tip_check_cfg_.passes,
                tip_check_cfg_.base_url.c_str());
    CurlHttp http(static_cast<long>(tip_check_cfg_.timeout_s) + 30);
    lab_machine::TipCheckClient client(
        http, tip_check_cfg_,
        [](double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); },
        []() {
            return std::chrono::duration<double>(
                       std::chrono::steady_clock::now().time_since_epoch()).count();
        });
    client.on_progress([this, index](const nlohmann::json& p) {
        publish_ot2_progress("tip_check", index, p);
    });
    return client.run(label);
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
        std::string body = "pwm=" + std::to_string(pwm);

        RCLCPP_INFO(this->get_logger(), "Shaker: POST %s body=%s", url.c_str(), body.c_str());

        // Fire-and-report: single blocking call in async thread, check next tick
        pending_http_   = std::async(std::launch::async,
                                     &LabMachineExecutive::http_post,
                                     url, body, std::string("application/x-www-form-urlencoded"));
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
