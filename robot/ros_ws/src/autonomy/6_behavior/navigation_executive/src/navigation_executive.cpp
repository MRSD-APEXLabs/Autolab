#include <navigation_executive/navigation_executive.hpp>

namespace {
constexpr char NAV_STATE_NAVIGATING[] = "NAVIGATING";
constexpr char NAV_STATE_SUCCEEDED[] = "SUCCEEDED";
constexpr char NAV_STATE_FAILED[] = "FAILED";
constexpr char NAV_STATE_CANCELED[] = "CANCELED";
constexpr char REJECTED_PREFIX[] = "REJECTED";
}  // namespace

NavigationExecutive::NavigationExecutive() : Node("navigation_executive") {
    this->declare_parameter<double>("nav_response_timeout", 10.0);
    this->get_parameter("nav_response_timeout", nav_response_timeout_);

    // ── BT objects ────────────────────────────────────────────────────────────
    go_to_location_channel_.commanded_condition =
        new bt::Condition("Go To Location Commanded", this);
    go_to_location_channel_.action = new bt::Action("Go To Location", this);

    navigate_to_pose_channel_.commanded_condition =
        new bt::Condition("Navigate To Pose Commanded", this);
    navigate_to_pose_channel_.action = new bt::Action("Navigate To Pose", this);

    channels_.push_back(&go_to_location_channel_);
    channels_.push_back(&navigate_to_pose_channel_);

    // ── nav_api topics (absolute: nav_api is not namespaced under behavior/) ───
    nav_goal_location_pub_ =
        this->create_publisher<std_msgs::msg::String>("/nav/goal_location", 1);
    nav_goal_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/nav/goal_pose", 1);
    nav_cancel_pub_ = this->create_publisher<std_msgs::msg::Empty>("/nav/cancel", 1);

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();
    nav_state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/nav/state", latched,
        std::bind(&NavigationExecutive::nav_state_callback, this, std::placeholders::_1));
    nav_result_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/nav/result", 10,
        std::bind(&NavigationExecutive::nav_result_callback, this, std::placeholders::_1));

    // ── dispatch: how each channel publishes its goal to nav_api ────────────────
    go_to_location_channel_.dispatch = [this]() {
        std_msgs::msg::String msg;
        msg.data = target_location_;
        nav_goal_location_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Go To Location: dispatching '%s'",
                    target_location_.c_str());
    };
    navigate_to_pose_channel_.dispatch = [this]() {
        nav_goal_pose_pub_->publish(target_pose_);
        RCLCPP_INFO(this->get_logger(), "Navigate To Pose: dispatching (%.2f, %.2f)",
                    target_pose_.pose.position.x, target_pose_.pose.position.y);
    };

    // ── command topics from the BT / routine_executor side ──────────────────────
    go_to_location_command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "go_to_location_command", 1,
        std::bind(&NavigationExecutive::go_to_location_command_callback, this,
                  std::placeholders::_1));
    navigate_to_pose_command_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "navigate_to_pose_command", 1,
        std::bind(&NavigationExecutive::navigate_to_pose_command_callback, this,
                  std::placeholders::_1));

    timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1. / 20.),
                                  std::bind(&NavigationExecutive::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "navigation_executive up: go_to_location_command / navigate_to_pose_command -> "
                "/nav/goal_location / /nav/goal_pose, response timeout %.1f s",
                nav_response_timeout_);
}

bool NavigationExecutive::any_goal_in_flight() const {
    return go_to_location_channel_.goal_in_flight || navigate_to_pose_channel_.goal_in_flight;
}

// ─────────────────────────────────────────────────────────────────────────────
// Commands in
// ─────────────────────────────────────────────────────────────────────────────

void NavigationExecutive::go_to_location_command_callback(
    const std_msgs::msg::String::SharedPtr msg) {
    if (any_goal_in_flight()) {
        RCLCPP_WARN(this->get_logger(), "navigation in progress — ignoring go_to_location('%s')",
                    msg->data.c_str());
        return;
    }
    target_location_ = msg->data;
    go_to_location_channel_.commanded_condition->set(true);
    navigate_to_pose_channel_.commanded_condition->set(false);
    RCLCPP_INFO(this->get_logger(), "Go To Location commanded: '%s'", target_location_.c_str());
}

void NavigationExecutive::navigate_to_pose_command_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (any_goal_in_flight()) {
        RCLCPP_WARN(this->get_logger(), "navigation in progress — ignoring navigate_to_pose");
        return;
    }
    target_pose_ = *msg;
    navigate_to_pose_channel_.commanded_condition->set(true);
    go_to_location_channel_.commanded_condition->set(false);
    RCLCPP_INFO(this->get_logger(), "Navigate To Pose commanded: (%.2f, %.2f)",
                target_pose_.pose.position.x, target_pose_.pose.position.y);
}

// ─────────────────────────────────────────────────────────────────────────────
// nav_api feedback in
// ─────────────────────────────────────────────────────────────────────────────

void NavigationExecutive::nav_state_callback(const std_msgs::msg::String::SharedPtr msg) {
    nav_state_ = msg->data;
    nav_state_stamp_ = this->get_clock()->now();
}

void NavigationExecutive::nav_result_callback(const std_msgs::msg::String::SharedPtr msg) {
    nav_result_ = msg->data;
    nav_result_stamp_ = this->get_clock()->now();
}

// ─────────────────────────────────────────────────────────────────────────────
// Tick
// ─────────────────────────────────────────────────────────────────────────────

void NavigationExecutive::timer_callback() {
    for (Channel* channel : channels_) tick_channel(*channel);

    go_to_location_channel_.commanded_condition->publish();
    navigate_to_pose_channel_.commanded_condition->publish();
    go_to_location_channel_.action->publish();
    navigate_to_pose_channel_.action->publish();
}

void NavigationExecutive::tick_channel(Channel& channel) {
    bt::Action* action = channel.action;

    if (!action->is_active()) {
        if (action->active_has_changed() && channel.goal_in_flight) {
            nav_cancel_pub_->publish(std_msgs::msg::Empty());
            RCLCPP_INFO(this->get_logger(), "%s: deactivated mid-flight — cancelling",
                        action->get_label().c_str());
        }
        channel.goal_in_flight = false;
        channel.seen_navigating = false;
        return;
    }

    if (action->active_has_changed()) {
        channel.dispatch();
        channel.goal_in_flight = true;
        channel.seen_navigating = false;
        channel.dispatched_at = this->get_clock()->now();
        action->set_running();
        return;
    }

    if (!channel.goal_in_flight) {
        action->set_running();
        return;
    }

    // An unknown location name never touches /nav/state at all — nav_api only ever reports
    // it as a one-shot /nav/result rejection. Check that first so a typo fails fast instead
    // of hanging until nav_response_timeout.
    bool result_is_fresh = nav_result_stamp_.nanoseconds() >= channel.dispatched_at.nanoseconds();
    if (result_is_fresh && !nav_result_.empty() && nav_result_.rfind(REJECTED_PREFIX, 0) == 0) {
        RCLCPP_WARN(this->get_logger(), "%s: rejected by nav_api: %s", action->get_label().c_str(),
                    nav_result_.c_str());
        action->set_failure();
        channel.goal_in_flight = false;
        return;
    }

    // /nav/state is latched: a subscription started after a previous, unrelated goal finished
    // would otherwise see its leftover terminal value and immediately (mis)report it as this
    // goal's outcome. Require an observed NAVIGATING transition first, same as nav_api's own
    // _seen_active bookkeeping.
    bool state_is_fresh = nav_state_stamp_.nanoseconds() >= channel.dispatched_at.nanoseconds();
    if (state_is_fresh && nav_state_ == NAV_STATE_NAVIGATING) {
        channel.seen_navigating = true;
    }

    if (state_is_fresh && channel.seen_navigating) {
        if (nav_state_ == NAV_STATE_SUCCEEDED) {
            action->set_success();
            channel.goal_in_flight = false;
            return;
        }
        if (nav_state_ == NAV_STATE_FAILED || nav_state_ == NAV_STATE_CANCELED) {
            action->set_failure();
            channel.goal_in_flight = false;
            return;
        }
    }

    double elapsed = (this->get_clock()->now() - channel.dispatched_at).seconds();
    if (!channel.seen_navigating && elapsed > nav_response_timeout_) {
        RCLCPP_ERROR(this->get_logger(),
                     "%s: no response from nav_api within %.1f s — is the navigation stack "
                     "running? (it does not autostart; see autonomy/1_navigation/README.md)",
                     action->get_label().c_str(), nav_response_timeout_);
        action->set_failure();
        channel.goal_in_flight = false;
        return;
    }

    action->set_running();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationExecutive>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
