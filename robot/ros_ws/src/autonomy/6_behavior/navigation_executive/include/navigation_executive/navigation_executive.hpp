#pragma once

#include <functional>
#include <string>
#include <vector>

#include <behavior_tree/behavior_tree.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

class NavigationExecutive : public rclcpp::Node {
   public:
    NavigationExecutive();

   private:
    // One navigation "channel": a (Condition, Action) pair plus the bookkeeping needed to
    // translate its activation into a nav_api goal and its outcome back into a BT status.
    struct Channel {
        bt::Condition* commanded_condition;
        bt::Action* action;
        bool goal_in_flight = false;
        bool seen_navigating = false;
        rclcpp::Time dispatched_at;
        std::function<void()> dispatch;  // publishes the goal to nav_api
    };

    Channel go_to_location_channel_;
    Channel navigate_to_pose_channel_;
    std::vector<Channel*> channels_;

    std::string target_location_;
    geometry_msgs::msg::PoseStamped target_pose_;

    // Latest values observed on the shared nav_api feedback topics.
    std::string nav_state_;
    rclcpp::Time nav_state_stamp_;
    std::string nav_result_;
    rclcpp::Time nav_result_stamp_;

    double nav_response_timeout_;

    // BT / routine_executor side (relative — namespaced under behavior/ by the launch file).
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr go_to_location_command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr navigate_to_pose_command_sub_;

    // nav_api side (absolute — nav_api is not namespaced under behavior/).
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_goal_location_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr nav_cancel_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_result_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void go_to_location_command_callback(const std_msgs::msg::String::SharedPtr msg);
    void navigate_to_pose_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void nav_state_callback(const std_msgs::msg::String::SharedPtr msg);
    void nav_result_callback(const std_msgs::msg::String::SharedPtr msg);

    void timer_callback();
    void tick_channel(Channel& channel);

    // True if some channel already has a goal dispatched and not yet terminal. Command
    // callbacks MUST check this instead of Action::is_active() — see the plan's Global
    // Constraints for why.
    bool any_goal_in_flight() const;
};
