// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <behavior_executive/behavior_executive.hpp>

BehaviorExecutive::BehaviorExecutive() : Node("behavior_executive") {
    // conditions
    armed_condition = new bt::Condition("Armed", this);
    stationary_condition = new bt::Condition("Stationary", this);
    pause_commanded_condition = new bt::Condition("Pause Commanded", this);
    rewind_commanded_condition = new bt::Condition("Rewind Commanded", this);
    fixed_trajectory_condition = new bt::Condition("Fixed Trajectory Commanded", this);
    global_plan_condition = new bt::Condition("Global Plan Commanded", this);
    arm_commanded_condition = new bt::Condition("Arm Commanded", this);
    disarm_commanded_condition = new bt::Condition("Disarm Commanded", this);
    state_estimate_timed_out_condition = new bt::Condition("State Estimate Timed Out", this);
    stuck_condition = new bt::Condition("Stuck", this);
    autonomously_explore_condition = new bt::Condition("Autonomously Explore Commanded", this);
    command_lab_machine_condition = new bt::Condition("Command Lab Machine", this);
    ot2_liquid_handler_condition = new bt::Condition("OT2 Liquid Handler", this);
    custom_shaker_module_condition = new bt::Condition("Custom Shaker Module", this);
    conditions.push_back(armed_condition);
    conditions.push_back(stationary_condition);
    conditions.push_back(pause_commanded_condition);
    conditions.push_back(rewind_commanded_condition);
    conditions.push_back(fixed_trajectory_condition);
    conditions.push_back(global_plan_condition);
    conditions.push_back(arm_commanded_condition);
    conditions.push_back(disarm_commanded_condition);
    conditions.push_back(state_estimate_timed_out_condition);
    conditions.push_back(stuck_condition);
    conditions.push_back(autonomously_explore_condition);
    conditions.push_back(command_lab_machine_condition);
    conditions.push_back(ot2_liquid_handler_condition);
    conditions.push_back(custom_shaker_module_condition);

    // actions
    arm_action = new bt::Action("Arm", this);
    pause_action = new bt::Action("Pause", this);
    rewind_action = new bt::Action("Rewind", this);
    follow_fixed_trajectory_action = new bt::Action("Follow Fixed Trajectory", this);
    global_plan_action = new bt::Action("Follow Global Plan", this);
    request_control_action = new bt::Action("Request Control", this);
    disarm_action = new bt::Action("Disarm", this);
    actions.push_back(arm_action);
    actions.push_back(pause_action);
    actions.push_back(rewind_action);
    actions.push_back(follow_fixed_trajectory_action);
    actions.push_back(global_plan_action);
    actions.push_back(request_control_action);
    actions.push_back(disarm_action);

    // subscribers
    behavior_tree_commands_sub =
        this->create_subscription<behavior_tree_msgs::msg::BehaviorTreeCommands>(
            "behavior_tree_commands", 1,
            std::bind(&BehaviorExecutive::bt_commands_callback, this, std::placeholders::_1));
    is_armed_sub = this->create_subscription<std_msgs::msg::Bool>(
        "is_armed", 1,
        std::bind(&BehaviorExecutive::is_armed_callback, this, std::placeholders::_1));
    state_estimate_timed_out_sub =
      this->create_subscription<std_msgs::msg::Bool>("state_estimate_timed_out", 1,
						     std::bind(&BehaviorExecutive::state_estimate_timed_out_callback,
							       this, std::placeholders::_1));
    stuck_sub =
      this->create_subscription<std_msgs::msg::Bool>("stuck", 1,
						     std::bind(&BehaviorExecutive::stuck_callback,
							       this, std::placeholders::_1));
									 

    // publishers
    recording_pub = this->create_publisher<std_msgs::msg::Bool>("set_recording_status", 1);
    reset_stuck_pub = this->create_publisher<std_msgs::msg::Empty>("reset_stuck", 1);
    clear_map_pub = this->create_publisher<std_msgs::msg::Empty>("clear_map", 1);

    // services
    service_callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    global_planner_toggle_client = this->create_client<std_srvs::srv::Trigger>(
        "global_plan_toggle", rmw_qos_profile_services_default, service_callback_group);

    // timers
    timer = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1. / 20.),
                                 std::bind(&BehaviorExecutive::timer_callback, this));
}

void BehaviorExecutive::timer_callback() {
    if (request_control_action->is_active()) {
    }

    if (arm_action->is_active()) {
        if (arm_action->active_has_changed()) {
	    std_msgs::msg::Bool start_msg;
	    start_msg.data = true;
	    recording_pub->publish(start_msg);
	
        }
    }

    if (disarm_action->is_active()) {
        if (disarm_action->active_has_changed()) {
	    std_msgs::msg::Bool stop_msg;
	    stop_msg.data = false;
	    recording_pub->publish(stop_msg);
        }
    }


    if (pause_action->is_active()) {
        pause_action->set_running();
    }

    if (rewind_action->is_active()) {
        rewind_action->set_running();
    }

    // follow fixed trajectory action
    if (follow_fixed_trajectory_action->is_active()) {
        follow_fixed_trajectory_action->set_running();

    }

    if (global_plan_action->is_active()) {
        global_plan_action->set_running();
        if (global_plan_action->active_has_changed()) {

	    if(global_planner_toggle_client->service_is_ready()){
	        std_srvs::srv::Trigger::Request::SharedPtr request =
                    std::make_shared<std_srvs::srv::Trigger::Request>();
	        auto result = global_planner_toggle_client->async_send_request(request);
	        result.wait();
	        if (result.get()->success)
                    global_plan_action->set_success();
	        else
                    global_plan_action->set_failure();
	    }
        };
    }

    for (bt::Condition* condition : conditions) condition->publish();
    for (bt::Action* action : actions) action->publish();
}

// callbacks

void BehaviorExecutive::bt_commands_callback(behavior_tree_msgs::msg::BehaviorTreeCommands msg) {
    for (size_t i = 0; i < msg.commands.size(); i++) {
        std::string condition_name = msg.commands[i].condition_name;
        int status = msg.commands[i].status;

        for (size_t j = 0; j < conditions.size(); j++) {
            bt::Condition* condition = conditions[j];
            if (condition_name == condition->get_label()) {
                if (status == behavior_tree_msgs::msg::Status::SUCCESS)
                    condition->set(true);
                else if (status == behavior_tree_msgs::msg::Status::FAILURE)
                    condition->set(false);
            }
        }
    }
}

void BehaviorExecutive::is_armed_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    armed_condition->set(msg->data);
}

void BehaviorExecutive::state_estimate_timed_out_callback(const std_msgs::msg::Bool::SharedPtr msg){
  state_estimate_timed_out_condition->set(msg->data);
}


void BehaviorExecutive::stuck_callback(const std_msgs::msg::Bool::SharedPtr msg){
  stuck_condition->set(msg->data);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<BehaviorExecutive>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
} 
