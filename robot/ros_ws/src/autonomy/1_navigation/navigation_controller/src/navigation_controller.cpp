#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "navigation_controller/swerve_ik.hpp"
#include "navigation_controller/cmd_vel_watchdog.hpp"

using std::placeholders::_1;

class NavigationController : public rclcpp::Node
{
public:
  NavigationController() : Node("navigation_controller")
  {
    wheel_radius_       = declare_parameter("wheel_radius", 0.0508);
    x_offset_           = declare_parameter("x_offset", 0.25);
    loop_hz_            = declare_parameter("loop_hz", 20.0);
    cmd_vel_timeout_sec_ = declare_parameter("cmd_vel_timeout_sec", 0.3);

    drive_joints_ = {
      "Revolute_2",  // front right
      "Revolute_1",  // front left
      "Revolute_4",  // rear right
      "Revolute_3"   // rear left
    };

    azimuth_joints_ = {
      "Revolute_6",  // front right
      "Revolute_5",  // front left
      "Revolute_8",  // rear right
      "Revolute_7"   // rear left
    };

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/robot_1/cmd_vel", 10,
      std::bind(&NavigationController::cmd_vel_cb, this, _1));

    cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/robot_1/joints/joint_command", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / loop_hz_),
      std::bind(&NavigationController::control_loop, this));

    RCLCPP_INFO(get_logger(), "NavigationController started (cmd_vel -> swerve IK)");
  }

private:
  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_vel_ = *msg;
    last_cmd_vel_time_ = now();
    have_cmd_vel_ = true;
  }

  void control_loop()
  {
    if (!have_cmd_vel_) return;

    const double seconds_since_last =
      (now() - last_cmd_vel_time_).seconds();

    geometry_msgs::msg::Twist cmd_vel = last_cmd_vel_;
    if (navigation_controller::is_cmd_vel_stale(seconds_since_last, cmd_vel_timeout_sec_)) {
      cmd_vel = geometry_msgs::msg::Twist{};
    }

    const auto swerve = navigation_controller::compute_swerve_command(
      cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,
      wheel_radius_, x_offset_);

    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = now();

    const navigation_controller::WheelCommand wheels[4] = {
      swerve.fr, swerve.fl, swerve.rr, swerve.rl};

    for (size_t i = 0; i < 4; ++i) {
      cmd.name.push_back(drive_joints_[i]);
      cmd.velocity.push_back(wheels[i].velocity);
      cmd.position.push_back(0.0);
      cmd.effort.push_back(0.0);

      cmd.name.push_back(azimuth_joints_[i]);
      cmd.position.push_back(wheels[i].position);
      cmd.velocity.push_back(0.0);
      cmd.effort.push_back(0.0);
    }

    cmd_pub_->publish(cmd);
  }

  // Parameters
  double wheel_radius_, x_offset_, loop_hz_, cmd_vel_timeout_sec_;

  // State
  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_;
  bool have_cmd_vel_{false};

  std::vector<std::string> drive_joints_;
  std::vector<std::string> azimuth_joints_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationController>());
  rclcpp::shutdown();
  return 0;
}
