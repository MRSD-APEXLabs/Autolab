#include <cmath>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

static inline double normalize_angle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double yaw_from_quat(double x, double y, double z, double w)
{
  const double siny = 2.0 * (w * z + x * y);
  const double cosy = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny, cosy);
}

class NavigationController : public rclcpp::Node
{
public:
  NavigationController() : Node("navigation_controller")
  {
    wheel_radius_ = declare_parameter("wheel_radius", 0.0508);
    kp_xy_        = declare_parameter("kp_xy", 1.0);
    kp_yaw_       = declare_parameter("kp_yaw", 2.0);
    loop_hz_      = declare_parameter("loop_hz", 20.0);
    x_offset_     = declare_parameter("x_offset", 0.25);

    drive_joints_ = {
      "Revolute_1",  // front right
      "Revolute_2",  // front left
      "Revolute_4",  // rear right
      "Revolute_3"   // rear left
    };

    azimuth_joints_ = {
      "Revolute_5",  // front right
      "Revolute_6",  // front left
      "Revolute_8",  // rear right
      "Revolute_7"   // rear left
    };

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/robot_1/joints/joint_states", 10,
      std::bind(&NavigationController::joint_cb, this, _1));

    goal_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "/robot_1/swerve/goal_pose", 10,
      std::bind(&NavigationController::goal_cb, this, _1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/robot_1/odom", 10,
      std::bind(&NavigationController::odom_cb, this, _1));

    cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/robot_1/joints/joint_command", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / loop_hz_),
      std::bind(&NavigationController::control_loop, this));

    RCLCPP_INFO(get_logger(), "NavigationController started (custom swerve kinematics)");
  }

private:
  void joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = *msg;
  }

  void goal_cb(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    goal_x_ = msg->x;
    goal_y_ = msg->y;
    goal_yaw_ = msg->theta * M_PI / 180.0;
    goal_received_ = true;
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    curr_x_ = msg->pose.pose.position.x;
    curr_y_ = msg->pose.pose.position.y;
    curr_yaw_ = yaw_from_quat(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  }

  void control_loop()
  {
    if (!goal_received_) return;

    const double ex = goal_x_ - curr_x_;
    const double ey = goal_y_ - curr_y_;

    const double c = std::cos(curr_yaw_);
    const double s = std::sin(curr_yaw_);

    const double ex_r =  c * ex + s * ey;
    const double ey_r = -s * ex + c * ey;

    const double linear_x_cmd = kp_xy_ * ex_r;
    const double linear_y_cmd = -kp_xy_ * ey_r;

    const double yaw_err = normalize_angle(goal_yaw_ - curr_yaw_);
    const double angular_cmd = -kp_yaw_ * yaw_err;

    const double a = -linear_y_cmd - angular_cmd * x_offset_ / 2.0;
    const double b = -linear_y_cmd + angular_cmd * x_offset_ / 2.0;
    const double c_ = linear_x_cmd - angular_cmd * x_offset_ / 2.0;
    const double d = linear_x_cmd + angular_cmd * x_offset_ / 2.0;

    const double fl_vel =  std::hypot(b, d) / (wheel_radius_ * M_PI);
    const double fr_vel = -std::hypot(b, c_) / (wheel_radius_ * M_PI);
    const double rl_vel =  std::hypot(a, d) / (wheel_radius_ * M_PI);
    const double rr_vel = -std::hypot(a, c_) / (wheel_radius_ * M_PI);

    const double fl_pos = -std::atan2(b, d);
    const double fr_pos = -std::atan2(b, c_);
    const double rl_pos = -std::atan2(a, d);
    const double rr_pos = -std::atan2(a, c_);

    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = now();

    const double wheel_vel[4] = {fr_vel, fl_vel, rr_vel, rl_vel};
    const double wheel_pos[4] = {fr_pos, fl_pos, rr_pos, rl_pos};

    for (size_t i = 0; i < 4; ++i) {
      cmd.name.push_back(drive_joints_[i]);
      cmd.velocity.push_back(std::isnan(wheel_vel[i]) ? 0.0 : wheel_vel[i]);
      cmd.position.push_back(0.0);
      cmd.effort.push_back(0.0);

      cmd.name.push_back(azimuth_joints_[i]);
      cmd.position.push_back(std::isnan(wheel_pos[i]) ? 0.0 : wheel_pos[i]);
      cmd.velocity.push_back(0.0);
      cmd.effort.push_back(0.0);
    }

    cmd_pub_->publish(cmd);
  }

  // Parameters
  double wheel_radius_, kp_xy_, kp_yaw_, loop_hz_, x_offset_;

  // State
  double curr_x_{0}, curr_y_{0}, curr_yaw_{0};
  double goal_x_{1}, goal_y_{0}, goal_yaw_{0};
  bool goal_received_{false};

  std::vector<std::string> drive_joints_;
  std::vector<std::string> azimuth_joints_;

  sensor_msgs::msg::JointState last_joint_state_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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
