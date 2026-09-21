// src/swerve_hardware_interface_node.cpp
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>

#include "swerve_hardware_interface/module_mapping.hpp"
#include "swerve_hardware_interface/swerve_module_hardware.hpp"

using namespace std::chrono_literals;

namespace swerve_hardware_interface
{

class SwerveHardwareInterfaceNode : public rclcpp::Node
{
public:
  SwerveHardwareInterfaceNode()
  : Node("swerve_hardware_interface_node"),
    bus_(declare_parameter("can_bus_name", std::string("AutoLab Canivore")))
  {
    SharedModuleParams params{
      declare_parameter("drive_gear_ratio", 7.03125),
      declare_parameter("steer_gear_ratio", 26.09090909090909),
      declare_parameter("coupling_gear_ratio", 4.5),
      declare_parameter("steer_kp", 100.0),
      declare_parameter("steer_ki", 0.0),
      declare_parameter("steer_kd", 0.5),
      declare_parameter("steer_ks", 0.1),
      declare_parameter("steer_kv", 2.49),
      declare_parameter("drive_kp", 0.1),
      declare_parameter("drive_ki", 0.0),
      declare_parameter("drive_kd", 0.0),
      declare_parameter("drive_ks", 0.0),
      declare_parameter("drive_kv", 0.124),
      declare_parameter("steer_stator_current_limit_a", 60.0),
    };

    // NOTE: declare_parameter("command_watchdog_timeout_ms", ...) is declared exactly once,
    // here. watchdog_timeout_ (an rclcpp::Duration) is derived from this same value below —
    // it must NOT be re-declared via a second declare_parameter call for the same name.
    const auto command_watchdog_timeout_ms =
      std::chrono::milliseconds(declare_parameter("command_watchdog_timeout_ms", 250));
    watchdog_timeout_ = rclcpp::Duration(command_watchdog_timeout_ms);
    double publish_rate_hz = declare_parameter("joint_states_publish_rate_hz", 100.0);

    module_configs_ = default_module_configs();
    for (const auto & cfg : module_configs_) {
      modules_.emplace_back(std::make_unique<SwerveModuleHardware>(cfg, params, bus_));
    }

    // Phoenix6's CANivore client session comes up asynchronously in the
    // background (see the "CANbus Connected"/"Network Up" log lines, which
    // can print AFTER this constructor already ran) — calling configure()
    // immediately after construction can race that bring-up and fail every
    // module on the first attempt even though the bus is fine moments
    // later. Retry the whole configure pass a few times with a short wait,
    // instead of giving up after one attempt.
    bool all_configured = false;
    for (int attempt = 0; attempt < 10 && !all_configured; ++attempt) {
      if (attempt > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      all_configured = true;
      for (auto & module : modules_) {
        all_configured = module->configure() && all_configured;
      }
    }
    if (!all_configured) {
      RCLCPP_ERROR(
        get_logger(),
        "One or more swerve modules failed to configure — check CAN wiring/IDs. "
        "Hardware is DISABLED (FeedEnable keepalive will not start, so all motors will "
        "auto-disable within 100ms). Fix the wiring/CAN issue and restart this node.");
    }

    joint_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/robot_1/joints/joint_command", 10,
      std::bind(&SwerveHardwareInterfaceNode::joint_command_cb, this, std::placeholders::_1));

    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/robot_1/joints/joint_states", 10);

    last_command_time_ = now();

    if (all_configured) {
      enable_feed_timer_ = create_wall_timer(
        50ms, [this]() {ctre::phoenix::unmanaged::FeedEnable(100);});
    }

    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      std::bind(&SwerveHardwareInterfaceNode::publish_joint_states, this));

    watchdog_timer_ = create_wall_timer(
      50ms, std::bind(&SwerveHardwareInterfaceNode::check_watchdog, this));

    RCLCPP_INFO(get_logger(), "swerve_hardware_interface_node started (%zu modules)", modules_.size());
  }

private:
  // KNOWN OPEN ISSUE — verify before bench-testing:
  // navigation_controller.cpp computes drive velocity as
  // hypot(...) / (wheel_radius_ * M_PI), i.e. linear_speed / (r * pi).
  // True wheel rot/s would be linear_speed / (2 * pi * r) — this looks
  // like it may be 2x true rot/s. This node takes msg->velocity[i] as
  // literal wheel rot/s with no scaling. If the 2x reading is correct,
  // real hardware will spin at roughly double the intended speed on
  // first command. This was NOT fixed here because the plan explicitly
  // keeps navigation_controller's kinematics unmodified (non-goal) —
  // confirm the actual units experimentally (command a known low speed,
  // measure wheel rotation rate) before running at speed.
  void joint_command_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_command_time_ = now();
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const ModuleConfig * cfg = find_module_for_joint(module_configs_, msg->name[i]);
      if (cfg == nullptr) {continue;}
      auto & module = modules_[static_cast<size_t>(cfg - module_configs_.data())];
      if (msg->name[i] == cfg->drive_joint && i < msg->velocity.size()) {
        module->set_drive_velocity_rotps(msg->velocity[i]);
      } else if (msg->name[i] == cfg->azimuth_joint && i < msg->position.size()) {
        module->set_steer_position_rad(msg->position[i]);
      }
    }
  }

  void check_watchdog()
  {
    bool stale_command = now() - last_command_time_ > watchdog_timeout_;

    bool any_unhealthy = false;
    for (auto & module : modules_) {
      if (!module->is_healthy()) {
        any_unhealthy = true;
        break;
      }
    }
    if (any_unhealthy) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "One or more swerve modules reported unhealthy status (CAN/device fault) — "
        "zeroing drive velocities until resolved");
    }

    if (stale_command || any_unhealthy) {
      for (auto & module : modules_) {
        module->set_drive_velocity_rotps(0.0);
      }
    }
  }

  void publish_joint_states()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    for (size_t m = 0; m < module_configs_.size(); ++m) {
      const auto & cfg = module_configs_[m];
      msg.name.push_back(cfg.drive_joint);
      msg.velocity.push_back(modules_[m]->get_drive_velocity_rotps());
      msg.position.push_back(0.0);
      msg.effort.push_back(0.0);

      msg.name.push_back(cfg.azimuth_joint);
      msg.position.push_back(modules_[m]->get_steer_position_rad());
      msg.velocity.push_back(0.0);
      msg.effort.push_back(0.0);
    }
    joint_states_pub_->publish(msg);
  }

  ctre::phoenix6::CANBus bus_;
  std::vector<ModuleConfig> module_configs_;
  std::vector<std::unique_ptr<SwerveModuleHardware>> modules_;

  rclcpp::Time last_command_time_;
  rclcpp::Duration watchdog_timeout_{0, 0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::TimerBase::SharedPtr enable_feed_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace swerve_hardware_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swerve_hardware_interface::SwerveHardwareInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
