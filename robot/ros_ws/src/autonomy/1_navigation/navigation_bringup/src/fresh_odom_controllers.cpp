#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace navigation_bringup
{

// Humble's costmap TransformListener can stop updating after startup while /tf itself remains
// healthy. Both ControllerServer and its controller plugins share that buffer, so a stale
// odom->base transform makes every valid path empty. Odometry already contains exactly this
// transform. Feeding it into the shared buffer also keeps the costmap and ControllerServer's
// progress/goal checks current, without creating a second source of robot pose.
class OdomTfBufferFeeder
{
public:
  void start(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::shared_ptr<tf2_ros::Buffer> & buffer,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap)
  {
    node_ = parent.lock();
    if (!node_) {
      throw std::runtime_error("FreshOdom controller: parent lifecycle node expired");
    }
    buffer_ = buffer;
    odom_frame_ = costmap->getGlobalFrameID();
    base_frame_ = costmap->getBaseFrameID();
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      std::bind(&OdomTfBufferFeeder::onOdom, this, std::placeholders::_1));
    RCLCPP_INFO(
      node_->get_logger(),
      "FreshOdom controller: /odom will keep shared TF %s -> %s current",
      odom_frame_.c_str(), base_frame_.c_str());
  }

  void stop()
  {
    odom_sub_.reset();
    buffer_.reset();
    node_.reset();
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!buffer_ || msg->header.frame_id != odom_frame_ ||
      (!msg->child_frame_id.empty() && msg->child_frame_id != base_frame_))
    {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg->header;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    transform.transform.rotation = msg->pose.pose.orientation;

    // RotationShim asks for a transform at clock->now(), a few milliseconds after this odometry
    // sample. Extending the sample's validity slightly avoids harmless future extrapolation while
    // remaining far below the 0.5 s command watchdog.
    transform.header.stamp =
      rclcpp::Time(msg->header.stamp) + rclcpp::Duration::from_seconds(0.1);
    buffer_->setTransform(transform, "odometry_buffer_feeder", false);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::string odom_frame_;
  std::string base_frame_;
};

class FreshOdomMPPIController : public nav2_mppi_controller::MPPIController
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    MPPIController::configure(parent, name, tf, costmap);
    feeder_.start(parent, tf, costmap);
  }

  void cleanup() override
  {
    feeder_.stop();
    MPPIController::cleanup();
  }

private:
  OdomTfBufferFeeder feeder_;
};

}  // namespace navigation_bringup

PLUGINLIB_EXPORT_CLASS(navigation_bringup::FreshOdomMPPIController, nav2_core::Controller)
