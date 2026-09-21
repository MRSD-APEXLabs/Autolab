#include "nav2_smac_planner/smac_planner_2d.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "navigation_bringup/latest_frame_path.hpp"

namespace navigation_bringup
{
// Retain Smac's search, smoothing, footprint checks and cancellation behavior.
// Only the temporal interpretation of poses in its fixed map-frame route differs.
//
// nav2_core::GlobalPlanner::createPlan() gained a cancel_checker argument after Humble, so
// CMakeLists defines NAV2_PLANNER_CANCEL_CHECKER on the distros that have it and this one file
// builds both in the robot container (Humble) and on the host (Jazzy).
class LatestFrameSmacPlanner2D : public nav2_smac_planner::SmacPlanner2D
{
public:
#ifdef NAV2_PLANNER_CANCEL_CHECKER
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override
  {
    auto path = SmacPlanner2D::createPlan(start, goal, cancel_checker);
    useLatestPoseTransforms(path);
    return path;
  }
#else
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    auto path = SmacPlanner2D::createPlan(start, goal);
    useLatestPoseTransforms(path);
    return path;
  }
#endif
};
}  // namespace navigation_bringup

PLUGINLIB_EXPORT_CLASS(navigation_bringup::LatestFrameSmacPlanner2D, nav2_core::GlobalPlanner)
