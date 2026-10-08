#ifndef NAVIGATION_BRINGUP__LATEST_FRAME_PATH_HPP_
#define NAVIGATION_BRINGUP__LATEST_FRAME_PATH_HPP_

#include "nav_msgs/msg/path.hpp"

namespace navigation_bringup
{
// A route in the map frame describes fixed locations, not historical sensor
// observations. Jazzy ControllerServer copies the last pose into end_pose_ and
// transforms it at its pose stamp. MPPI and RotationShim use the current TF.
// Request current TF for the route poses so all three agree after AMCL updates.
// Preserve the Path header stamp: PathExpiringTimer needs a new path identity
// even when a periodic replan produces exactly the same geometric route.
inline void useLatestPoseTransforms(nav_msgs::msg::Path & path)
{
  for (auto & pose : path.poses) {
    pose.header.stamp = builtin_interfaces::msg::Time();
  }
}
}  // namespace navigation_bringup
#endif
