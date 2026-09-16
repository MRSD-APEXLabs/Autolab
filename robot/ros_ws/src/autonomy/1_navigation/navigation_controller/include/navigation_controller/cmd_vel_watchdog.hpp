#pragma once

namespace navigation_controller {

inline bool is_cmd_vel_stale(double seconds_since_last_msg, double timeout_sec) {
  return seconds_since_last_msg >= timeout_sec;
}

}  // namespace navigation_controller
