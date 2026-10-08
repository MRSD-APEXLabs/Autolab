cmake_minimum_required(VERSION 3.10)
project(navigation_bringup)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Smac 2D with the map-frame route re-transformed at the current localisation (see
# include/navigation_bringup/latest_frame_path.hpp). nav2_params.yaml selects it as "GridBased".
add_library(latest_frame_smac_planner SHARED src/latest_frame_smac_planner.cpp)
target_compile_features(latest_frame_smac_planner PUBLIC cxx_std_17)
target_include_directories(latest_frame_smac_planner PRIVATE include)
ament_target_dependencies(latest_frame_smac_planner nav2_core nav2_smac_planner pluginlib nav_msgs)
# nav2_core::GlobalPlanner::createPlan() takes a cancel_checker after Humble.
if(NOT "$ENV{ROS_DISTRO}" STREQUAL "humble")
  target_compile_definitions(latest_frame_smac_planner PRIVATE NAV2_PLANNER_CANCEL_CHECKER)
endif()
pluginlib_export_plugin_description_file(nav2_core planner_plugins.xml)
install(TARGETS latest_frame_smac_planner LIBRARY DESTINATION lib)

# ControllerServer's shared Humble TF buffer can stop receiving /tf. This MPPI wrapper retains the
# stock controller and feeds odom->base into that same buffer directly from nav_msgs/Odometry.
add_library(fresh_odom_controllers SHARED src/fresh_odom_controllers.cpp)
target_compile_features(fresh_odom_controllers PUBLIC cxx_std_17)
ament_target_dependencies(
  fresh_odom_controllers nav2_core nav2_mppi_controller nav_msgs
  pluginlib tf2_ros)
pluginlib_export_plugin_description_file(nav2_core controller_plugins.xml)
install(TARGETS fresh_odom_controllers LIBRARY DESTINATION lib)

install(
  DIRECTORY launch config rviz urdf behavior_trees maps phoenix6
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
