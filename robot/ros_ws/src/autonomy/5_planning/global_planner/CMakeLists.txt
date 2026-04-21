cmake_minimum_required(VERSION 3.8)
project(global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(ompl REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
# Executable
find_package(nlohmann_json REQUIRED)
find_package(moveit_core REQUIRED)
find_package(xarm_msgs REQUIRED)
add_executable(move_to_pose_node src/move_to_pose_node.cpp)

# Include directories (optional but recommended)
target_include_directories(move_to_pose_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(move_to_pose_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  nav_msgs
  moveit_core
  sensor_msgs
  std_msgs
  std_srvs
  moveit_msgs
  trajectory_msgs
  visualization_msgs
  shape_msgs
  tf2
  tf2_geometry_msgs
  xarm_msgs
)
target_link_libraries(move_to_pose_node
  moveit_ros_planning::moveit_planning_scene_monitor
  ${moveit_core_LIBRARIES}
  nlohmann_json::nlohmann_json
  moveit_core::moveit_trajectory_processing
  moveit_core::moveit_robot_trajectory
)
# Install target
install(TARGETS
  move_to_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()