cmake_minimum_required(VERSION 3.5)
project(global_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(move_to_pose src/move_to_pose.cpp)

ament_target_dependencies(move_to_pose
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  moveit_msgs
  trajectory_msgs
  visualization_msgs
  shape_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS move_to_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()