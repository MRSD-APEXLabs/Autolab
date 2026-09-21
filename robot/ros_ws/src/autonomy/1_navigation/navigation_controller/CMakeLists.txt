cmake_minimum_required(VERSION 3.8)
project(navigation_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(navigation_controller_ik
  src/swerve_ik.cpp
)
target_include_directories(navigation_controller_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(navigation_controller src/navigation_controller.cpp)
target_link_libraries(navigation_controller navigation_controller_ik)
ament_target_dependencies(navigation_controller
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

install(TARGETS
  navigation_controller
  navigation_controller_ik
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_swerve_ik test/test_swerve_ik.cpp)
  target_link_libraries(test_swerve_ik navigation_controller_ik)
endif()

ament_package()
