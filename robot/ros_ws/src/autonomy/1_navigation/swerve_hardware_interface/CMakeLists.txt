cmake_minimum_required(VERSION 3.8)
project(swerve_hardware_interface)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(phoenix6 REQUIRED)

add_library(swerve_module_mapping
  src/module_mapping.cpp
)
target_include_directories(swerve_module_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(swerve_hardware_interface_node
  src/swerve_hardware_interface_node.cpp
  src/swerve_module_hardware.cpp
)
target_link_libraries(swerve_hardware_interface_node
  swerve_module_mapping
  phoenix6
)
ament_target_dependencies(swerve_hardware_interface_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  swerve_hardware_interface_node
  swerve_module_mapping
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_module_mapping test/test_module_mapping.cpp)
  target_link_libraries(test_module_mapping swerve_module_mapping)
endif()

ament_package()
