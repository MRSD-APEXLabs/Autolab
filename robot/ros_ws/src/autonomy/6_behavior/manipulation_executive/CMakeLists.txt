cmake_minimum_required(VERSION 3.8)
project(manipulation_executive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behavior_tree REQUIRED)
find_package(behavior_tree_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(manipulation_executive src/manipulation_executive.cpp)

target_include_directories(manipulation_executive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(manipulation_executive PUBLIC cxx_std_17)

ament_target_dependencies(manipulation_executive
  rclcpp
  std_msgs
  std_srvs
  behavior_tree
  behavior_tree_msgs
)

target_link_libraries(manipulation_executive
  nlohmann_json::nlohmann_json
)

install(TARGETS manipulation_executive
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # Pure logic only (manipulation_logic.hpp) — no ROS needed
  ament_add_gtest(test_manipulation_logic test/test_manipulation_logic.cpp)
  target_include_directories(test_manipulation_logic PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_manipulation_logic PUBLIC cxx_std_17)
  target_link_libraries(test_manipulation_logic
    nlohmann_json::nlohmann_json
  )
endif()

ament_package()
