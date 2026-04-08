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
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(manipulation_executive src/manipulation_executive.cpp)

target_include_directories(manipulation_executive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(manipulation_executive PUBLIC cxx_std_17)

ament_target_dependencies(manipulation_executive
  rclcpp
  std_msgs
  behavior_tree
  behavior_tree_msgs
)

target_link_libraries(manipulation_executive
  nlohmann_json::nlohmann_json
  Boost::system
)

install(TARGETS manipulation_executive
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
