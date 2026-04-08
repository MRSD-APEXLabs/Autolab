cmake_minimum_required(VERSION 3.8)
project(lab_machine_executive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behavior_tree REQUIRED)
find_package(behavior_tree_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(lab_machine_executive src/lab_machine_executive.cpp)

target_include_directories(lab_machine_executive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(lab_machine_executive PUBLIC cxx_std_17)

ament_target_dependencies(lab_machine_executive
  rclcpp
  std_msgs
  behavior_tree
  behavior_tree_msgs
)

target_link_libraries(lab_machine_executive
  CURL::libcurl
  nlohmann_json::nlohmann_json
)

install(TARGETS lab_machine_executive
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
