cmake_minimum_required(VERSION 3.8)
project(base_urdf)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(FILES robot.urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()