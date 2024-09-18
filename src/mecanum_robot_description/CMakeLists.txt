cmake_minimum_required(VERSION 3.8)
project(mecanum_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

install(DIRECTORY urdf models launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY models/mecanum_robot_description/meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(gazebo_ros_pkgs)

ament_package()
