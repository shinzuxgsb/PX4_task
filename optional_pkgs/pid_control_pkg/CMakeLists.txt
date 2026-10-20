cmake_minimum_required(VERSION 3.8)
project(pid_control_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(traj_utils REQUIRED)
find_package(bspline_opt REQUIRED)

add_executable(position_pid_controller src/pid_controller.cpp)
ament_target_dependencies(position_pid_controller
  rclcpp
  rcl_interfaces
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)
target_include_directories(position_pid_controller PRIVATE include)

add_executable(bspline_trajectory_tracker src/bspline_trajectory_tracker.cpp)
ament_target_dependencies(bspline_trajectory_tracker
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  Eigen3
  traj_utils
  bspline_opt
)
target_include_directories(bspline_trajectory_tracker PRIVATE include)

if(MSVC)
  target_compile_options(position_pid_controller PRIVATE /W4)
else()
  target_compile_options(position_pid_controller PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS position_pid_controller bspline_trajectory_tracker
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py")

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml")

ament_package()
