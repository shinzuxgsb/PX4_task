cmake_minimum_required(VERSION 3.8)
project(activity_control_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

add_library(route_target_publisher SHARED
  src/route_target_publisher.cpp
  src/waypoint_navigator.cpp
)
target_include_directories(route_target_publisher PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
   $<INSTALL_INTERFACE:include>)
ament_target_dependencies(route_target_publisher
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)
if(MSVC)
  target_compile_options(route_target_publisher PRIVATE /W4)
else()
  target_compile_options(route_target_publisher PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(route_target_publisher_node src/route_target_publisher_main.cpp)
target_link_libraries(route_target_publisher_node route_target_publisher)
ament_target_dependencies(route_target_publisher_node rclcpp)

add_executable(planner_target_adapter_node
  src/planner_target_adapter.cpp
  src/planner_target_adapter_main.cpp
)
target_include_directories(planner_target_adapter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(planner_target_adapter_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  tf2
  tf2_geometry_msgs
)
if(MSVC)
  target_compile_options(planner_target_adapter_node PRIVATE /W4)
else()
  target_compile_options(planner_target_adapter_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(planner_velocity_tracker_node
  src/planner_velocity_tracker.cpp
  src/planner_velocity_tracker_main.cpp
)
target_include_directories(planner_velocity_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(planner_velocity_tracker_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  tf2
  tf2_geometry_msgs
  angles
)
if(MSVC)
  target_compile_options(planner_velocity_tracker_node PRIVATE /W4)
else()
  target_compile_options(planner_velocity_tracker_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(precision_landing_manager_node
  src/precision_landing_manager.cpp
)
target_include_directories(precision_landing_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(precision_landing_manager_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)
if(MSVC)
  target_compile_options(precision_landing_manager_node PRIVATE /W4)
else()
  target_compile_options(precision_landing_manager_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS route_target_publisher
  EXPORT export_route_target_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  route_target_publisher_node
  planner_target_adapter_node
  planner_velocity_tracker_node
  precision_landing_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py")

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml")

ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)
ament_export_include_directories(include)
ament_export_targets(export_route_target_publisher HAS_LIBRARY_TARGET)

ament_package()
