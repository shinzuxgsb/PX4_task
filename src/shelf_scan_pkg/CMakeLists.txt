cmake_minimum_required(VERSION 3.8)
project(shelf_scan_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui videoio objdetect aruco)
find_library(ZBAR_LIBRARY zbar REQUIRED)

# ---------------------------------------------------------------- WebSocket 链路开关
# 与视觉/相机部分无关，只影响 shelf_gs_link_node 一个 target：
#   SHELF_NO_WEBSOCKET=1 bash build.sh          （环境变量方式关掉）
#   colcon build --cmake-args -DSHELF_ENABLE_WEBSOCKET=OFF
option(SHELF_ENABLE_WEBSOCKET "编译 WebSocket 链路（需要 Boost.Beast）" ON)
if(DEFINED ENV{SHELF_NO_WEBSOCKET})
  set(SHELF_ENABLE_WEBSOCKET OFF)
  message(STATUS "SHELF_NO_WEBSOCKET 已设置：本次不编译 WebSocket 链路，改用 UDP/串口")
endif()
find_package(Boost QUIET COMPONENTS system)
find_package(Threads QUIET)

# ---------------------------------------------------------------- 纯逻辑库
# 场地几何 / 航线规划 / 协议解析，与地面站 groundstation/gs 一一对应
add_library(shelf_scan_core SHARED
  src/field_geometry.cpp
  src/route_planner.cpp
  src/gs_protocol.cpp
  src/simple_serial.cpp
)
target_include_directories(shelf_scan_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(shelf_scan_core ament_index_cpp)

# ---------------------------------------------------------------- 任务主节点
add_executable(shelf_mission_node src/mission_node.cpp)
target_link_libraries(shelf_mission_node shelf_scan_core)
ament_target_dependencies(shelf_mission_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

# ---------------------------------------------------------------- 链路节点
add_executable(shelf_gs_link_node src/gs_link_node.cpp)
target_link_libraries(shelf_gs_link_node shelf_scan_core)
ament_target_dependencies(shelf_gs_link_node rclcpp std_msgs)
if(Boost_FOUND AND SHELF_ENABLE_WEBSOCKET)
  target_include_directories(shelf_gs_link_node PRIVATE ${Boost_INCLUDE_DIRS})
  target_link_libraries(shelf_gs_link_node ${Boost_LIBRARIES})
  target_compile_definitions(shelf_gs_link_node PRIVATE SHELF_WEBSOCKET_ENABLED=1)
endif()
if(Threads_FOUND)
  target_link_libraries(shelf_gs_link_node Threads::Threads)
endif()

# ---------------------------------------------------------------- 识别节点
add_executable(shelf_code_reader_node src/code_reader_node.cpp)
ament_target_dependencies(shelf_code_reader_node rclcpp std_msgs std_srvs)
target_include_directories(shelf_code_reader_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include   # mjpeg_server.hpp（浏览器预览服务）
)
target_link_libraries(shelf_code_reader_node ${OpenCV_LIBS} ${ZBAR_LIBRARY})
# 预览服务跑在自己的 accept 线程上，需要 pthread
if(Threads_FOUND)
  target_link_libraries(shelf_code_reader_node Threads::Threads)
endif()

# ---------------------------------------------------------------- 激光节点
add_executable(shelf_laser_node src/laser_node.cpp)
target_link_libraries(shelf_laser_node shelf_scan_core)
ament_target_dependencies(shelf_laser_node rclcpp std_msgs)

# ---------------------------------------------------------------- 逻辑自测
add_executable(shelf_selftest src/shelf_selftest.cpp)
target_link_libraries(shelf_selftest shelf_scan_core)
ament_target_dependencies(shelf_selftest ament_index_cpp)

install(TARGETS shelf_scan_core
  EXPORT export_shelf_scan_pkg
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  shelf_mission_node
  shelf_gs_link_node
  shelf_code_reader_node
  shelf_laser_node
  shelf_selftest
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_shelf_scan_pkg HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs ament_index_cpp)

ament_package()
