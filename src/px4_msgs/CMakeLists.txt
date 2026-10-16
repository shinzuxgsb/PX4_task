cmake_minimum_required(VERSION 3.8)
project(px4_msgs)

# PX4 uORB 消息定义。
# msg/ 和 srv/ 直接取自 PX4-Autopilot v1.15.4 源码，保证与飞控固件版本一致。
# 换 PX4 版本时，重跑 tools/sync_px4_msgs.ps1 同步即可。

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

file(GLOB msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
file(GLOB srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces)

ament_package()
