cmake_minimum_required(VERSION 3.8)
project(my_carto_pkg)

find_package(ament_cmake REQUIRED)

# 只做"安装文件"：内容来自队友交接包 03_slam/（amphi.lua、fly.urdf、launch 等）
install(DIRECTORY launch urdf configuration_files
  DESTINATION share/${PROJECT_NAME})

ament_package()
