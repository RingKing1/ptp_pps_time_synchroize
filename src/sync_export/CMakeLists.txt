cmake_minimum_required(VERSION 3.8)
project(sync_export)

# 默认编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(pcl_conversions REQUIRED)
find_package(autoware_localization_msgs REQUIRED)

add_executable(sync_export_node src/sync_export_node.cpp)

ament_target_dependencies(sync_export_node
  rclcpp
  sensor_msgs
  message_filters
  cv_bridge
  pcl_conversions
  autoware_localization_msgs
)

target_include_directories(sync_export_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(sync_export_node
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS sync_export_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
