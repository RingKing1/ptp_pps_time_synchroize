cmake_minimum_required(VERSION 3.8)
project(sync_export)

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
find_package(beidou_ins_driver REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(SQLite3 REQUIRED)

add_executable(sync_export_node src/sync_export_node.cpp)
add_executable(test_sync_node src/test_sync_node.cpp)

ament_target_dependencies(sync_export_node
  rclcpp
  sensor_msgs
  message_filters
  cv_bridge
  pcl_conversions
  autoware_localization_msgs
  beidou_ins_driver
)
ament_target_dependencies(test_sync_node
  rclcpp
  sensor_msgs
  message_filters
  autoware_localization_msgs
  beidou_ins_driver
)

target_include_directories(sync_export_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${SQLite3_INCLUDE_DIRS}
)

target_link_libraries(sync_export_node
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  ${SQLite3_LIBRARIES}
)
target_link_libraries(test_sync_node yaml-cpp)

install(TARGETS sync_export_node test_sync_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
