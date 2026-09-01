cmake_minimum_required(VERSION 3.0.2)
project(scout_cloud_adapter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  cloud_frame_adapter_node
  src/cloud_frame_adapter.cpp
)

target_link_libraries(
  cloud_frame_adapter_node
  ${catkin_LIBRARIES}
)

