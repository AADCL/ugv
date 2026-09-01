cmake_minimum_required(VERSION 3.0.2)
project(wheeltec_pointcloud_mapper)

add_compile_options(-std=c++14 -O2)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_srvs
)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud_mapper_node src/pointcloud_mapper_node.cpp)
target_link_libraries(pointcloud_mapper_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS pointcloud_mapper_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
