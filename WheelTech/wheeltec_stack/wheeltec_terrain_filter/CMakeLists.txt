cmake_minimum_required(VERSION 3.0.2)
project(wheeltec_terrain_filter)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  message_filters
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_srvs
  tf2_ros
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs message_filters nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_srvs tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(terrain_guard_node src/terrain_guard_node.cpp)
target_link_libraries(terrain_guard_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(terrain_guard_node ${catkin_EXPORTED_TARGETS})

add_executable(terrain_map_accumulator_node src/terrain_map_accumulator_node.cpp)
target_link_libraries(terrain_map_accumulator_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(terrain_map_accumulator_node ${catkin_EXPORTED_TARGETS})

install(TARGETS terrain_guard_node terrain_map_accumulator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
