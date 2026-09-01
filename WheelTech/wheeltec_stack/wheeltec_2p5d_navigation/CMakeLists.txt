cmake_minimum_required(VERSION 3.0.2)
project(wheeltec_2p5d_navigation)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d geometry_msgs nav_core nav_msgs pcl_conversions pcl_ros
  pluginlib roscpp sensor_msgs tf2 tf2_geometry_msgs visualization_msgs)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wheeltec_terrain_map wheeltec_2p5d_plugins
  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(wheeltec_terrain_map src/terrain_map.cpp)
target_link_libraries(wheeltec_terrain_map ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

add_library(wheeltec_2p5d_plugins
  src/terrain_global_planner.cpp
  src/terrain_costmap_layer.cpp)
target_link_libraries(wheeltec_2p5d_plugins wheeltec_terrain_map ${catkin_LIBRARIES})

add_executable(terrain_map_builder_node src/terrain_map_builder_node.cpp)
target_link_libraries(terrain_map_builder_node wheeltec_terrain_map ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp)

add_executable(terrain_map_server_node src/terrain_map_server_node.cpp)
target_link_libraries(terrain_map_server_node wheeltec_terrain_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS wheeltec_terrain_map wheeltec_2p5d_plugins terrain_map_builder_node terrain_map_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES wheeltec_2p5d_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/replay_terrain_bags.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
