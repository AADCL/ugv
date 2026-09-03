cmake_minimum_required(VERSION 3.0.2)
project(scout_2p5d_navigation)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d geometry_msgs nav_msgs pcl_conversions pcl_ros
  pluginlib roscpp sensor_msgs tf2 tf2_geometry_msgs visualization_msgs)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scout_terrain_map scout_2p5d_plugins
  CATKIN_DEPENDS costmap_2d geometry_msgs nav_msgs pluginlib roscpp)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(scout_terrain_map src/terrain_map.cpp)
target_link_libraries(scout_terrain_map ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

add_library(scout_2p5d_plugins src/terrain_costmap_layer.cpp)
target_link_libraries(scout_2p5d_plugins scout_terrain_map ${catkin_LIBRARIES})

add_executable(terrain_map_builder_node src/terrain_map_builder_node.cpp)
target_link_libraries(terrain_map_builder_node scout_terrain_map ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp)

add_executable(terrain_map_server_node src/terrain_map_server_node.cpp)
target_link_libraries(terrain_map_server_node scout_terrain_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(terrain_relative_obstacle_node src/terrain_relative_obstacle_node.cpp)
target_link_libraries(terrain_relative_obstacle_node scout_terrain_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS scout_terrain_map scout_2p5d_plugins terrain_map_builder_node terrain_map_server_node terrain_relative_obstacle_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES scout_2p5d_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
