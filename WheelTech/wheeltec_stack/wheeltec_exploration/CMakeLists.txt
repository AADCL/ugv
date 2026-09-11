cmake_minimum_required(VERSION 3.0.2)
project(wheeltec_exploration)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d diagnostic_msgs geometry_msgs nav_msgs
  pluginlib roscpp rospy sensor_msgs std_msgs std_srvs tf2
  tf2_ros tf2_geometry_msgs nav_core teb_local_planner)
find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES wheeltec_exploration_plugins
  CATKIN_DEPENDS costmap_2d diagnostic_msgs geometry_msgs
                 nav_msgs pluginlib roscpp rospy sensor_msgs std_msgs
                 std_srvs tf2 tf2_ros tf2_geometry_msgs nav_core teb_local_planner)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(wheeltec_exploration_plugins src/forward_teb_planner.cpp src/observed_obstacle_layer.cpp)
target_link_libraries(wheeltec_exploration_plugins ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_supervisor_start.py)
  catkin_add_nosetests(test/test_observed_map_memory.py)
endif()

install(TARGETS wheeltec_exploration_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES wheeltec_exploration_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/explore_supervisor.py scripts/check_exploration.py scripts/check_navigation_clearance.py
                 scripts/frontier_costmap_adapter.py scripts/observed_map_memory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
