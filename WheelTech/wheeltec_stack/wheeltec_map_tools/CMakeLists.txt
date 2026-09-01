cmake_minimum_required(VERSION 3.0.2)
project(wheeltec_map_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable(
  pcd_to_pgm_node
  src/pcd_to_pgm.cpp
)

target_link_libraries(
  pcd_to_pgm_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(
  pcd_transform_node
  src/pcd_transform.cpp
)

target_link_libraries(
  pcd_transform_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_install_python(
  PROGRAMS
    scripts/finalize_map.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
