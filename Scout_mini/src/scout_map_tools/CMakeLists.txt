cmake_minimum_required(VERSION 3.0.2)
project(scout_map_tools)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)

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

add_executable(
  pcd_static_gate_node
  src/pcd_static_gate.cpp
)

target_link_libraries(
  pcd_static_gate_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(
  terrain_reclassify_node
  src/terrain_reclassify.cpp
)

target_link_libraries(
  terrain_reclassify_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_install_python(
  PROGRAMS
    scripts/finalize_map.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    pcd_to_pgm_node
    pcd_transform_node
    pcd_static_gate_node
    terrain_reclassify_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
