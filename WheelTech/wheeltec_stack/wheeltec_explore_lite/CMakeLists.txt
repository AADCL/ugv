cmake_minimum_required(VERSION 3.1)
project(wheeltec_explore_lite)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    costmap_2d
    geometry_msgs
    map_msgs
    move_base_msgs
    nav_msgs
    roscpp
    std_msgs
    tf
    visualization_msgs
)

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(wheeltec_explore
  src/costmap_client.cpp
  src/explore.cpp
  src/frontier_search.cpp
)
add_dependencies(wheeltec_explore ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheeltec_explore ${catkin_LIBRARIES})

#############
## Install ##
#############

# install nodes
install(TARGETS wheeltec_explore
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# The WheelTech exploration package owns the sole user-facing launch.
install(FILES UPSTREAM_AND_CHANGES.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_boundary_goal test/test_boundary_goal.cpp src/frontier_search.cpp)
  if(TARGET test_boundary_goal)
    target_link_libraries(test_boundary_goal ${catkin_LIBRARIES})
  endif()
endif()
