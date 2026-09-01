cmake_minimum_required(VERSION 3.0.2)
project(scout_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  move_base_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/global_plan_tester.py
    scripts/analyze_nav_bag.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/nav_log_session.sh
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

