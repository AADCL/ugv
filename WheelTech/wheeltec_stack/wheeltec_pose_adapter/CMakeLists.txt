cmake_minimum_required(VERSION 3.0.2)

project(wheeltec_pose_adapter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
  nav_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/tf_to_odom.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
