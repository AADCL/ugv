cmake_minimum_required(VERSION 3.0.2)
project(scout_odom_fusion)
find_package(catkin REQUIRED COMPONENTS rospy nav_msgs diagnostic_msgs)
catkin_package()
catkin_install_python(PROGRAMS scripts/fusion_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  find_package(PythonInterp 3 REQUIRED)
  add_test(NAME scout_fusion_guard_unit
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_guard.py)
endif()
