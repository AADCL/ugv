#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash --extend
source /home/nvidia/r3live_ws/devel/setup.bash --extend
if [ -f /home/nvidia/lidar_camera_calib_ws/devel/setup.bash ]; then
  source /home/nvidia/lidar_camera_calib_ws/devel/setup.bash --extend
fi
export ROS_PACKAGE_PATH="/home/nvidia/realsense_ws/src:${ROS_PACKAGE_PATH}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/home/nvidia/realsense_ws/devel/lib"
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:/home/nvidia/realsense_ws/devel"
export PYTHONPATH="/home/nvidia/r3live_ws/src/scout_r3live_bringup/scripts:${PYTHONPATH}"
exec python3 /home/nvidia/r3live_ws/calibration_tools/calibrate.py "$@"
