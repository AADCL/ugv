#!/usr/bin/env bash
set -eo pipefail
task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source /home/nvidia/realsense_ws/devel/setup.bash
source /home/nvidia/livox_fastlio/devel/setup.bash
source "$task_workspace/devel/setup.bash"
# catkin's cached underlay omits the separately built camera workspace.
export ROS_PACKAGE_PATH="/home/nvidia/realsense_ws/src:${ROS_PACKAGE_PATH}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/home/nvidia/realsense_ws/devel/lib"
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:/home/nvidia/realsense_ws/devel"
exec roslaunch scout_r3live_bringup scout_r3live.launch "$@"
