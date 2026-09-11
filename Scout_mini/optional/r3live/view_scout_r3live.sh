#!/usr/bin/env bash
# RViz uses distro plugins/OpenCV, isolated from the estimator's C++ overlay.
set -eo pipefail
task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_config="${1:-$task_workspace/src/scout_r3live_bringup/config/Scout_R3LIVE.rviz}"
exec env -u LD_LIBRARY_PATH -u CMAKE_PREFIX_PATH -u ROS_PACKAGE_PATH -u PYTHONPATH \
  bash --noprofile --norc -c 'source /opt/ros/noetic/setup.bash; exec rviz -d "$1"' \
  scout-r3live-view "$task_config"
