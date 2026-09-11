#!/usr/bin/env bash
set -eo pipefail
task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source /home/nvidia/realsense_ws/devel/setup.bash
source /home/nvidia/livox_fastlio/devel/setup.bash
source "$task_workspace/devel/setup.bash"
# Camera, cv_bridge and image plugins are built in this same OpenCV overlay.
# Prepending the legacy camera workspace would restore mixed OpenCV ABIs.
test -f "$task_workspace/devel/lib/librealsense2_camera.so" || {
  echo 'Install the R3LIVE camera overlay before starting.' >&2; exit 1;
}
python3 "$task_workspace/check_opencv_runtime.py" "$task_workspace"
# Acquire before roslaunch registers its fixed session node; otherwise a second
# launch could shut down the first node before the Python conflict check runs.
if [ "${1:-}" = --session-node ]; then
  shift
  # Called as a node by the main workspace launch; preserve __name/__log remaps.
  exec flock --no-fork --nonblock --conflict-exit-code 75 "$task_workspace/.scout_r3live.lock" \
    rosrun scout_r3live_bringup session.py "$@"
fi
exec flock --nonblock --conflict-exit-code 75 "$task_workspace/.scout_r3live.lock" \
  roslaunch scout_r3live_bringup scout_r3live.launch "$@"
