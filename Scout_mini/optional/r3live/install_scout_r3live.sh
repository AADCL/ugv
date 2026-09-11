#!/usr/bin/env bash
# Reproducible separate overlay. Run on the Scout Jetson, not the Windows host.
set -eo pipefail
task_assets="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_ws="${R3LIVE_WORKSPACE:-/home/nvidia/r3live_ws}"
mkdir -p "$task_ws/src"
source /opt/ros/noetic/setup.bash
source /home/nvidia/realsense_ws/devel/setup.bash
source /home/nvidia/livox_fastlio/devel/setup.bash
if [ ! -d "$task_ws/src/r3live/.git" ]; then
  git clone https://github.com/hku-mars/r3live.git "$task_ws/src/r3live"
fi
if [ ! -d "$task_ws/src/vision_opencv/.git" ]; then
  git clone --branch noetic https://github.com/ros-perception/vision_opencv.git "$task_ws/src/vision_opencv"
fi
if [ ! -d "$task_ws/src/realsense-ros/.git" ]; then
  git clone --branch 2.3.2 --depth 1 https://github.com/IntelRealSense/realsense-ros.git "$task_ws/src/realsense-ros"
fi
if [ ! -d "$task_ws/src/image_transport_plugins/.git" ]; then
  git clone --branch noetic-devel https://github.com/ros-perception/image_transport_plugins.git "$task_ws/src/image_transport_plugins"
fi
task_r3rev=6143a38537f28cb36eb24e9bbe2e39c8f7967157
task_cvrev=cfabf72fb02970a661b5e68fbee503c5d9f94729
for task_pair in "r3live:$task_r3rev" "vision_opencv:$task_cvrev" \
    "realsense-ros:f400d682beee6c216052a419f419e95b797255ad" \
    "image_transport_plugins:cae592ce0816ea03bc38f0b6d27b6eae907feb10"; do
  task_repo="${task_pair%%:*}"; task_rev="${task_pair#*:}"
  if [ "$(git -C "$task_ws/src/$task_repo" rev-parse HEAD)" != "$task_rev" ]; then
    if [ -n "$(git -C "$task_ws/src/$task_repo" status --porcelain)" ]; then
      echo "Refusing to overwrite modified $task_repo" >&2; exit 1
    fi
    git -C "$task_ws/src/$task_repo" checkout --detach "$task_rev"
  fi
done
if git -C "$task_ws/src/realsense-ros" apply --reverse --check "$task_assets/scout-realsense-opencv.patch" 2>/dev/null; then
  echo 'Scout camera OpenCV patch already applied'
else
  git -C "$task_ws/src/realsense-ros" apply --check "$task_assets/scout-realsense-opencv.patch"
  git -C "$task_ws/src/realsense-ros" apply "$task_assets/scout-realsense-opencv.patch"
fi
if git -C "$task_ws/src/r3live" apply --reverse --check "$task_assets/scout-r3live.patch" 2>/dev/null; then
  echo 'Scout patch already applied'
else
  git -C "$task_ws/src/r3live" apply --check "$task_assets/scout-r3live.patch"
  git -C "$task_ws/src/r3live" apply "$task_assets/scout-r3live.patch"
fi
cp -a "$task_assets/scout_r3live_bringup" "$task_ws/src/"
cp "$task_assets/start_scout_r3live.sh" "$task_ws/"
cp "$task_assets/start_scout_r3live_test.sh" "$task_ws/"
# Install the reviewed trial only if no device-selected calibration exists.
if [ ! -e "$task_ws/config/accepted_20260911" ]; then
  mkdir -p "$task_ws/config"
  cp -a "$task_assets/config/accepted_20260911" "$task_ws/config/"
fi
cp "$task_assets/view_scout_r3live.sh" "$task_ws/"
cp "$task_assets/check_opencv_runtime.py" "$task_assets/observe_indoor.py" "$task_ws/"
chmod +x "$task_ws/start_scout_r3live.sh" "$task_ws/src/scout_r3live_bringup/scripts/session.py"
chmod +x "$task_ws/start_scout_r3live_test.sh"
chmod +x "$task_ws/view_scout_r3live.sh"
cd "$task_ws"
catkin_make -j1 -DCMAKE_BUILD_TYPE=Release -DR3LIVE_BUILD_MESHING=OFF
source "$task_ws/devel/setup.bash"
ldd "$task_ws/devel/lib/r3live/r3live_mapping" | grep -E 'cv_bridge|opencv|not found'
python3 "$task_assets/check_opencv_runtime.py" "$task_ws"
echo "Built official R3LIVE; launch with $task_ws/start_scout_r3live.sh"
