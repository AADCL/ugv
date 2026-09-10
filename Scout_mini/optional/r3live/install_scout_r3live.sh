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
task_r3rev=6143a38537f28cb36eb24e9bbe2e39c8f7967157
task_cvrev=cfabf72fb02970a661b5e68fbee503c5d9f94729
for task_pair in "r3live:$task_r3rev" "vision_opencv:$task_cvrev"; do
  task_repo="${task_pair%%:*}"; task_rev="${task_pair#*:}"
  if [ "$(git -C "$task_ws/src/$task_repo" rev-parse HEAD)" != "$task_rev" ]; then
    if [ -n "$(git -C "$task_ws/src/$task_repo" status --porcelain)" ]; then
      echo "Refusing to overwrite modified $task_repo" >&2; exit 1
    fi
    git -C "$task_ws/src/$task_repo" checkout --detach "$task_rev"
  fi
done
if git -C "$task_ws/src/r3live" apply --reverse --check "$task_assets/scout-r3live.patch" 2>/dev/null; then
  echo 'Scout patch already applied'
else
  git -C "$task_ws/src/r3live" apply --check "$task_assets/scout-r3live.patch"
  git -C "$task_ws/src/r3live" apply "$task_assets/scout-r3live.patch"
fi
cp -a "$task_assets/scout_r3live_bringup" "$task_ws/src/"
cp "$task_assets/start_scout_r3live.sh" "$task_ws/"
chmod +x "$task_ws/start_scout_r3live.sh" "$task_ws/src/scout_r3live_bringup/scripts/session.py"
cd "$task_ws"
catkin_make -j1 -DCMAKE_BUILD_TYPE=Release -DR3LIVE_BUILD_MESHING=OFF
source "$task_ws/devel/setup.bash"
ldd "$task_ws/devel/lib/r3live/r3live_mapping" | grep -E 'cv_bridge|opencv|not found'
echo "Built official R3LIVE; launch with $task_ws/start_scout_r3live.sh"
