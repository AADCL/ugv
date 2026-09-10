#!/usr/bin/env bash
set -eo pipefail
task_assets="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_ws=/home/nvidia/lidar_camera_calib_ws
task_rev=061fdaa647fc806e59d73a8505a05a10dfcfdaa1
dpkg-query -W libceres-dev >/dev/null 2>&1 || sudo apt-get install -y libceres-dev
mkdir -p "$task_ws/src"
if [ ! -d "$task_ws/src/livox_camera_calib/.git" ]; then
  git clone https://github.com/hku-mars/livox_camera_calib.git "$task_ws/src/livox_camera_calib"
fi
if [ "$(git -C "$task_ws/src/livox_camera_calib" rev-parse HEAD)" != "$task_rev" ]; then
  if [ -n "$(git -C "$task_ws/src/livox_camera_calib" status --porcelain)" ]; then
    echo 'Different modified upstream revision; refusing overwrite' >&2; exit 1
  fi
  git -C "$task_ws/src/livox_camera_calib" checkout --detach "$task_rev"
fi
if git -C "$task_ws/src/livox_camera_calib" apply --reverse --check "$task_assets/scout-calibration.patch" 2>/dev/null; then
  echo 'Scout calibration patch already applied'
else
  git -C "$task_ws/src/livox_camera_calib" apply --check "$task_assets/scout-calibration.patch"
  git -C "$task_ws/src/livox_camera_calib" apply "$task_assets/scout-calibration.patch"
fi
source /home/nvidia/r3live_ws/devel/setup.bash --extend
cd "$task_ws"
catkin_make -j1 -DCMAKE_BUILD_TYPE=Release
mkdir -p /home/nvidia/r3live_ws/calibration_tools
cp "$task_assets/"*.py /home/nvidia/r3live_ws/calibration_tools/
cp "$task_assets/scout_calibrate.sh" /home/nvidia/r3live_ws/
chmod +x /home/nvidia/r3live_ws/scout_calibrate.sh
cp -a "$task_assets/../scout_r3live_bringup/." /home/nvidia/r3live_ws/src/scout_r3live_bringup/
chmod +x /home/nvidia/r3live_ws/src/scout_r3live_bringup/scripts/session.py
cd /home/nvidia/r3live_ws
catkin_make -j1 -DR3LIVE_BUILD_MESHING=OFF
/home/nvidia/r3live_ws/scout_calibrate.sh --help
