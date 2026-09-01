#!/usr/bin/env bash
set -uo pipefail

TAG="${1:-nav_test}"
TAG="$(printf '%s' "${TAG}" | tr -cs 'A-Za-z0-9_.-' '_')"
PKG_DIR="$(rospack find scout_navigation)"
STAMP="$(date +%Y%m%d_%H%M%S)"
ROOT_DIR="${HOME}/livox_fastlio/logs/navigation"
RUN_DIR="${ROOT_DIR}/${STAMP}_${TAG}"
BAG_PID=""
FINISHED=0

if ! rosnode list >/dev/null 2>&1; then
  echo "[NAV_LOG][ERROR] ROS master is unavailable. Start localization/navigation first."
  exit 1
fi

mkdir -p "${RUN_DIR}/config_snapshot" "${RUN_DIR}/launch_snapshot"
printf '%s\n' "${RUN_DIR}" > "${ROOT_DIR}/LAST_RUN"

cp -a "${PKG_DIR}/config/." "${RUN_DIR}/config_snapshot/" 2>/dev/null || true
cp -a "${PKG_DIR}/launch/." "${RUN_DIR}/launch_snapshot/" 2>/dev/null || true
rosparam dump "${RUN_DIR}/move_base_params.yaml" /move_base 2>/dev/null || true
rosparam get /move_base/base_local_planner > "${RUN_DIR}/local_planner.txt" 2>/dev/null || true
rosparam get /scout_geometry > "${RUN_DIR}/scout_geometry.yaml" 2>/dev/null || true
rosnode list > "${RUN_DIR}/rosnode_list.txt" 2>/dev/null || true
rostopic list -v > "${RUN_DIR}/rostopic_list_verbose.txt" 2>/dev/null || true
rosservice list > "${RUN_DIR}/rosservice_list.txt" 2>/dev/null || true
date --iso-8601=seconds > "${RUN_DIR}/started_at.txt"
df -h "${HOME}" > "${RUN_DIR}/disk_before.txt" 2>/dev/null || true

if ! rosnode list 2>/dev/null | grep -qx '/move_base'; then
  echo "[NAV_LOG][WARN] /move_base is not currently visible. The bag will still start, but navigation data may be incomplete."
fi

# 同时保留 DWA 与 TEB 相关 topic。
# 当前使用哪一个 planner，就会有哪一组 topic 实际产生消息；另一组为空不影响 rosbag。
TOPICS=(
  /tf
  /tf_static
  /rosout_agg
  /cmd_vel
  /scout/odom
  /fastlio_odom
  /move_base_simple/goal
  /move_base/status
  /move_base/goal
  /move_base/cancel
  /move_base/feedback
  /move_base/result
  /move_base/GlobalPlanner/plan

  /move_base/DWAPlannerROS/global_plan
  /move_base/DWAPlannerROS/local_plan
  /move_base/DWAPlannerROS/trajectory_cloud
  /move_base/DWAPlannerROS/cost_cloud
  /move_base/DWAPlannerROS/parameter_updates

  /move_base/TebLocalPlannerROS/global_plan
  /move_base/TebLocalPlannerROS/local_plan
  /move_base/TebLocalPlannerROS/teb_poses
  /move_base/TebLocalPlannerROS/teb_markers
  /move_base/TebLocalPlannerROS/teb_feedback
  /move_base/TebLocalPlannerROS/obstacles
  /move_base/TebLocalPlannerROS/via_points
  /move_base/TebLocalPlannerROS/parameter_updates

  /move_base/local_costmap/obstacle_layer/parameter_updates
  /move_base/local_costmap/inflation_layer/parameter_updates
  /move_base/global_costmap/inflation_layer/parameter_updates
  /move_base/parameter_updates
  /move_base/local_costmap/costmap
  /move_base/local_costmap/costmap_updates
  /move_base/local_costmap/footprint
  /move_base/global_costmap/costmap
  /move_base/global_costmap/costmap_updates
  /nav_static_map
  /cloud_registered_body
)

finish_session() {
  if [[ "${FINISHED}" -eq 1 ]]; then
    return
  fi
  FINISHED=1
  trap - INT TERM EXIT

  echo
  echo "[NAV_LOG] stopping rosbag..."
  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi

  date --iso-8601=seconds > "${RUN_DIR}/ended_at.txt"
  df -h "${HOME}" > "${RUN_DIR}/disk_after.txt" 2>/dev/null || true
  du -sh "${RUN_DIR}" > "${RUN_DIR}/run_size.txt" 2>/dev/null || true

  : > "${RUN_DIR}/rosbag_info.txt"
  shopt -s nullglob
  BAGS=("${RUN_DIR}"/*.bag)
  shopt -u nullglob
  if [[ "${#BAGS[@]}" -eq 0 ]]; then
    echo "[NAV_LOG][ERROR] no finalized .bag file found in ${RUN_DIR}" | tee -a "${RUN_DIR}/analysis_console.txt"
  else
    for bag in "${BAGS[@]}"; do
      echo "===== ${bag} =====" >> "${RUN_DIR}/rosbag_info.txt"
      rosbag info "${bag}" >> "${RUN_DIR}/rosbag_info.txt" 2>&1 || true
    done

    echo "[NAV_LOG] recording stopped"
    echo "[NAV_LOG] analyzing..."
    python3 "${PKG_DIR}/scripts/analyze_nav_bag.py" "${RUN_DIR}" 2>&1 | tee "${RUN_DIR}/analysis_console.txt" || true
  fi

  echo
  echo "[NAV_LOG] DONE"
  echo "[NAV_LOG] run_dir=${RUN_DIR}"
  echo "[NAV_LOG] summary=${RUN_DIR}/summary.txt"
}

trap finish_session INT TERM EXIT

echo "[NAV_LOG] run_dir=${RUN_DIR}"
echo "[NAV_LOG] DWA/TEB compatible navigation log enabled (including /cloud_registered_body)"
echo "[NAV_LOG] perform the test now; Ctrl+C this launch when finished"

rosbag record --lz4 --split --size=2048 -O "${RUN_DIR}/navigation" "${TOPICS[@]}" &
BAG_PID=$!
wait "${BAG_PID}" 2>/dev/null || true
finish_session

