#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/nrc19/livox_fastlio/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311

bags=("$@")
if [ "${#bags[@]}" -eq 0 ]; then
  bags=(
    /home/nrc19/2026-09-01-13-25-39.bag
    /home/nrc19/2026-09-01-13-27-32.bag
    /home/nrc19/2026-09-01-13-33-12.bag
    /home/nrc19/2026-09-01-13-35-01_first65s.bag
  )
fi
names=(terrain_stairs_up terrain_stairs_down terrain_ramp_down terrain_ramp_up)
if [ "${#bags[@]}" -gt "${#names[@]}" ]; then
  echo "At most four named terrain bags are supported" >&2
  exit 2
fi

roscore >/tmp/wheeltec_2p5d_bag_roscore.log 2>&1 &
task_core_pid=$!
for _ in $(seq 1 30); do
  rosparam get /run_id >/dev/null 2>&1 && break
  sleep 0.2
done
roslaunch wheeltec_tf_manager tf_manager.launch \
  >/tmp/wheeltec_2p5d_bag_tf.log 2>&1 &
task_tf_pid=$!

cleanup_nodes() {
  rosnode kill /wheeltec_bag_terrain_adapter /wheeltec_patchworkpp \
    /wheeltec_terrain_guard /wheeltec_terrain_map_accumulator \
    >/dev/null 2>&1 || true
  kill "${task_adapter_pid:-}" "${task_filter_pid:-}" \
    "${task_accumulator_pid:-}" >/dev/null 2>&1 || true
}
cleanup_all() {
  cleanup_nodes
  kill "$task_tf_pid" "$task_core_pid" >/dev/null 2>&1 || true
}
trap cleanup_all EXIT
sleep 2
rosnode ping -c 1 /wheeltec_tf_manager >/dev/null
rosparam set /use_sim_time true

for ((i=0; i<${#bags[@]}; ++i)); do
  bag=${bags[$i]}
  name=${names[$i]}
  output=/home/nrc19/livox_fastlio/maps/$name
  mkdir -p "$output"

  roslaunch wheeltec_cloud_adapter cloud_adapter.launch \
    node_name:=wheeltec_bag_terrain_adapter \
    output_topic:=/cloud_registered_terrain target_frame:=terrain_sensor \
    >"/tmp/${name}_adapter.log" 2>&1 &
  task_adapter_pid=$!
  roslaunch wheeltec_terrain_filter wheeltec_terrain_filter.launch \
    >"/tmp/${name}_filter.log" 2>&1 &
  task_filter_pid=$!
  roslaunch wheeltec_terrain_filter terrain_map_accumulator.launch \
    map_name:="$name" >"/tmp/${name}_accumulator.log" 2>&1 &
  task_accumulator_pid=$!
  sleep 2

  rosbag play "$bag" --clock -r 1.5 \
    --topics /cloud_registered_body /Odometry \
    >"/tmp/${name}_bag.log" 2>&1
  rosservice call /wheeltec_terrain_map_accumulator/save_map \
    >"$output/save.txt"
  cleanup_nodes

  rosrun wheeltec_map_tools pcd_transform_node \
    _input_pcd:="$output/terrain_ground_camera_init.pcd" \
    _output_pcd:="$output/terrain_ground_map.pcd" \
    _x:=0.10 _y:=0 _z:=0.15 _roll_deg:=0 _pitch_deg:=20 _yaw_deg:=0
  rosrun wheeltec_map_tools pcd_transform_node \
    _input_pcd:="$output/terrain_obstacles_camera_init.pcd" \
    _output_pcd:="$output/terrain_obstacles_map.pcd" \
    _x:=0.10 _y:=0 _z:=0.15 _roll_deg:=0 _pitch_deg:=20 _yaw_deg:=0

  rosparam load "$(rospack find wheeltec_2p5d_navigation)/config/terrain_builder.yaml" \
    /wheeltec_terrain_map_builder
  rosrun wheeltec_2p5d_navigation terrain_map_builder_node \
    __name:=wheeltec_terrain_map_builder \
    _ground_pcd:="$output/terrain_ground_map.pcd" \
    _obstacle_pcd:="$output/terrain_obstacles_map.pcd" \
    _output_yaml:="$output/terrain_2p5d.yaml" \
    >"$output/build_2p5d.log" 2>&1
  echo "[OK] $name -> $output/terrain_2p5d.yaml"
done

echo "WHEELTEC_2P5D_FOUR_BAGS_DONE"
