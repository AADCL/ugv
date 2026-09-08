# Scout Mini 自主导航机器人使用文档 V5.1

> 本文用于日常建图、地图生成、重定位和导航。V5.1 使用实测雷达中心离地 `0.48 m` 和轮胎总高 `0.15 m`。

## 1. 使用前检查

登录 Scout Jetson 后执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/nvidia/livox_fastlio/devel/setup.bash
ip link show can0
rospack find scout_system_bringup
rospack find scout_terrain_filter
rospack find scout_2p5d_navigation
```

检查 Mid-360 网络：

```bash
ip -4 addr
ping -c 3 192.168.1.1
```

主机雷达网口通常为 `192.168.1.5/24`，雷达地址按设备实际编号核对。确认遥控器、急停、CAN 和供电正常。

## 2. 建图

### 2.1 建立 CAN

```bash
rosrun scout_bringup bringup_can2usb.bash
ip -details link show can0
```

### 2.2 唯一建图入口

```bash
roslaunch scout_system_bringup scout_mapping.launch \
  map_name:=factory_a
```

不要再启动另一套2.5D建图launch。默认入口启动：

- Livox 与 FAST-LIO；
- Scout CAN 底盘；
- 可逆贝叶斯静态PCD；
- `/fastlio_odom`车体中心轨迹保存；
- TF和pose adapter。

Patchwork++在线分类默认关闭，最终PGM和2.5D由`finalize_map.py`从同一贝叶斯静态PCD统一重建。启动约2秒后`/scout/static_scan`趋于稳定属于正常现象。

### 2.3 建图中检查

```bash
rostopic hz /cloud_registered
rostopic hz /scout/static_scan
rostopic hz /fastlio_odom
rostopic echo -n 1 /scout/dynamic_points   # 仅手动打开调试发布时
```

驾驶要求：

- 低速平稳移动，避免急加速和剧烈原地旋转；
- 对墙角、门口、坡道入口和坡顶从不同方向重复观察；
- 人员尽量不要紧跟车尾，人员离开后重新观察该区域；
- 坡道和平地连接处要完整覆盖，避免高程图边缘成为未知区；
- 地图名称不要包含空格或斜杠。

### 2.4 停止与自动保存

在建图终端按一次`Ctrl+C`，等待静态PCD和轨迹最终保存日志。不要直接断电或`kill -9`。

应生成：

```text
/home/nvidia/livox_fastlio/maps/factory_a/
├── filtered_camera_init.pcd
└── traversed_path_map.pcd
```

日常不需要手动调用 finish 服务。

## 3. 一次生成全部地图资产

```bash
rosrun scout_map_tools finalize_map.py factory_a
```

随后检查：

```bash
MAP_DIR=/home/nvidia/livox_fastlio/maps/factory_a
ls -lh "$MAP_DIR"
```

正常目录包含：

```text
raw_camera_init.pcd
traversed_path_map.pcd
public_map.pcd
terrain_ground_candidates_map.pcd
terrain_ground_map.pcd
terrain_obstacles_map.pcd
map_raw.pgm / map_raw.yaml
map.pgm / map.yaml
terrain_cost.pgm / terrain_cost.yaml
terrain_2p5d.yaml
terrain_2p5d_elevation.bin
terrain_2p5d_confidence.bin
terrain_2p5d_slope.bin
terrain_2p5d_traversability.bin
terrain_2p5d_residual.bin
terrain_2p5d_obstacle.bin
map_metadata.yaml
```

`map_raw.yaml`是正式导航静态占据图；`map.yaml`用于定位入口显示兼容。二者不预先膨胀。车体轨迹只在0.30 m半宽的真实扫掠走廊中补充自由证据，障碍始终覆盖自由证据。

不要单独移动或删除 `terrain_2p5d.yaml` 配套的 `.bin` 文件。只有明确要替换已经归档的 raw PCD 时才运行：

```bash
rosrun scout_map_tools finalize_map.py factory_a --replace-raw
```

## 4. 重定位

单独启动并保持运行：

```bash
roslaunch scout_system_bringup scout_localization.launch \
  map_name:=factory_a
```

等待 FAST-LIO 初始化，在 RViz 用 `2D Pose Estimate` 给出较准确初值。确认 NDT 收敛、实时点云与历史 PCD 重合：

```bash
rostopic hz /cloud_registered_base
rostopic hz /scout/odom
rosrun tf tf_echo map base_link
```

启动最初数秒 TF 未完全连通通常是 IMU 初始化尚未完成；持续断开才属于故障。

## 5. 导航

保持上一节的定位入口运行，另开终端：

```bash
source /home/nvidia/livox_fastlio/devel/setup.bash
roslaunch scout_navigation navigation_teb.launch \
  map_name:=factory_a
```

两终端分工固定：

```text
终端1：scout_localization.launch  # Livox、FAST-LIO、NDT、TF、底盘，只启动一次
终端2：navigation_teb.launch      # 地图、地形层、GlobalPlanner、TEB、move_base
```

导航入口不会重启重定位，也不会再次启动 Scout 底盘。地图名称必须一致。

发送目标前检查：

```bash
rosparam get /move_base/base_global_planner
rosparam get /move_base/base_local_planner
rosparam get /move_base/global_costmap/inflation_layer/inflation_radius
rosparam get /move_base/local_costmap/inflation_layer/inflation_radius
rostopic hz /terrain/patchwork_ground
rostopic hz /terrain/patchwork_nonground
rostopic hz /terrain/obstacle_points
rostopic hz /terrain/clearing_points
rostopic hz /scout/odom
rosrun tf tf_echo map base_link
```

预期输出：

```text
global_planner/GlobalPlanner
teb_local_planner/TebLocalPlannerROS
0.10
0.10
```

V5.1正式话题是`/terrain/obstacle_points`和`/terrain/clearing_points`。在RViz发送`2D Nav Goal`。坡道在全局图中产生连续软代价，局部障碍由当前帧Patchwork++与Terrain Guard判断，不查询保存高程。

## 6. 10 cm 膨胀的含义

- global costmap `inflation_radius=0.10 m`：静态障碍向外扩展的运行距离；
- local costmap `inflation_radius=0.10 m`：实时障碍向外扩展的运行距离；
- TEB `inflation_dist=0.10 m`：轨迹优化的软代价范围；
- footprint：Scout 实际车体 polygon，保持原值；
- TEB `min_obstacle_dist=0.15 m`：车体边界到障碍的期望净空，保持原值。

footprint 不是障碍膨胀。即使膨胀改小，规划器仍然不能让真实车体轮廓穿过障碍。

## 7. 雷达与轮胎高度

当前Patchwork++`sensor_height=0.48 m`，对应雷达中心到地面的实测垂直高度。刚性雷达Z偏移是0.20 m，因此`base_link_height_above_ground=0.28 m`。轮胎总高0.15 m，正式地图与在线障碍下限取0.08 m。

修改文件：

```text
scout_terrain_filter/config/patchworkpp_scout.yaml
scout_terrain_filter/config/terrain_guard_scout.yaml
scout_system_bringup/config/scout_geometry.yaml
```

不要把`base_link -> body`的`z=0.20 m`直接当成雷达离地高度或base_link离地高度。V5.1 Terrain Guard、PGM、2.5D和导航障碍阈值统一为0.08 m。

## 8. D435i

```bash
roslaunch scout_system_bringup D435I.launch
```

D435i 当前独立提供 RGB、Depth、CameraInfo 和相机 TF，不进入 Livox/FAST-LIO、NDT 或导航障碍链。不要重复启动同一相机。

## 9. 导航记录

先启动定位和导航，再运行：

```bash
roslaunch scout_navigation nav_logging.launch \
  tag:=factory_a_slope
```

结束时在日志终端按 `Ctrl+C`，结果位于：

```text
/home/nvidia/livox_fastlio/logs/navigation/<时间>_<标签>/
```

日志脚本只记录 TEB 正式链路，不再记录 DWA 诊断话题。

## 10. 快速排错

| 现象 | 检查顺序 |
|---|---|
| 无雷达点云 | 网口 IP -> 雷达供电 -> `/livox/lidar` -> Livox 配置 |
| 无 `/scout/static_scan` | `/cloud_registered` -> `/Odometry` -> mapper 日志；等待启动约 2 秒 |
| 建图时Patchwork++无输出 | 默认不启动，属于正常；仅诊断模式检查`/cloud_registered_terrain`与TF |
| 导航时Patchwork++无输出 | `/cloud_registered_terrain` frame -> `terrain_sensor` TF -> `sensor_height` |
| 地面大量进 nonground | 先核对雷达绝对离地高度，再核对重力对齐 TF 和姿态 |
| 人员留下轻微残影 | 人员离开后复扫；检查12次/2秒/60%晋升和8次/0.75秒清除参数 |
| finalize失败 | `filtered_camera_init.pcd`、轨迹和新工具是否完整；重新编译并source |
| 没有 `terrain_2p5d.yaml` | 不要使用 `--legacy-2d-only`；检查 `scout_2p5d_navigation` |
| 重定位没有 `map` TF | NDT 是否收敛；地图名和 `public_map.pcd` 是否正确 |
| 启动导航后定位消失 | 当前 launch 不应发生；用 `roslaunch --nodes` 检查是否部署了旧版导航文件 |
| 有全局路径但不走 | `/cmd_vel` -> `/scout/odom` -> 急停/遥控 -> local costmap -> TEB 状态 |
| 局部图几乎全占据 | 当前帧ground/nonground、`terrain_sensor` TF和0.08 m阈值 |
| 坡道变成墙 | `sensor_height`、重力对齐TF和当前帧分类；不要查询旧高程补偿 |
| 起点逃逸不倒车 | 这是安全拒绝；检查全局起点占用、局部走廊和后向区域新鲜点数 |

## 11. 正常停止顺序

1. 取消或停止发送导航目标；
2. 停止 `navigation_teb.launch`；
3. 停止 `scout_localization.launch`；
4. 建图模式等待mapper完成静态PCD和轨迹最终保存；
5. 最后关闭 CAN 和整车电源。
