# Scout Mini 自主导航机器人开发实施文档 V4.3

> 平台：AgileX Scout Mini、Jetson、Ubuntu 20.04、ROS Noetic、Livox Mid-360、Intel RealSense D435i。
> Scout 车端工作空间：`/home/nvidia/livox_fastlio`。
> 本文按“复制什么、打开什么、编译什么、测试什么”组织，面向第一次接触本项目的开发者。

## 1. V4.3 正式架构

```text
建图
Mid-360 -> FAST-LIO -> /cloud_registered
                         |
                         +-> scout_pointcloud_mapper
                             （半径滤波 + 三维贝叶斯静态点判定）
                             +-> filtered_camera_init.pcd
                             +-> /scout/static_scan
                                  -> terrain_sensor 重力对齐
                                  -> Patchwork++ ground/nonground
                                  -> 分类 PCD 累积
                                  -> PGM + 2.5D高程/坡度

定位
public_map.pcd + 实时 /cloud_registered_base
  -> NDT-OMP -> map -> odom

导航
map_raw 静态占据 -------------------------------+
保存的坡度代价 -> TerrainCostmapLayer ----------+-> GlobalPlanner
实时点云 - 保存的同位置地面高程 -> 局部障碍/清除点 +-> local costmap -> TEB -> /cmd_vel
```

约束如下：

- FAST-LIO 输入不变，任何过滤点云都不能回灌 FAST-LIO。
- `scout_mapping.launch` 是唯一建图入口，完整 PCD、PGM 和高程坡度资产共享同一贝叶斯静态点源。
- `scout_localization.launch` 与 `navigation_teb.launch` 职责分离。导航入口不启动 Livox、FAST-LIO、NDT 或 Scout 底盘。
- PGM 仍决定墙体、固定障碍和未知区；坡度层只给已知自由格增加代价，不能把未知格改成自由格。
- 正式导航使用 GlobalPlanner + TEB。DWA 和旧 TerrainGlobalPlanner 不进入当前代码树。
- Patchwork++ 只在建图阶段分类地面；导航阶段以实时点相对保存地面高程判定障碍。
- 本版本目标为平地、连续坡道以及平地与坡道组合路段；不承诺楼梯语义或跨层地图。

## 2. 目录与包职责

将仓库 `Scout_mini/src/` 中的项目包复制到 `/home/nvidia/livox_fastlio/src/`。本地开发备份采用 `Scout_mini/src/src/` 的 catkin 工作空间布局，两者只相差一层工作空间目录。第三方包按固定版本另行获取，不要把 WheelTech 的底盘、外参和导航参数覆盖到 Scout。

| 包 | 需要复制或配置的内容 | 职责 |
|---|---|---|
| `scout_ros`、`ugv_sdk` | Scout 原包 | CAN 驱动、`/scout/odom` 和 `/cmd_vel` |
| `livox_ros_driver2`、`Livox-SDK2` | 上游依赖 | Mid-360 点云与 IMU |
| `FAST_LIO` | 上游包加 Scout 配置 | 本地激光惯性里程计 |
| `scout_system_bringup` | 整包 | 唯一建图入口、独立定位入口和几何真值 |
| `scout_tf_manager` | 整包 | `body -> base_link`、`base_link -> terrain_sensor` |
| `scout_pose_adapter` | 整包 | TF 转 `/fastlio_odom` |
| `scout_cloud_adapter` | 整包 | 按 launch 参数转换点云坐标系 |
| `scout_pointcloud_mapper` | 整包 | 贝叶斯静态点、完整 PCD 自动保存 |
| `patchwork-plusplus-ros` | 上游依赖 | 单帧地面/非地面分类 |
| `scout_terrain_filter` | 新增整包 | Patchwork++ 配置、诊断和分类点累积 |
| `scout_2p5d_navigation` | 新增整包 | 高程地图、坡度 costmap 插件、相对地面障碍 |
| `scout_map_tools` | 整包 | PCD 坐标变换、PGM 与 2.5D 资产生成 |
| `fast_lio_localization` | 现有包 | PCD 加载和 NDT 全局定位 |
| `scout_navigation` | 整包 | GlobalPlanner、TEB、costmap、测试与日志 |

Patchwork++ ROS 已验证版本与 WheelTech V4.3 保持一致：`f8c070bf2774b2f3ef622644a511bdfe3f2f27bb`。它必须位于 FAST-LIO 输出支路。

## 3. 环境与依赖

```bash
source /opt/ros/noetic/setup.bash
sudo apt-get update
sudo apt-get install -y \
  can-utils libyaml-cpp-dev \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
  ros-noetic-map-server ros-noetic-move-base \
  ros-noetic-global-planner ros-noetic-costmap-2d \
  ros-noetic-teb-local-planner ros-noetic-pluginlib \
  ros-noetic-jsk-recognition-msgs
```

Livox-SDK2 按官方流程安装后运行 `sudo ldconfig`。D435i 不是建图、定位或导航的必需输入。

## 4. FAST-LIO 与底盘保持原链路

打开 `FAST_LIO/config/mid360.yaml`，确认：

```yaml
common:
  lid_topic: /livox/lidar
  imu_topic: /livox/imu

publish:
  scan_publish_en: true
  dense_publish_en: true
  scan_bodyframe_pub_en: true

pcd_save:
  pcd_save_en: false
```

FAST-LIO 必须发布 `/Odometry`、`/cloud_registered`、`/cloud_registered_body` 和 `camera_init -> body`。原生 PCD 保存关闭，最终 PCD 由 `scout_pointcloud_mapper` 负责。

底盘入口继续使用：

```bash
rosrun scout_bringup bringup_can2usb.bash
roslaunch scout_bringup scout_mini_robot_base.launch \
  odom_topic_name:=/scout/odom pub_tf:=false
```

`pub_tf=false` 必须保留，避免底盘和 FAST-LIO 同时发布导航 TF。

## 5. TF 与实测高度

打开：

```text
scout_tf_manager/config/extrinsics.yaml
scout_system_bringup/config/scout_geometry.yaml
scout_terrain_filter/config/patchworkpp_scout.yaml
```

现有刚体外参保持 Scout 原值：Livox 相对 `base_link` 前 `0.25 m`、上 `0.20 m`、pitch `+45 deg`。发布关系为：

```text
map -> odom -> camera_init -> body -> base_link

                         base_link -> terrain_sensor
```

`terrain_sensor` 位于 Livox 原点但姿态与车体重力方向一致，Patchwork++ 不能直接在倾斜的 `body` 坐标中做地面分割。

当前 `patchworkpp_scout.yaml`：

```yaml
sensor_height: 0.48
```

该值是 Livox 中心到地面的实测垂直高度。它与 `base_link -> body` 的刚体 `z=0.20 m` 含义不同，不要互相替换。轮胎实测总高为 `0.15 m`，对应半径约 `0.075 m`。

## 6. 统一建图入口

打开 `scout_system_bringup/launch/scout_mapping.launch`。它必须一次启动：

1. Livox 驱动；
2. FAST-LIO；
3. `scout_pointcloud_mapper`；
4. Scout TF 与 pose adapter；
5. Scout CAN 底盘，`pub_tf=false`；
6. `scout_cloud_adapter`，将 `/scout/static_scan` 转为 `/cloud_registered_terrain`；
7. `scout_terrain_filter` 和分类点累积器。

关键参数：

```xml
<arg name="terrain_input_topic" default="/scout/static_scan" />
```

正式建图不得把该参数改为 `/cloud_registered_body`。贝叶斯静态点判定需要累计观测，启动后最初约 2 秒分类输出尚未稳定属于正常现象。

## 7. Patchwork++ 与分类地图

`scout_terrain_filter/launch/scout_terrain_filter.launch` 启动 Patchwork++ 和建图期诊断节点。输入已转换为 `terrain_sensor`，输出：

```text
/terrain/patchwork_ground
/terrain/patchwork_nonground
/terrain/ground_points
/terrain/obstacle_points
/terrain/status
```

`terrain_map_accumulator_node` 使用 `/Odometry` 将单帧分类点重新累积到 `camera_init`，生成：

```text
terrain_ground_camera_init.pcd
terrain_obstacles_camera_init.pcd
```

分类体素为 `0.05 m`；非地面障碍至少命中 3 帧且观察跨度达到 `0.4 s` 后保存，减少人员短时经过造成的残影。

Scout 按 0.15 m 轮胎直径、约 0.075 m 轮半径设置正式相对障碍阈值：

```yaml
obstacle_min_relative_height: 0.08
obstacle_max_relative_height: 1.50
```

该阈值与 WheelTech 的 0.08 m 不同，不能跨车型复制。

## 8. 地图最终生成

正常结束建图后运行：

```bash
rosrun scout_map_tools finalize_map.py factory_a
```

脚本顺序如下：

1. 保留或归档 `raw_camera_init.pcd`；
2. 使用 `scout_geometry.yaml` 生成 `public_map.pcd`；
3. 把 ground/nonground 分类 PCD 变换到 `map`；
4. 生成 `map_raw.pgm/yaml` 与兼容 `map.pgm/yaml`；
5. 构建 `terrain_2p5d.yaml` 及高程、置信度、坡度和代价层；
6. 写入 `map_metadata.yaml`。

只有确认要用新的 `filtered_camera_init.pcd` 覆盖已经归档的 raw PCD 时才加 `--replace-raw`。

`scout_map_tools/config/scout_raw.yaml`、`scout_nav.yaml` 和 `scout_terrain_cost.yaml` 采用分类地面高度作为投影基准。PGM 不做离线障碍膨胀，运行时统一由 costmap 膨胀。

## 9. 高程坡度与在线障碍

`scout_2p5d_navigation/config/terrain_builder.yaml` 当前基线：

```yaml
resolution: 0.10
fit_radius_m: 0.30
preferred_slope_deg: 5.0
max_slope_deg: 25.0
max_slope_cost: 80
obstacle_min_relative_height_m: 0.08
obstacle_max_relative_height_m: 1.50
```

每格保存地面高度，局部平面拟合坡度。5 度以内不增加坡度成本，5 到 25 度逐渐增加软代价；持续超过 25 度才成为致命坡度。小孔洞只有邻居充足时才填充，未知区域不能被坡度层改成自由区。

导航阶段的 `terrain_relative_obstacle_node` 读取实时 `/cloud_registered_terrain`，把每个点与保存的同位置高程比较：

- 高于局部地面 `0.08-1.50 m`：发布 `/terrain/elevation_obstacle_points`；
- 可作为射线清除的点：发布 `/terrain/elevation_clearing_points`；
- 没有可信保存地面：不擅自标成自由。

这条在线链路不运行 Patchwork++，从而避免坡道入口处先被固定 `base_link.z` 截取判为障碍的闭环问题。

## 10. 定位与导航入口必须分离

`scout_system_bringup/launch/scout_localization.launch` 启动 Livox、FAST-LIO、TF、NDT、地图显示和 Scout 底盘，并持续运行。

`scout_navigation/launch/navigation_teb.launch` 只启动：

```text
scout_terrain_cloud_adapter
scout_terrain_relative_obstacle_filter
scout_navigation_map_server
scout_terrain_map_server
move_base
```

导航入口不得出现第二个 Livox、`laserMapping`、NDT localizer、底盘驱动、Patchwork++ 或 terrain_guard。因此启动导航不会重启或抢占重定位节点。

## 11. 导航参数

`scout_navigation/config/global_costmap_slope.yaml`：静态层 + TerrainCostmapLayer + InflationLayer。

`scout_navigation/config/local_costmap_slope.yaml`：实时高程相对障碍 + InflationLayer。

`scout_navigation/config/teb_local_planner.yaml`：保留 Scout 原 footprint、速度和运动学参数。

本次统一修改：

```yaml
# global_costmap_slope.yaml
inflation_radius: 0.10

# local_costmap_slope.yaml
inflation_radius: 0.10

# teb_local_planner.yaml
inflation_dist: 0.10
```

`costmap_common.yaml` 的 polygon footprint 和 `footprint_padding: 0.03` 未改。TEB `min_obstacle_dist: 0.15` 也未改，它是轨迹净空，不是 costmap 障碍膨胀。

## 12. 编译顺序

先把 Patchwork++ 放入工作空间，再执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/nvidia/livox_fastlio

catkin_make --pkg scout_tf_manager scout_cloud_adapter \
  scout_pose_adapter scout_pointcloud_mapper \
  scout_terrain_filter -DCMAKE_BUILD_TYPE=Release -j1

catkin_make --pkg scout_2p5d_navigation scout_map_tools \
  scout_navigation scout_system_bringup \
  -DCMAKE_BUILD_TYPE=Release -j1

catkin_make -DCMAKE_BUILD_TYPE=Release -j1
source devel/setup.bash
```

如果提示找不到 `patchworkpp`，先确认其 ROS 包名和提交版本，不要从 launch 中删掉地形分割节点绕过编译问题。

## 13. 静态检查

```bash
source /home/nvidia/livox_fastlio/devel/setup.bash
rospack find scout_terrain_filter
rospack find scout_2p5d_navigation
rospack find scout_navigation

roslaunch --nodes scout_system_bringup scout_mapping.launch map_name:=check_map
roslaunch --nodes scout_system_bringup scout_localization.launch map_name:=check_map
roslaunch --nodes scout_navigation navigation_teb.launch map_name:=check_map
```

第三条只应列出纯导航层节点，不能列出 Livox、FAST-LIO、NDT 或 Scout 底盘。

## 14. 实车验证流程

### 14.1 建图

```bash
rosrun scout_bringup bringup_can2usb.bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a
```

```bash
rostopic hz /cloud_registered
rostopic hz /scout/static_scan
rostopic hz /terrain/patchwork_ground
rostopic hz /terrain/patchwork_nonground
```

正常 `Ctrl+C` 后确认完整 PCD 和两份分类 PCD 均存在。

### 14.2 生成资产

```bash
rosrun scout_map_tools finalize_map.py factory_a
MAP_DIR=/home/nvidia/livox_fastlio/maps/factory_a
test -s "$MAP_DIR/public_map.pcd"
test -s "$MAP_DIR/map_raw.yaml"
test -s "$MAP_DIR/terrain_2p5d.yaml"
```

### 14.3 重定位

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a
```

使用 RViz `2D Pose Estimate` 给出初值，确认 NDT 收敛、实时点云与地图重合，并检查：

```bash
rosrun tf tf_echo map base_link
rostopic hz /scout/odom
```

### 14.4 纯导航

保持定位入口运行，另开终端：

```bash
roslaunch scout_navigation navigation_teb.launch map_name:=factory_a
```

```bash
rosparam get /move_base/base_global_planner
rosparam get /move_base/base_local_planner
rosparam get /move_base/global_costmap/inflation_layer/inflation_radius
rosparam get /move_base/local_costmap/inflation_layer/inflation_radius
rostopic hz /terrain/elevation_obstacle_points
rostopic hz /terrain/elevation_clearing_points
```

预期规划器分别为 `global_planner/GlobalPlanner`、`teb_local_planner/TebLocalPlannerROS`，两处膨胀均为 `0.10`。

## 15. 验收清单

- [ ] `scout_mapping.launch` 是唯一建图入口。
- [ ] 完整 PCD 与分类 PCD 共用 `/scout/static_scan`。
- [ ] FAST-LIO 不读取任何过滤后点云。
- [ ] `map -> odom` 只有 NDT 一个发布者。
- [ ] Scout 底盘 `pub_tf=false`，速度反馈使用 `/scout/odom`。
- [ ] 定位入口持续运行，导航入口不重启 Livox、FAST-LIO、NDT 或底盘。
- [ ] GlobalPlanner + TEB 为唯一正式规划组合，没有 DWA。
- [ ] 全局、局部 costmap 和 TEB 软膨胀均为 `0.10 m`。
- [ ] Scout footprint、速度限制和刚体外参未被 WheelTech 参数覆盖。
- [ ] `sensor_height=0.48 m` 与实测雷达中心离地高度一致。
- [ ] 轮胎总高 `0.15 m`，正式相对障碍阈值为 `0.08 m`。
- [ ] 实车移动测试前先完成 Patchwork++ 平地输出和 TF 检查。

## 16. Git 同步

组织仓库为 `git@github.com:AADCL/ugv.git`，NX 克隆目录为 `/home/nrc19/github_upload/ugv_scout_wheeltech`。只同步 `Scout_mini/` 和必要的仓库根 README，不提交地图、PCD、bag、日志、密钥、`build/` 或 `devel/`。

本版本允许部署到 Scout 车端。先完成编译和静态话题检查；任何车辆移动测试仍需提前告知现场人员。
