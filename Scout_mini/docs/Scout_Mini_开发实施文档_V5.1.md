# Scout Mini 自主导航机器人开发实施文档 V5.1

> 平台：AgileX Scout Mini、Jetson、Ubuntu 20.04、ROS Noetic、Livox Mid-360、Intel RealSense D435i。
> Scout 车端工作空间：`/home/nvidia/livox_fastlio`。
> 本文按“复制什么、打开什么、编译什么、测试什么”组织，面向第一次接触本项目的开发者。

## 1. V5.1 正式架构

```text
建图
Mid-360 -> FAST-LIO -> /cloud_registered
                         |
                         +-> scout_pointcloud_mapper
                             （半径滤波 + 可逆三维贝叶斯静态点判定）
                             +-> filtered_camera_init.pcd
                             +-> traversed_path_map.pcd
                                  -> finalize_map.py
                                  -> PMF保守种子 + 鲁棒局部平面生长
                                  -> ground/nonground + PGM + 2.5D高程/坡度

定位
public_map.pcd + 实时 /cloud_registered_base
  -> NDT-OMP -> map -> odom

导航
map_raw 静态占据 -------------------------------+
保存的坡度代价 -> TerrainCostmapLayer ----------+-> GlobalPlanner
实时点云 -> terrain_sensor -> Patchwork++ -------+
                         -> Terrain Guard障碍/清除点 -> local costmap -> TEB -> /cmd_vel
```

约束如下：

- FAST-LIO 输入不变，任何过滤点云都不能回灌 FAST-LIO。
- `scout_mapping.launch`是唯一建图入口；默认只维护贝叶斯静态PCD和车体轨迹，分类PCD、PGM和高程坡度在收尾阶段从同一静态点源统一重建。
- `scout_localization.launch` 与 `navigation_teb.launch` 职责分离。导航入口不启动 Livox、FAST-LIO、NDT 或 Scout 底盘。
- PGM 仍决定墙体、固定障碍和未知区；坡度层只给已知自由格增加代价，不能把未知格改成自由格。
- 正式导航使用 GlobalPlanner + TEB。DWA 和旧 TerrainGlobalPlanner 不进入当前代码树。
- Patchwork++在正式导航阶段对当前帧直接分类；保存高程只提供全局坡度软代价，不再参与局部障碍判定。
- 建图期Patchwork++累积仅作为`enable_online_terrain_diagnostics:=true`的回归诊断支路，默认关闭以降低Jetson负载。
- `StartEscapeRecovery`已安装但默认关闭；完成实车静态覆盖验证后，才可在全局起点确定为致命占据、局部倒车走廊安全且后向点云覆盖新鲜时启用低速倒车。
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
| `scout_pointcloud_mapper` | 整包 | 可逆贝叶斯静态点、完整PCD与`base_link`轨迹自动保存 |
| `patchwork-plusplus-ros` | 上游依赖 | 单帧地面/非地面分类 |
| `scout_terrain_filter` | 整包 | Patchwork++配置、当前帧Terrain Guard和可选分类点累积 |
| `scout_2p5d_navigation` | 整包 | 高程坡度costmap插件、地图服务和安全逃逸恢复 |
| `scout_map_tools` | 整包 | PCD变换、鲁棒地面重建、轨迹自由走廊、PGM与2.5D生成 |
| `fast_lio_localization` | 现有包 | PCD 加载和 NDT 全局定位 |
| `scout_navigation` | 整包 | GlobalPlanner、TEB、costmap、测试与日志 |

Patchwork++ ROS 已验证版本与 WheelTech V5.1 保持一致：`f8c070bf2774b2f3ef622644a511bdfe3f2f27bb`。它必须位于 FAST-LIO 输出支路。

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

该值是Livox中心到地面的实测垂直高度。它与`base_link -> body`的刚体`z=0.20 m`含义不同。由两者得到`base_link_height_above_ground=0.28 m`，必须写在`scout_geometry.yaml`，供离线地面种子使用。轮胎实测总高为`0.15 m`，对应半径约`0.075 m`。

## 6. 统一建图入口

打开 `scout_system_bringup/launch/scout_mapping.launch`。它必须一次启动：

1. Livox 驱动；
2. FAST-LIO；
3. `scout_pointcloud_mapper`；
4. Scout TF 与 pose adapter；
5. Scout CAN 底盘，`pub_tf=false`；
6. 仅当`enable_online_terrain_diagnostics:=true`时，再启动cloud adapter、Patchwork++和分类点累积器。

关键参数：

```xml
<arg name="terrain_input_topic" default="/scout/static_scan" />
```

默认参数`enable_online_terrain_diagnostics=false`。正式交付不依赖在线分类PCD；贝叶斯静态点判定需要累计观测，启动后最初约2秒`/scout/static_scan`尚未稳定属于正常现象。

历史入口`scout_system.launch`保留为兼容别名，但其内部只能include`scout_mapping.launch`并透传`map_name`和诊断开关，不能再直接include上游`mapping_mid360.launch`。这样旧操作命令也不会绕过mapper、pose adapter或统一TF链。

## 7. 可逆贝叶斯地图与轨迹证据

必须修改以下文件：

```text
scout_pointcloud_mapper/src/pointcloud_mapper_node.cpp
scout_pointcloud_mapper/config/mapper.yaml
scout_pointcloud_mapper/launch/pointcloud_mapper.launch
```

0.20 m贝叶斯体素是唯一动态/静态权威。候选晋升必须同时满足：至少12次命中、观察跨度至少2秒、命中率至少60%、占据概率达到0.75。晋升后使用单独的保留条件；只有连续自由射线至少8次且持续0.75秒，才撤销该体素的整代0.05 m精细点，避免旧几何复活。

射线使用完整三维DDA遍历，`ray_stride=2`、最大20 m；它只处理FAST-LIO输出，不反馈到前端。mapper另订阅`/fastlio_odom`，每移动0.05 m保存一个`base_link`中心点：

```text
filtered_camera_init.pcd
traversed_path_map.pcd
```

`publish_dynamic_points=false`保持默认，避免Jetson调试带宽。`self_filter=false`保持默认，直到实测车体包围盒确认。

需要回归旧在线分类时，显式运行：

```bash
roslaunch scout_system_bringup scout_mapping.launch \
  map_name:=factory_a enable_online_terrain_diagnostics:=true
```

这时才会生成`terrain_ground_camera_init.pcd`与`terrain_obstacles_camera_init.pcd`，但它们不是默认交付输入。

## 8. 地图最终生成

正常结束建图后运行：

```bash
rosrun scout_map_tools finalize_map.py factory_a
```

脚本顺序如下：

1. 归档或核对`raw_camera_init.pcd`和`traversed_path_map.pcd`；
2. 使用`scout_geometry.yaml`生成`public_map.pcd`；
3. `terrain_reclassify_node`从最终贝叶斯PCD提取保守PMF种子；
4. 以二维支持、平面内点率、RMSE、坡度和连接连续性执行鲁棒局部平面生长；
5. 仅在原点云附近有XY观测时拟合稠密地面，并按相对地面0.08～1.50 m重建障碍；
6. PGM把0.30 m半宽的真实车体轨迹作为自由证据，障碍证据优先；
7. 生成`map_raw.*`、`map.*`、分类PCD、`terrain_2p5d.yaml`六层文件和`map_metadata.yaml`。

只有确认要用新的 `filtered_camera_init.pcd` 覆盖已经归档的 raw PCD 时才加 `--replace-raw`。

新增/修改文件为`terrain_reclassify.cpp/.yaml`、`pcd_static_gate.cpp`、`pcd_to_pgm.cpp`、`finalize_map.py`和三个Scout地图配置。PGM不做离线障碍膨胀；`free_evidence_radius_m=0.30`来自Scout半宽0.295 m，不复制WheelTech的0.20 m。

## 9. 高程坡度与当前帧局部障碍

`scout_2p5d_navigation/config/terrain_builder.yaml` 当前基线：

```yaml
resolution: 0.10
fit_radius_m: 0.30
preferred_slope_deg: 5.0
max_slope_deg: 25.0
max_slope_cost: 80
obstacle_min_relative_height_m: 0.08
obstacle_max_relative_height_m: 1.50
fuse_static_obstacles: false
```

每格保存地面高度，局部平面拟合坡度。5 度以内不增加坡度成本，5 到 25 度逐渐增加软代价；持续超过 25 度才成为致命坡度。小孔洞只有邻居充足时才填充，未知区域不能被坡度层改成自由区。

`fuse_static_obstacles=false`表示静态致命障碍只由PGM StaticLayer负责，2.5D层只叠加坡度软代价，避免同一障碍在粗栅格中二次扩大。

导航阶段不再启动`terrain_relative_obstacle_node`。`navigation_teb.launch`将实时点云转换到重力对齐`terrain_sensor`，再启动Patchwork++和Terrain Guard：

- `/terrain/obstacle_points`写入local costmap marking；
- `/terrain/clearing_points`只执行raytrace clearing；
- `obstacle_min_relative_height=0.08 m`来自Scout 0.15 m轮径；
- `observation_persistence=0`避免历史局部点云拖影。

局部障碍由当前扫描自行估计地面，不查询保存高程，因此不会随`map -> odom`重定位修正整体漂移。

## 10. 定位与导航入口必须分离

`scout_system_bringup/launch/scout_localization.launch` 启动 Livox、FAST-LIO、TF、NDT、地图显示和 Scout 底盘，并持续运行。

`scout_navigation/launch/navigation_teb.launch` 只启动：

```text
scout_terrain_cloud_adapter
scout_navigation_patchworkpp
scout_navigation_terrain_guard
scout_navigation_map_server
scout_terrain_map_server
move_base
```

导航入口不得出现第二个Livox、`laserMapping`、NDT localizer或底盘驱动。这里的Patchwork++/terrain_guard只处理当前帧局部碰撞，不发布TF，不会抢占重定位节点。

## 11. 导航参数

`scout_navigation/config/global_costmap_slope.yaml`：静态层 + TerrainCostmapLayer + InflationLayer。

`scout_navigation/config/local_costmap_slope.yaml`：当前帧marking/clearing双源 + InflationLayer。

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

`move_base_slope_teb.yaml`安装并配置了`StartEscapeRecovery`，但`recovery_behavior_enabled=false`保持Scout现有导航行为不变。完成现场静态覆盖验收后才能手动启用。启用时逃逸速度0.05 m/s、目标0.30 m；后向覆盖区按雷达前置0.25 m换算为`terrain_sensor`坐标X=-1.05～-0.55 m、半宽0.36 m。覆盖还必须满足至少20个新鲜点、X跨度0.20 m且左右各至少5点；全局起点不是明确致命碰撞（未知区和图外均拒绝）、局部走廊不安全或1.5秒内位移不足0.02 m时，插件拒绝/停止倒车。TEB自身`max_vel_x_backwards=0`保持原值。

### 11.1 功能包逐包复现索引

下表是从空工作空间复现本项目时的最小闭环。上游包只做版本集成；项目包按所列文件实现，不要把WheelTech同名配置覆盖进来。

| 功能包 | 依赖/性质 | 必须复制或修改的文件 | 编译目标 | 启动与验证 | 故障边界 |
|---|---|---|---|---|---|
| `ugv_sdk`、`scout_ros/{scout_base,scout_bringup,scout_msgs}` | Scout上游底盘包、CAN | 仓库对应整包；`scout_mapping.launch`与`scout_localization.launch`传入`/scout/odom`和`pub_tf=false` | `ugv_sdk scout_msgs scout_base scout_bringup` | `bringup_can2usb.bash`后检查`candump can0`、`/scout/odom`、`/scout_status` | 无CAN帧时停在底盘层；不允许用底盘TF替代FAST-LIO链 |
| `Livox-SDK2`、`livox_ros_driver2` | 上游固定版本 | 驱动整包及Mid-360连接配置；项目入口只include `msg_MID360.launch` | 按上游安装SDK，再编译`livox_ros_driver2` | 检查`/livox/lidar`、`/livox/imu`频率和时间戳 | 无原始点云时不排查mapper、NDT或导航 |
| `FAST_LIO` | 上游算法+Scout配置 | `config/mid360.yaml`、项目内`fastlio_mapping_scout.launch`与`fastlio_local_odom.launch`；关闭`pcd_save_en`，开启body点云 | `fast_lio`实际包名对应目标 | 检查`/Odometry`、`/cloud_registered`、`/cloud_registered_body`和`camera_init -> body` | 不接收预处理点云；姿态/时间异常先在前端解决 |
| `scout_tf_manager`、`scout_system_bringup` | 项目自有 | `config/extrinsics.yaml`、`config/scout_geometry.yaml`、`scripts/{tf_manager,geometry_tf_publisher}.py`及三个正式入口launch | `scout_tf_manager scout_system_bringup` | 用`tf_monitor`核对每条边只有一个发布者 | 重复`map->odom`或`odom->camera_init`时禁止继续导航 |
| `scout_pose_adapter`、`scout_cloud_adapter` | 项目自有 | 两整包；前者输出`/fastlio_odom`，后者按launch目标frame变换点云 | `scout_pose_adapter scout_cloud_adapter` | 检查消息frame和TF时间，不只看话题是否存在 | 转换失败不得通过新增静态TF掩盖 |
| `scout_pointcloud_mapper` | 项目自有C++ | `src/pointcloud_mapper_node.cpp`、`config/mapper.yaml`、`launch/pointcloud_mapper.launch`、CMake/package | `pointcloud_mapper_node` | 建图入口下检查`/scout/static_scan`及私有save/reset服务；退出后检查两份PCD | 候选约2秒未输出正常；长期为空再查frame、里程计时差和门限 |
| `scout_terrain_filter`、`patchworkpp` | 项目适配+上游算法 | `patchworkpp_scout.yaml`、`terrain_guard_scout.yaml`、两个launch、guard/accumulator源码 | `terrain_guard_node terrain_map_accumulator_node`及`patchworkpp` | 正式导航检查ground/nonground、obstacle/clearing和`/terrain/status` | 建图默认无这些话题；仅导航或诊断模式要求存在 |
| `scout_map_tools` | 项目自有离线工具 | `finalize_map.py`、四个C++工具、四份地图配置和CMake/package | `pcd_transform_node pcd_static_gate_node terrain_reclassify_node pcd_to_pgm_node` | 对同名地图运行finalize并检查第14.2节全部产物 | 原始PCD或轨迹来自不同会话时默认拒绝混用 |
| `scout_2p5d_navigation` | 项目自有库/插件 | terrain map头源、builder/server、costmap layer、`start_escape_recovery.cpp`、插件XML与配置 | `scout_terrain_map scout_2p5d_plugins terrain_map_builder_node terrain_map_server_node` | 检查插件可加载、坡度话题存在；逃逸先做静态安全门测试 | 后向覆盖不足时拒绝倒车是正确结果，不得关闭安全门掩盖 |
| `fast_lio_localization` | 现有NDT-OMP包 | `map_loader.cpp`、`fast_lio_localization.cpp`、`scout_relocalization.launch`参数 | `fast_lio_localization map_loader` | RViz给`/initialpose`，确认`/map_cloud`与唯一`map -> odom` | NDT未收敛不得启动运动测试 |
| `scout_navigation` | 项目自有配置/脚本 | `navigation_teb.launch`、四份planner/costmap配置、`nav_logging.launch`、`global_planning_test.launch`和scripts | Python/shell脚本由catkin安装；依赖上表插件库 | 依次做规划屏蔽测试、正式导航和日志分析 | 导航launch出现Livox/FAST-LIO/NDT/底盘节点即为入口回归 |
| `realsense2_camera` + `D435I.launch` | 上游相机驱动+项目入口 | `scout_system_bringup/launch/D435I.launch`；仅开color/depth/CameraInfo/TF，关IMU和PointCloud2 | 驱动按上游编译，bringup无C++目标 | 独立启动后检查彩色、对齐深度和CameraInfo；不应改变导航节点 | 当前外参RPY配置为0；若实机倒装必须实测后修改，不能复制轮趣外参 |

日志功能的执行入口为：

```bash
roslaunch scout_navigation nav_logging.launch tag:=factory_a_baseline
```

它在`~/livox_fastlio/logs/navigation/`创建带时间戳目录，保存launch/config/rosparam/节点话题快照、分卷LZ4 bag，并调用`analyze_nav_bag.py`输出CSV和摘要。默认只记录Terrain Guard障碍点和状态，不重复录制body、terrain、ground、nonground与近似全量clearing点云，以免拖慢Jetson。日志只观察系统，不启动或重启导航；地图、bag和日志都禁止提交Git。

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

V5.1新增编译目标：`scout_map_tools/{pcd_static_gate_node,terrain_reclassify_node}`以及`scout_2p5d_navigation`库中的`StartEscapeRecovery`。只改YAML/Launch不用单独编译，但本次包含C++变更，必须完整执行上述构建。

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

第三条必须包含`scout_navigation_patchworkpp`、`scout_navigation_terrain_guard`、两个地图服务器节点和`move_base`，不能列出Livox、FAST-LIO、NDT或Scout底盘。

## 14. 实车验证流程

### 14.1 建图

```bash
rosrun scout_bringup bringup_can2usb.bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a
```

```bash
rostopic hz /cloud_registered
rostopic hz /scout/static_scan
rostopic hz /fastlio_odom
```

正常`Ctrl+C`后确认`filtered_camera_init.pcd`和`traversed_path_map.pcd`均存在。默认不会生成在线分类PCD，这是V5.1预期行为。

### 14.2 生成资产

```bash
rosrun scout_map_tools finalize_map.py factory_a
MAP_DIR=/home/nvidia/livox_fastlio/maps/factory_a
test -s "$MAP_DIR/public_map.pcd"
test -s "$MAP_DIR/map_raw.yaml"
test -s "$MAP_DIR/terrain_2p5d.yaml"
test -s "$MAP_DIR/terrain_ground_map.pcd"
test -s "$MAP_DIR/terrain_obstacles_map.pcd"
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
rostopic hz /terrain/patchwork_ground
rostopic hz /terrain/patchwork_nonground
rostopic hz /terrain/obstacle_points
rostopic hz /terrain/clearing_points
rosparam get /move_base/recovery_behaviors
```

预期规划器分别为 `global_planner/GlobalPlanner`、`teb_local_planner/TebLocalPlannerROS`，两处膨胀均为 `0.10`。

## 15. 验收清单

- [ ] `scout_mapping.launch` 是唯一建图入口。
- [ ] mapper保存`filtered_camera_init.pcd`与`traversed_path_map.pcd`。
- [ ] 静态晋升满足12次、2秒和60%命中率；撤销满足8次自由射线与0.75秒。
- [ ] FAST-LIO 不读取任何过滤后点云。
- [ ] `map -> odom` 只有 NDT 一个发布者。
- [ ] Scout 底盘 `pub_tf=false`，速度反馈使用 `/scout/odom`。
- [ ] 定位入口持续运行，导航入口不重启 Livox、FAST-LIO、NDT 或底盘。
- [ ] GlobalPlanner + TEB 为唯一正式规划组合，没有 DWA。
- [ ] 全局、局部 costmap 和 TEB 软膨胀均为 `0.10 m`。
- [ ] Scout footprint、速度限制和刚体外参未被 WheelTech 参数覆盖。
- [ ] `sensor_height=0.48 m` 与实测雷达中心离地高度一致。
- [ ] `base_link_height_above_ground=0.28 m`，离线地面种子不误用0.20 m刚性偏移。
- [ ] 轮胎总高 `0.15 m`，正式相对障碍阈值为 `0.08 m`。
- [ ] 导航局部costmap订阅`/terrain/obstacle_points`和`/terrain/clearing_points`，不再订阅elevation前缀旧话题。
- [ ] `recovery_behavior_enabled=false`保持默认；启用前验证未知区、图外和后向覆盖不足均拒绝倒车。
- [ ] 实车移动测试前先完成Patchwork++平地输出、TF和后向覆盖检查。

## 16. Git 同步

组织仓库为`git@github.com:AADCL/ugv.git`。Scout车端克隆目录以`git remote -v`实机确认；只同步`Scout_mini/`和必要的仓库根README，不提交地图、PCD、bag、日志、密钥、`build/`或`devel/`。

本版本允许部署到 Scout 车端。先完成编译和静态话题检查；任何车辆移动测试仍需提前告知现场人员。
