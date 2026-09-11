# Scout Mini 自主导航机器人开发实施文档 V5.1 · BATF-Nav

> 技术名称：`BATF-NavBayesianAccumulationTerrainFusionNavigation`；简称 **BATF-Nav**；英文展开 **Bayesian Accumulation Terrain Fusion Navigation**。见[统一技术说明](../../docs/BATF-Nav_V5.1.md)。本次统一命名不改变V5.1版本、接口或车型参数。

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
- `scout_mapping.launch`是正式一键入口，与轮趣相同；采集结束停车并Ctrl+C，等待贝叶斯静态PCD和车体轨迹保存完成，再执行`finalize_map.py NAME`生成分类PCD、PGM和高程坡度。
- `scout_localization.launch` 与 `navigation_teb.launch` 职责分离。导航入口不启动 Livox、FAST-LIO、NDT 或 Scout 底盘。
- PGM 仍决定墙体、固定障碍和未知区；坡度层只给已知自由格增加代价，不能把未知格改成自由格。
- 正式导航使用 GlobalPlanner + TEB。DWA 和旧 TerrainGlobalPlanner 不进入当前代码树。
- Patchwork++在正式导航阶段对当前帧直接分类；保存高程只提供全局坡度软代价，不再参与局部障碍判定。
- 建图期Patchwork++累积仅作为`enable_online_terrain_diagnostics:=true`的回归诊断支路，默认关闭以降低Jetson负载。
- `StartEscapeRecovery`已安装但默认关闭；完成实车静态覆盖验证后，才可在全局起点确定为致命占据、局部倒车走廊安全且后向点云覆盖新鲜时启用低速倒车。
- 本版本目标为平地、连续坡道以及平地与坡道组合路段；不承诺楼梯语义或跨层地图。

## 2. 目录与包职责

将仓库 `Scout_mini/src/` 中的项目包复制到 `/home/nvidia/livox_fastlio/src/`。Git仓库布局就是`Scout_mini/src/<各ROS包>`；仓库外Windows备份才使用`D:\设备文档\Scout_mini\src\src\<各ROS包>`这一层级。第三方包按固定版本另行获取，不要把 WheelTech 的底盘、外参和导航参数覆盖到 Scout。

| 包 | 需要复制或配置的内容 | 职责 |
|---|---|---|
| `scout_ros`、`ugv_sdk` | Scout 原包 | CAN 驱动、`/scout/odom` 和 `/cmd_vel` |
| `livox_ros_driver2`、`Livox-SDK2` | 上游依赖 | Mid-360 点云与 IMU |
| `FAST_LIO` | 上游包加 Scout 配置 | 本地激光惯性里程计 |
| `scout_system_bringup` | 整包 | 一键建图会话、唯一底层建图链、独立定位入口和几何真值 |
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

## 6. 统一建图入口（与轮趣一致）

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

交付给操作人员的正式入口是`scout_system_bringup/launch/scout_mapping.launch`：

```bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a
```

操作顺序与轮趣一致：用户停车、释放遥控/teleop并按一次`Ctrl+C`，等待mapper正常退出保存，再执行第8节的finalize。launch已包含点云处理、TF、里程计和底盘，不需要另行启动过滤器。旧`scripts/scout_mapping_session.py`保留为可选兼容监督工具，仍由CMake安装，但不是正式入口；其20 Hz零速保持和自动finalize仅在显式运行旧脚本时生效。正常launch不会自动执行地图转换，也不能把转换节点放进并发shutdown流程读取尚未保存完成的PCD。

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

`publish_dynamic_points=false`保持默认，避免Jetson调试带宽。`self_filter=false`保持默认，直到实测车体包围盒确认。崩溃恢复自动保存周期为120秒；正式结束由mapper正常退出时保存。`/scout/static_map_cloud`只有存在订阅者时才重建整图，headless建图不再每2秒遍历和序列化全部精细体素。

需要回归旧在线分类时，显式运行：

```bash
roslaunch scout_system_bringup scout_mapping.launch \
  map_name:=factory_a enable_online_terrain_diagnostics:=true
```

这时才会生成`terrain_ground_camera_init.pcd`与`terrain_obstacles_camera_init.pcd`，但它们不是默认交付输入。

## 8. 地图最终生成

每次建图采集结束、确认mapper保存和launch退出后，都执行本节，与轮趣一致：

```bash
rosrun scout_map_tools finalize_map.py factory_a
```

脚本顺序如下：

1. 在地图目录原子创建`.finalization_incomplete`，让正在运行或随后启动的定位/导航安全退出；
2. 归档或核对`raw_camera_init.pcd`和`traversed_path_map.pcd`；
3. 使用`scout_geometry.yaml`生成`public_map.pcd`；
4. `terrain_reclassify_node`从最终贝叶斯PCD提取保守PMF种子；
5. 以二维支持、平面内点率、RMSE、坡度和连接连续性执行鲁棒局部平面生长；
6. 仅在原点云附近有XY观测时拟合稠密地面，并按相对地面0.08～1.50 m重建障碍；
7. PGM把0.30 m半宽的真实车体轨迹作为自由证据，障碍证据优先；
8. 生成并校验`map_raw.*`、`map.*`、`terrain_cost.*`、分类PCD、`terrain_2p5d.yaml`六层文件和`map_metadata.yaml`；
9. 只有全部校验通过才原子删除`.finalization_incomplete`；任一中间命令、文件头、尺寸或YAML引用失败都保留该标志。

若不希望替换已归档raw，可省略`--replace-raw`，脚本只接受内容完全相同的输入。mapper一旦触发体素容量硬限制，会保存恢复PCD和`.capacity_limited`标志并让保存服务失败；finalize默认拒绝该截断地图。`--allow-capacity-limited`仅用于开发抢救，不能交付。

新增/修改文件为`terrain_reclassify.cpp/.yaml`、`pcd_static_gate.cpp`、`pcd_to_pgm.cpp`、`finalize_map.py`和三个Scout地图配置。2026-09-09按用户要求，三份PGM配置的`obstacle_inflation_m`从0.15减至`0.10`；move_base现有0.10 m运行时膨胀不改，两者依次生效而非覆盖，也不能简单相加作为车体净空。`free_evidence_radius_m=0.30`来自Scout半宽0.295 m，不复制WheelTech的0.20 m。离线重分类、PGM与2.5D builder均设置`max_extent_m=100`和`max_grid_cells=2000000`，遇到远端XY离群点时明确失败，避免Jetson创建无界稠密网格。

## 9. 高程坡度与当前帧局部障碍

`scout_2p5d_navigation/config/terrain_builder.yaml` 当前基线：

```yaml
resolution: 0.10
fit_radius_m: 0.30
preferred_slope_deg: 5.0
max_slope_deg: 22.0
max_slope_cost: 80
obstacle_min_relative_height_m: 0.08
obstacle_max_relative_height_m: 1.50
fuse_static_obstacles: false
```

每格保存地面高度，局部平面拟合坡度。5 度以内不增加坡度成本，5 到 22 度逐渐增加软代价；持续超过 22 度才成为致命坡度。该阈值与当前帧Terrain Guard一致，避免全局允许22～25度而局部封死。小孔洞只有邻居充足时才填充，未知区域不能被坡度层改成自由区。

`fuse_static_obstacles=false`表示静态致命障碍只由PGM StaticLayer负责，2.5D层只叠加坡度软代价，避免同一障碍在粗栅格中二次扩大。

导航阶段不再启动`terrain_relative_obstacle_node`。`navigation_teb.launch`将实时点云转换到重力对齐`terrain_sensor`，再启动Patchwork++和Terrain Guard：

- `/terrain/obstacle_points`写入local costmap marking；
- `/terrain/clearing_points`只执行raytrace clearing；
- `obstacle_min_relative_height=0.08 m`来自Scout 0.15 m轮径；
- Patchwork++和Guard最小水平半径为`0.12 m`，只在正前方对应前置Livox到车头边界；侧后方自反射需用实测footprint crop处理；
- 无地面参考的非地面点以`z=-0.42 m`保守兜底，未分类点不得进入clearing；
- 邻域不足4格或平面矩阵秩不足3的ground cell按未知/不安全处理，不允许fail-open；
- `observation_persistence=0`避免历史局部点云拖影。

局部障碍由当前扫描自行估计地面，不查询保存高程，因此不会随`map -> odom`重定位修正整体漂移。

## 10. 定位与导航入口必须分离

`scout_system_bringup/launch/scout_localization.launch` 包含常驻`scout_localization_map_bundle_guard`，并启动 Livox、FAST-LIO、TF、NDT、地图显示和 Scout 底盘持续运行。ROS launch中的进程近乎并发启动；守卫发现收尾未完成、容量截断或必要文件缺失时立即非零退出，节点配置为`required=true`，因此整套定位入口会一起停止，不能继续使用旧的混合产物。

`scout_navigation/launch/navigation_teb.launch` 只启动：

```text
scout_navigation_map_bundle_guard
scout_terrain_cloud_adapter
scout_navigation_patchworkpp
scout_navigation_terrain_guard
scout_navigation_map_server
scout_terrain_map_server
move_base
```

导航入口不得出现第二个Livox、`laserMapping`、NDT localizer或底盘驱动。定位、导航和规划测试三个guard使用不同ROS节点名，避免同时运行时互相踢掉并触发`required`级联停机。这里的Patchwork++/terrain_guard只处理当前帧局部碰撞，不发布TF，不会抢占重定位节点。

## 11. 导航参数

`scout_navigation/config/global_costmap_slope.yaml`：静态层 + TerrainCostmapLayer + InflationLayer。

`scout_navigation/config/local_costmap_slope.yaml`：当前帧marking/clearing双源 + InflationLayer。

`scout_navigation/config/teb_local_planner.yaml`：保留 Scout 原footprint和线速度；2026-09-09降低转向速度及加速度，作为角度晃动首轮调参，尚待实车验收。

本次统一修改：

```yaml
# global_costmap_slope.yaml
inflation_radius: 0.10

# local_costmap_slope.yaml
inflation_radius: 0.10

# teb_local_planner.yaml
TebLocalPlannerROS:
  max_vel_theta: 0.40
  acc_lim_theta: 0.30
  inflation_dist: 0.20
```

`costmap_common.yaml` 的 polygon footprint 和 `footprint_padding: 0.03` 未改。TEB `min_obstacle_dist: 0.15` 也未改，它是轨迹净空，不是 costmap 障碍膨胀。`inflation_dist=0.20`使软代价范围大于净空，修正原0.10小于0.15的问题。

本轮逐文件操作（路径均相对`~/livox_fastlio/src`）：

| 文件 | 修改内容 | 生效条件 |
|---|---|---|
| `scout_navigation/config/teb_local_planner.yaml` | 在现有`TebLocalPlannerROS`下修改上述三项，不新增第二个同名块；原角速度1.00、角加速度2.50、软距离0.10 | 重启导航launch |
| `scout_navigation/config/costmap_common.yaml` | 仅修正base_link非几何中心的注释；轮廓、padding不变 | 无行为变化 |
| `scout_map_tools/config/scout_raw.yaml` | `obstacle_inflation_m: 0.10` | 重新生成PGM |
| `scout_map_tools/config/scout_nav.yaml` | `obstacle_inflation_m: 0.10` | 重新生成PGM |
| `scout_map_tools/config/scout_terrain_cost.yaml` | `obstacle_inflation_m: 0.10` | 重新生成PGM |
| `scout_navigation/scripts/nav_log_session.sh` | TOPICS数组加入`/Odometry`，保留现有控制、底盘、TF记录 | 下次启动日志记录 |

本轮只改YAML、注释和日志脚本，不需要重新编译；新工作空间仍按逐包索引执行`catkin_make -j1`。既有导航launch已加载该TEB文件，无需额外命令或新节点。轮趣参考值为角速度0.40、角加速度0.30、离线膨胀0.00；Scout保留更大车体、3 cm padding和15 cm净空，离线膨胀只降到10 cm，未复制轮趣小车尺寸。

配置车体前0.370、后0.300、左右各0.295 m，即0.670×0.590 m，padding后的costmap外包络0.730×0.650 m。它们是配置值，不代表已复测当前附件。`scout_base/src/scout_messenger.cpp`将速度送往底盘SDK，本次不改固件PID。TEB角加速度是优化约束而非独立硬件限幅器，不限制手动遥控。

```bash
source ~/livox_fastlio/devel/setup.bash
# 只展开参数，不启动节点或让车运动。
roslaunch --dump-params scout_navigation navigation_teb.launch map_name:=factory_a
# 由现场人员正常启动定位和导航后核对：
rosparam get /move_base/TebLocalPlannerROS/max_vel_theta
rosparam get /move_base/TebLocalPlannerROS/acc_lim_theta
rosparam get /move_base/TebLocalPlannerROS/inflation_dist
```

预期分别为0.40、0.30、0.20。停车后备份地图、停止建图/定位/导航，再按使用文档运行`finalize_map.py <地图名> --replace-raw`迁移旧PGM；不要直接腐蚀现有PGM，不自动改动在用地图。回滚配置时将上述值恢复为1.00、2.50、0.10，三个离线值恢复0.15；旧地图需恢复完整目录备份，改回YAML不会恢复PGM。

验收必须分辨物理摆动和定位跳变：用本文日志流程记录`/cmd_vel`、`/scout/odom`、`/Odometry`、`/tf`。指令反复换向查TEB/路径，指令稳定而底盘摆动查底盘响应，车不动而map方向变查NDT和TF唯一性。空旷场地低速测试直行、转弯、终点对齐，现场持急停。部署时ROS未运行，不能以静态配置检查代替运动验收。

`move_base_slope_teb.yaml`安装并配置了`StartEscapeRecovery`，但`recovery_behavior_enabled=false`保持Scout现有导航行为不变。完成现场静态覆盖验收后才能手动启用。启用时逃逸速度0.05 m/s、目标0.30 m；后向覆盖区按雷达前置0.25 m换算为`terrain_sensor`坐标X=-1.05～-0.55 m、半宽0.36 m。覆盖还必须来自`terrain_sensor`帧且消息时间戳与回调墙钟都在0.25秒内，并满足至少20点、X跨度0.20 m且左右各至少5点；costmap在互斥锁内检查足迹边界和内部。全局起点不是明确致命碰撞（未知区和图外均拒绝）、局部走廊不安全或1.5秒内位移不足0.02 m时，插件拒绝/停止倒车。TEB自身`max_vel_x_backwards=0`保持原值。

### 11.1 功能包逐包复现索引

下表是从空工作空间复现本项目时的最小闭环。上游包只做版本集成；项目包按所列文件实现，不要把WheelTech同名配置覆盖进来。

| 功能包 | 依赖/性质 | 必须复制或修改的文件 | 编译目标 | 启动与验证 | 故障边界 |
|---|---|---|---|---|---|
| `ugv_sdk`、`scout_ros/{scout_base,scout_bringup,scout_msgs}` | Scout上游底盘包、CAN | 仓库对应整包；`scout_mapping.launch`与`scout_localization.launch`传入`/scout/odom`和`pub_tf=false` | `ugv_sdk scout_msgs scout_base scout_bringup` | `bringup_can2usb.bash`后检查`candump can0`、`/scout/odom`、`/scout_status` | 无CAN帧时停在底盘层；不允许用底盘TF替代FAST-LIO链 |
| `Livox-SDK2`、`livox_ros_driver2` | 上游固定版本 | 驱动整包及Mid-360连接配置；项目入口只include `msg_MID360.launch` | 按上游安装SDK，再编译`livox_ros_driver2` | 检查`/livox/lidar`、`/livox/imu`频率和时间戳 | 无原始点云时不排查mapper、NDT或导航 |
| `FAST_LIO` | 上游算法+Scout配置 | `config/mid360.yaml`、项目内`fastlio_mapping_scout.launch`与`fastlio_local_odom.launch`；关闭`pcd_save_en`，开启body点云 | `fast_lio`实际包名对应目标 | 检查`/Odometry`、`/cloud_registered`、`/cloud_registered_body`和`camera_init -> body` | 不接收预处理点云；姿态/时间异常先在前端解决 |
| `scout_tf_manager`、`scout_system_bringup` | 项目自有 | `config/extrinsics.yaml`、`config/scout_geometry.yaml`、`scripts/{tf_manager,geometry_tf_publisher}.py`、`scripts/{scout_mapping_session,map_bundle_guard}.py`及三个正式入口launch；CMake安装两脚本，package声明其运行依赖 | `scout_tf_manager scout_system_bringup` | 用`tf_monitor`核对每条边只有一个发布者；用无效地图验证guard让launch退出 | 重复TF或地图事务/容量标志存在时禁止继续导航 |
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

第二、三条应分别包含唯一命名的定位/导航地图guard。第三条还必须包含`scout_navigation_patchworkpp`、`scout_navigation_terrain_guard`、两个地图服务器节点和`move_base`，不能列出Livox、FAST-LIO、NDT或Scout底盘。

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

车辆停稳并释放遥控/teleop后按一次`Ctrl+C`，等待mapper保存PCD、轨迹和launch完全退出。随后执行`rosrun scout_map_tools finalize_map.py factory_a`，看到`[DONE] map finalized`后才检查全部资产。默认不会生成在线累积分类PCD。

### 14.2 检查资产（手动命令仅作恢复）

```bash
MAP_DIR=/home/nvidia/livox_fastlio/maps/factory_a
test -s "$MAP_DIR/public_map.pcd"
test -s "$MAP_DIR/map_raw.yaml"
test -s "$MAP_DIR/terrain_2p5d.yaml"
test -s "$MAP_DIR/terrain_ground_map.pcd"
test -s "$MAP_DIR/terrain_obstacles_map.pcd"
test ! -e "$MAP_DIR/.finalization_incomplete"
test ! -e "$MAP_DIR/filtered_camera_init.pcd.capacity_limited"
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

- [ ] `scout_mapping.launch` 是正式一键建图入口，与轮趣保持一致。
- [ ] 操作员停车Ctrl+C后等待保存和launch退出，再单独执行`finalize_map.py NAME`。
- [ ] mapper保存`filtered_camera_init.pcd`与`traversed_path_map.pcd`。
- [ ] mapper体素容量超限时保存服务和finalize均明确拒绝正式交付。
- [ ] finalize执行期间/失败后保留`.finalization_incomplete`，只有全套资产校验通过才删除。
- [ ] 静态晋升满足12次、2秒和60%命中率；撤销满足8次自由射线与0.75秒。
- [ ] FAST-LIO 不读取任何过滤后点云。
- [ ] `map -> odom` 只有 NDT 一个发布者。
- [ ] Scout 底盘 `pub_tf=false`，速度反馈使用 `/scout/odom`。
- [ ] 定位入口持续运行，导航入口不重启 Livox、FAST-LIO、NDT 或底盘。
- [ ] 定位、导航和规划测试入口各有唯一命名的常驻地图guard，残缺或容量截断地图会使对应launch整体退出。
- [ ] GlobalPlanner + TEB 为唯一正式规划组合，没有 DWA。
- [ ] 全局、局部 costmap 和 TEB 软膨胀均为 `0.10 m`。
- [ ] Scout footprint、速度限制和刚体外参未被 WheelTech 参数覆盖。
- [ ] `sensor_height=0.48 m` 与实测雷达中心离地高度一致。
- [ ] `base_link_height_above_ground=0.28 m`，离线地面种子不误用0.20 m刚性偏移。
- [ ] 轮胎总高 `0.15 m`，正式相对障碍阈值为 `0.08 m`。
- [ ] 新PGM离线障碍膨胀为`0.10 m`；TEB角速度0.40、角加速度0.30、软距离0.20；实车晃动改善与避障净空单独验收。
- [ ] 保存和当前帧最大可通行坡度均为`22 deg`，近距起点为`0.12 m`。
- [ ] 导航局部costmap订阅`/terrain/obstacle_points`和`/terrain/clearing_points`，不再订阅elevation前缀旧话题。
- [ ] `recovery_behavior_enabled=false`保持默认；启用前验证未知区、图外和后向覆盖不足均拒绝倒车。
- [ ] 实车移动测试前先完成Patchwork++平地输出、TF和后向覆盖检查。

## 16. Git 同步

组织仓库为`git@github.com:AADCL/ugv.git`。Scout车端克隆目录以`git remote -v`实机确认；只同步`Scout_mini/`和必要的仓库根README，不提交地图、PCD、bag、日志、密钥、`build/`或`devel/`。

本版本允许部署到 Scout 车端。先完成编译和静态话题检查；任何车辆移动测试仍需提前告知现场人员。

## 17. FAST-LIO与轮速旁路融合开发

目标：保留全部旧输出，新增`/scout/fused_odom`供记录比较。以成熟的`robot_localization` EKF为融合核心，不自制位置加权平均。输出仍处于FAST-LIO的odom世界原点，以base_link为车体参考点；不发布TF，不替换NDT、TEB、地图轨迹采集的输入。

### 17.1 依赖、文件和编译顺序

以下路径相对`~/livox_fastlio/src`。把仓库中完整功能包放入工作空间，不只复制一个脚本。

| 文件 | 开发内容与责任 |
|---|---|
| `scout_odom_fusion/package.xml` | 声明rospy、nav_msgs、diagnostic_msgs和robot_localization依赖 |
| `scout_odom_fusion/CMakeLists.txt` | 安装Python节点、config、launch，注册单元测试 |
| `scout_odom_fusion/scripts/fusion_guard.py` | 输入帧/时间/数值校验，协方差赋值，EKF初始化顺序、输出有效性、断流锁止和状态发布 |
| `scout_odom_fusion/config/ekf.yaml` | 20 Hz三维EKF；LIO选择XYZ/RPY，轮速只选择vx；禁用TF和relative/differential归零 |
| `scout_odom_fusion/config/guard.yaml` | 明确输入话题、帧、时效、跳变门限及观测方差 |
| `scout_odom_fusion/launch/fusion.launch` | 启动`scout_shadow_ekf`与`scout_fusion_guard`，只输出新话题 |
| `scout_odom_fusion/test/test_guard.py` | 时间窗、四元数、重复样本、原点保留、轮速000重置、负速度、异常帧、LIO跳变和断流输出测试 |
| `scout_system_bringup/launch/scout_mapping.launch` | 添加默认true的enable_shadow_fusion参数和fusion.launch include |
| `scout_system_bringup/launch/scout_localization.launch` | 同上；导航入口不再重复include |
| `scout_system_bringup/package.xml` | 添加scout_odom_fusion运行依赖 |
| `scout_navigation/scripts/nav_log_session.sh` | TOPICS数组添加融合输出、状态及两路预处理输入 |

```bash
sudo apt-get install ros-noetic-robot-localization
cd ~/livox_fastlio
source /opt/ros/noetic/setup.bash
catkin_make -j1 --pkg scout_odom_fusion
source devel/setup.bash
python3 src/scout_odom_fusion/test/test_guard.py
python3 src/scout_odom_fusion/test/test_isolated_ekf.py
roslaunch --dump-params scout_odom_fusion fusion.launch
```

新环境须先按本文前面的逐包索引完成全部上游包构建。只更新本包Python/YAML无需重编C++；首次新增包需运行catkin以生成可执行入口。车端采用ROS Noetic robot_localization 2.7.7。

`test_isolated_ekf.py`自动使用独立11431端口ROS master，验证非零起点、90度车头朝向时vx转为世界Y运动、LIO延迟80 ms、轮速pose中途归零、输出单调时间戳、无TF及断流停止。端口占用时拒绝运行；不向实车master发布合成数据，测试结束自动关闭测试进程。此测试不等于实车精度验收。

### 17.2 坐标与观测契约

FAST-LIO原始camera_init/body位姿经现有TF适配器转换为odom/base_link，再送入guard。guard严格校验这两个frame，错误输入拒绝，不通过“改frame_id标签”冒充坐标转换。轮速Odometry的twist按ROS语义位于child_frame_id，本版必须为base_link且前左上。轮速世界原点和pose完全不参与融合；不需要估计其000原点与FAST-LIO原点的偏移，也不重复融合由轮速积分来的位置。

EKF必须先收到有效LIO位姿并输出接近该位姿的初始解，guard才放行轮速，防止轮速先到导致滤波器初始化在000。输出不人为归零，不发布新odom到base_link TF。EKF状态选择按x/y/z/roll/pitch/yaw/vx/vy/vz/vroll/vpitch/vyaw/ax/ay/az顺序：

```yaml
odom0_config: [true, true, true, true, true, true,
               false, false, false, false, false, false, false, false, false]
odom1_config: [false, false, false, false, false, false,
               true, false, false, false, false, false, false, false, false]
publish_tf: false
two_d_mode: false
smooth_lagged_data: true
history_length: 1.0
```

保留三维姿态以兼容现有坡道，不把车体速度当成odom世界X速度。EKF根据车体姿态将前向速度映射到世界坐标。不是紧耦合，不回写FAST-LIO内部状态；不引入NDT反馈或同源IMU的重复观测。

### 17.3 时间截取、噪声及故障边界（baseline原配置）

- header时间戳保留，不换成接收时间；重复/乱序、超过0.30秒或未来超过0.05秒的样本拒绝，EKF用1秒历史处理两路异步延迟。
- LIO位姿方差初值`[0.01,0.01,0.04,0.0025,0.0025,0.0025]`，单位依次为m²和rad²；为调试假设，非标定精度，不把原适配器零协方差解释为完美观测。
- 轮速vx方差0.0025 (m/s)²；转向乘数`min(25,1+(abs(omega)/0.30)^2)`，只降权vx，不将omega送入EKF。这不是完整打滑检测。
- 数值有效性门限：速度绝对值不超过2 m/s、角速度不超过3 rad/s；仅用于拒绝数据，不改底盘运动限速。
- LIO相邻位姿允许平移`0.30+2*dt`米、旋转`0.20+3*dt`弧度，超过门限或间隔超过0.50秒锁止输出。小于门限的源重置无法保证被识别，重启LIO时必须同时重启融合。
- 正常开始发布后，任一路或EKF断流超过0.50秒锁止；同时检查ROS时间和墙钟，/clock停止也不能无限假预测。时钟回跳同样锁止。修复源后重启整个融合launch，不能仅恢复guard而保留旧EKF状态。
- 该保护只关闭新输出，绝不停止原导航或发送cmd_vel；原导航/NDT输入保持不变。

### 17.4 验证和回滚

先静态验证`/scout/fused_odom`的frame/stamp/非零协方差、原始话题频率，以及tf树未增加任何边。当前系统已经运行时只启动一次独立fusion.launch，不重启底盘或定位。检查`/scout/fusion/status`，正常应为SHADOW_OK_NOT_NAVIGATION；它是数据健康状态，不是定位精度保证。

动态验收在现场人员控制下执行直行、转弯、倒车，按共同时间窗计算两路及融合的相对刚体运动；不能把轮速累计pose直接当同原点真值。回放必须使用独立ROS master，禁止将历史/cmd_vel或/initialpose重放到实车master。回滚时停止两个融合节点，并在下一次建图/定位启动时设置`enable_shadow_fusion:=false`；旧链路和原始数据无需修改。

## 18. 无NDT实车对比脚本

| 相对src的文件 | 实现 |
|---|---|
| `scout_system_bringup/scripts/scout_fusion_test.py` | ROS图预检查；旧节点存在则退出码2拒绝，不杀节点；启动独立进程组的测试launch；Ctrl+C等待退出 |
| `scout_system_bringup/launch/scout_fusion_test.launch` | 独立硬件/FAST-LIO/TF/轮速/融合/比较链，无地图、NDT、Navigation；PCD保存保持false |
| `scout_odom_fusion/scripts/compare_odometry.py` | 三路时间缓冲、位置线性插值＋四元数SLERP、共同时间起点SE(3)对齐、差异统计、CSV/JSON/PNG |
| `scout_odom_fusion/test/test_compare.py` | 非零原点与90度朝向对齐、插值、正负180度跨界、禁止外推和跨长断流 |
| 两个包的`CMakeLists.txt`/`package.xml` | 安装脚本，声明rosgraph/rosnode和numpy/scipy/matplotlib依赖，注册比较测试 |

```bash
sudo apt-get install python3-numpy python3-scipy python3-matplotlib
cd ~/livox_fastlio
source devel/setup.bash
catkin_make -j1 --pkg scout_odom_fusion scout_system_bringup
source devel/setup.bash
python3 src/scout_odom_fusion/test/test_compare.py
roslaunch --nodes scout_system_bringup scout_fusion_test.launch
rosrun scout_system_bringup scout_fusion_test.py --check-only
# 实车停车并退出旧链路后：
rosrun scout_system_bringup scout_fusion_test.py
```

节点列表不得出现scout_global_localizer、move_base或map_server。启动前检查发现冲突退出是预期行为，不应为了绕过检查改节点名。检查不能消除另一个操作者同时启动节点的竞态，现场需保持唯一操作者。

比较输入固定参考点base_link，lio/fused要求odom世界帧；wheel允许odom/scout_odom且帧不可中途变化。保留header时间，不晚于0.50秒、不超前0.05秒；重复和乱序不纳入缓冲。每路最多300条，用共同最新时刻减20 ms作为采样点，插值跨度不能超过0.20秒，不外推；结果约10 Hz，终端1 Hz。原点取同一时刻三路各自位姿，完整相对变换后比较水平XY与包裹到正负180度的yaw。

运行后单路断流超过0.50秒、相邻平移超过`0.30+2*dt`米或转角超过`0.20+3*dt`弧度会锁止比较并保留失效原因。明显轮速000重置必须分新会话，不能用于同一条参考轨迹；小于门限的重置无法保证检测。没有独立真值时禁止把RMS参考差标为绝对定位精度。

Ubuntu20.04 SciPy1.3使用from_dcm/as_dcm，脚本同时兼容新版本from_matrix/as_matrix，不要求升级Jetson系统SciPy。图像保存用无界面Agg后端，不依赖桌面环境。脚本只订阅Odometry、写诊断文件，不发布TF/里程计/cmd_vel。

### 18.1 轮速优先试验版本（2026-09-10）

沿用robot_localization EKF，新增可选`wheel_priority`，保留`baseline`。只改变无NDT试验入口的默认配置；不改FAST-LIO内部、NDT、TF发布权、导航输入及参数。

按下表逐文件同步仓库`Scout_mini/src/`中的完整文件到设备`~/livox_fastlio/src/`同名位置，不拼接历史代码片段。

| 文件 | 实现与修改内容 |
|---|---|
| `scout_odom_fusion/config/guard_wheel_priority.yaml` | 新增下方完整覆盖配置，基线`guard.yaml`保持原值 |
| `scout_odom_fusion/launch/fusion.launch` | `profile`默认baseline；只接受baseline/wheel_priority；先加载guard.yaml，再按profile加载覆盖文件；写入guard私有参数profile |
| `scout_odom_fusion/scripts/fusion_guard.py` | 读取initial_lio_pose_variances，默认等于原pose方差；EKF尚未确认初始化时持续使用原点先验，确认后才切换运行方差；诊断增加profile |
| `scout_system_bringup/scripts/scout_fusion_test.py` | 增加`--profile {baseline,wheel_priority}`，默认wheel_priority，传递给launch；原冲突检查保留 |
| `scout_system_bringup/launch/scout_fusion_test.launch` | 新增fusion_profile默认wheel_priority，并传入fusion.launch的profile参数 |
| `scout_odom_fusion/scripts/compare_odometry.py` | 启动保存guard/EKF实际参数到configuration.json，退出summary.json也包含快照 |
| `scout_odom_fusion/test/test_guard.py` | 增加启动消息未被EKF接收时保持原点先验、收到输出后切换位置方差的回归测试 |
| `scout_odom_fusion/test/test_isolated_ekf.py` | 增加profile、yaw-deg、turn-rate、speed、lio-scale、lio-delay、duration、port参数；校验任意朝向/转弯/倒车、LIO少报距离、非零原点、轮速归零、断流及无TF |

覆盖文件完整有效配置：

```yaml
lio_pose_variances: [1000000.0, 1000000.0, 1000000.0, 0.0025, 0.0025, 0.0025]
wheel_speed_variance: 0.0004
initial_lio_pose_variances: [0.01, 0.01, 0.04, 0.0025, 0.0025, 0.0025]
```

位置方差单位m²、角度rad²、速度(m/s)²。1000000 m²是刻意近乎关闭运行位置纠偏的试验权重，不能解释为实测精度；本轮重点测试轮速积分加LIO姿态，正常场景也会失去大部分LIO平移纠偏能力。轮速标准差调参值从0.05降为0.02 m/s。转弯降权仍为`min(25,1+(omega/0.30)^2)`，不融合轮速角速度。运行XYZ同等弱化，不能只弱化世界X，否则走廊方向改变就失效。此版不是方向退化检测，也未加入车体横向零速伪观测。

首个ROS发布消息可能早于EKF订阅者连接，因此不能以“已经发布一帧”作为初始化成功；以guard收到接近LIO位姿的EKF输出为准。原点先验只定义初始坐标的不确定性，不证明初始绝对位置正确。初始化后不使用轮速pose，驱动累计位置归零不会拉回融合原点。

速度观测`twist.linear.x`属于`child_frame_id=base_link`的前左上坐标。状态中的车体速度通过当前融合姿态旋转进入odom：`v_odom=R_odom_base*v_base`。水平前向贡献是`[v*cos(yaw),v*sin(yaw),0]`，上坡使用三维旋转。禁止手动把前向速度直接写入世界X或重复旋转输入。

同步后编译与隔离验证（不会启动硬件）：

```bash
cd ~/livox_fastlio
source /opt/ros/noetic/setup.bash
catkin_make -j1 --pkg scout_odom_fusion scout_system_bringup
source devel/setup.bash
python3 src/scout_odom_fusion/test/test_guard.py
python3 src/scout_odom_fusion/test/test_compare.py
roslaunch --dump-params scout_odom_fusion fusion.launch profile:=wheel_priority
python3 src/scout_odom_fusion/test/test_isolated_ekf.py --profile baseline
python3 src/scout_odom_fusion/test/test_isolated_ekf.py --profile wheel_priority --yaw-deg 90 --lio-scale 0.2
python3 src/scout_odom_fusion/test/test_isolated_ekf.py --profile wheel_priority --yaw-deg 37 --turn-rate 0.1 --lio-scale 0.2
python3 src/scout_odom_fusion/test/test_isolated_ekf.py --profile wheel_priority --yaw-deg -125 --speed -0.2 --lio-scale 0.2
python3 src/scout_odom_fusion/test/test_isolated_ekf.py --profile wheel_priority --yaw-deg 90 --lio-scale 0.2 --duration 60
```

私有master默认11431，占用则拒绝，可用`--port 11432`指定另一个空闲端口。测试从不向11311发送数据。合成轨迹通过只证明实现和给定条件下的行为；真实长管廊、打滑及航向误差仍需实测。

真车测试使用`rosrun scout_system_bringup scout_fusion_test.py --profile wheel_priority`，静止等待READY后再运动，停车后Ctrl+C保存。原版用`--profile baseline`。比较configuration.json中的实际参数，避免两轮配置混淆。保持原始话题、新输出和诊断；无新增TF。LIO完全失效仍锁止，不能宣称轮速已能独立接管；长期错误位置观测仍可能慢慢拉偏，姿态偏差也会直接旋转前向积分方向。

2026-09-10在120端侧ROS Noetic私有master验证记录（合成数据，非真车精度）：

| 配置/场景 | 条件 | 结果 |
|---|---|---|
| baseline正常LIO | yaw=90°，0.2 m/s，10秒 | 终点误差约0.004 m |
| baseline位置退化对照 | 相同速度，LIO仅报告20%距离 | 约2 m真值得到0.428 m位移，终点误差约1.571 m |
| wheel_priority长时间直行 | yaw=90°，0.2 m/s，60秒，LIO报告20%距离 | 约12 m真值得到11.933 m位移，终点误差约0.063 m |
| wheel_priority斜向转弯 | 初始yaw=37°，转速0.1 rad/s，10秒，LIO报告20%距离 | 终点误差约0.045 m |
| wheel_priority倒车 | yaw=-125°，速度-0.2 m/s，10秒，LIO报告20%距离 | 终点误差约0.045 m |

各场景均通过非零世界原点、80 ms LIO延迟、轮速累计位置中途归零、输出时间戳单调、不发布TF及断流停止检查。guard共10项、比较数学共4项单元测试通过。测试包含启动时短暂未接收轮速的误差，数值不代表标定精度；没有模拟轮胎打滑和LIO姿态失真，也未替代真实走廊验收。

## 独立 R³LIVE：从源码到编译、启动和验证

本节新增可选工作空间，不改 `~/livox_fastlio/src` 中的 FAST-LIO、融合、地图或导航包。项目适配源码完整保存在仓库 `Scout_mini/optional/r3live/`；上游代码由固定提交加完整补丁复现，不用文档片段代替源码。采用官方 R³LIVE，不把论文补充材料当成已经公开的 R³LIVE++ 实现。

### 1. 版本和依赖

| 组件 | 固定来源/提交 | 作用 |
|---|---|---|
| R³LIVE | `https://github.com/hku-mars/r3live.git`，`6143a38537f28cb36eb24e9bbe2e39c8f7967157` | 激光/惯性/视觉局部估计 |
| vision_opencv | `https://github.com/ros-perception/vision_opencv.git`，`cfabf72fb02970a661b5e68fbee503c5d9f94729` | 在独立overlay中重编译cv_bridge和image_geometry |
| Livox | 复用已安装的livox_ros_driver2及Mid-360配置 | CustomMsg、IMU输入 |
| 相机 | IntelRealSense/realsense-ros，`f400d682beee6c216052a419f419e95b797255ad`（2.3.2），在r3live_ws独立重编 | D435i彩色图像和出厂内参 |
| 图像插件 | ros-perception/image_transport_plugins，`cae592ce0816ea03bc38f0b6d27b6eae907feb10` | 在同一overlay编译compressed、compressedDepth、theora，避免混用OpenCV |
| 平台 | Ubuntu20.04、Noetic、ARM64、OpenCV4.5.4、Eigen3、PCL、Boost.Python、Python3 numpy/yaml | 保留端侧现有依赖，不替换系统库 |

端侧原 `/opt/ros/noetic` 的cv_bridge链接OpenCV4.2，与本机OpenCV4.5不一致。必须使用新overlay的cv_bridge；直接source旧工作空间后运行新二进制可能重新混用版本。可选CGAL网格重建未启用，核心定位不需要因此安装CGAL。

### 2. 文件逐项落地

以下路径均相对 `Scout_mini/optional/r3live/`；表内简写的`config/`、`launch/`、`scripts/`均位于其`scout_r3live_bringup/`下。需要修改时编辑对应完整文件，重新执行安装脚本并编译；配置和launch修改无需重编译C++。

| 文件 | 写入内容与责任 |
|---|---|
| `install_scout_r3live.sh` | 创建独立workspace，固定上游提交，检查并应用补丁，复制自有包，串行编译；遇到不同提交的脏目录拒绝覆盖 |
| `scout-r3live.patch` | 所有上游修改的完整统一diff，保留上游许可；不能只应用其中一部分 |
| `start_scout_r3live.sh` | source独立overlay，要求存在新相机库；不前置旧相机包路径，执行独立launch |
| `scout-realsense-opencv.patch` | 在相机CMakeLists显式查找OpenCV，将其include和library加入目标；随overlay的cv_bridge统一版本，不修改旧realsense_ws |
| `check_opencv_runtime.py` | 用ldd检查相机、cv_bridge、三种图像插件及mapping；缺库或不是单一OpenCV4.5时失败 |
| `calibration/prepare_indoor_trial.py` | 核对用户选定六场景loose矩阵、备份默认配置及哈希、新建独立试用外参和start.sh；拒绝重复目录，试用授权不表示精度验收 |
| `scout_r3live_bringup/package.xml`、`CMakeLists.txt` | 自有ROS包依赖、Python可执行脚本、launch/config安装 |
| `config/rig.yaml` | `T_base_imu`、`T_base_camera_link`，xyz单位m、RPY单位度，明确标记为近似安装尺寸 |
| `config/estimator.yaml` | Mid-360前端、LIO、VIO公共参数；实际相机K/D和外参由启动器生成 |
| `launch/scout_r3live.launch` | 用户入口、check_only和传感器开关、日志目录参数 |
| `launch/sensors.launch` | Mid-360驱动与D435i彩色640×480@15Hz；关闭深度、红外、相机IMU、相机点云 |
| `launch/estimator.launch` | 加载完整运行配置，启动前端和估计器，隔离输出话题 |
| `scripts/session.py` | 冲突检查、等待真实传感器和相机TF、生成运行快照、等待两路位姿、维护并退出自己创建的进程 |
| `test_frontend.py` | 私有master下验证360°、非法点过滤、时间和畸形消息拒绝 |
| `test_camera_config.py` | 验证真实图像/内参/光学TF并生成回放配置，写到`/tmp` |
| `test_observe.py` | 回放位姿有限值、四元数、帧名和时间单调检查；不输出定位精度结论 |
| `observe_indoor.py` | 有限10～120秒读取LIO、相机位姿、raw图像、IMU、跟踪图；保存频率、时间差、间断、相对位移和转角；原地观测必须由人确认停车，返回成功也不是精度合格 |

补丁修改的上游文件相对 `~/r3live_ws/src/r3live/r3live/`：

| 上游文件 | 必须写入的适配内容 |
|---|---|
| `CMakeLists.txt`、`package.xml` | C++14、ARM不使用x86 SSE编译选项、driver2/cv_bridge依赖、CGAL可选 |
| `src/optical_flow/lkpyramid.hpp`、`.cpp` | 去掉强制SSE开关；x86特有实现条件编译，ARM使用已有通用路径 |
| `src/tools/tools_logger.hpp` | CPUID仅在x86调用；ARM不执行x86诊断指令 |
| `src/loam/LiDAR_front_end.cpp` | 新增type4 Mid-360 CustomMsg处理，球形盲区、line/tag/有限性检查，ns转ms，不沿用Avia前向裁剪 |
| `src/loam/include/common_lib.h`、`src/r3live.cpp` | 激光到IMU平移由单一定义共享，替代每个编译单元独有的Avia常量 |
| `src/r3live.hpp` | 在线程前加载外参平移、收敛消息队列长度 |
| `src/r3live_lio.cpp` | `r3live_world→r3live_imu`，里程计/路径/TF使用测量时间，路径头与轨迹帧一致 |
| `src/r3live_vio.cpp` | 默认无窗口无键盘等待、独立输出帧、相机位姿使用源时间、RGB地图独立话题 |

### 3. 安装与编译

```bash
cd ~/github_upload/ugv
bash Scout_mini/optional/r3live/install_scout_r3live.sh
```

若HTTPS网络不可达，可先用本机已配置的GitHub SSH方式克隆到脚本指定目录，再执行脚本；不要在仓库中写入密钥或密码。核心编译命令为：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_fastlio/devel/setup.bash
cd ~/r3live_ws
catkin_make -j1 -DCMAKE_BUILD_TYPE=Release -DR3LIVE_BUILD_MESHING=OFF
source devel/setup.bash
ldd devel/lib/r3live/r3live_mapping | grep -E 'cv_bridge|opencv|not found'
python3 ~/github_upload/ugv/Scout_mini/optional/r3live/check_opencv_runtime.py ~/r3live_ws
```

`r3live_mapping`、`r3live_LiDAR_front_end`、cv_bridge、image_geometry、realsense2_camera、三种图像插件及自有启动包应编译成功。ldd应指向`~/r3live_ws/devel/lib/libcv_bridge.so`，所有检查对象不得出现`not found`或同时加载OpenCV4.2与4.5。源码补丁完整重应用会触发较长编译；Jetson使用`-j1`。

2026-09-11真机发现旧相机nodelet在compressed图像发布时SIGSEGV：core回溯为OpenCV4.5的cvtColor调用4.2的_OutputArray::create，且旧RealSense库直接链接两版OpenCV。修复范围为独立overlay内的相机与插件，不修改原导航工作空间。仅检查mapping的ldd不足以验收整条图像链路，必须再检查相机进程`/proc/<PID>/maps`及真实图像订阅。

室内试用部署：先完成上述安装，再运行`python3 ~/r3live_ws/calibration_tools/prepare_indoor_trial.py <新试用名> --confirm-trial`。脚本由`calibration/install_calibration.sh`复制到设备，无需单独编译Python。它固定读取`prior_robust_final_20260911/loose.yaml`，校验已审阅矩阵，输出`calibration/trials/<新试用名>/backup/`、`backup_hashes.yaml`、`trial_calibration.yaml`、`expected_runtime_extrinsic.yaml`和`start.sh`；不适用于任意其他候选。运行生成的start.sh，核对会话runtime.yaml的R/t与expected_runtime_extrinsic.yaml一致，再验证视觉输出。回退时停车退出试用入口，原默认rig/estimator未被覆盖，直接使用原入口即可；不要把整棵试用目录的备份覆盖到不同版本工程。

### 4. 坐标与标定代码约定

`rig.yaml`的transform均为“子坐标点变换到父坐标”。初值沿用Scout现有安装配置：base→IMU `[0.25,0,0.20]m`、pitch45°；base→camera_link `[0.27,0,0.10]m`。D435i自身camera_link到彩色光学坐标的变换从驱动读取，不手写轴交换矩阵。

启动器实际计算：`T_imu_optical = inverse(T_base_imu) * T_base_camera_link * T_camera_link_optical`；将旋转行优先展平写入`r3live_vio/camera_ext_R`，平移写入`camera_ext_t`。相机K、D、宽高来自同一彩色流的CameraInfo。上游使用`T_world_camera=T_world_imu*T_imu_camera`，不要把外参取反。

Mid-360内部雷达→IMU平移为`[-0.011,-0.02329,0.04412]m`，沿用当前FAST-LIO配置，旋转单位阵。它与底盘安装45°、相机外参是三件不同的事。源图像/IMU时间相差超过1秒拒绝启动，只是时钟粗检，不等于毫秒级时间标定。在线内外参估计默认关闭，不能用它掩盖未标定的初值。

### 5. 分层验证与限制

真实启动按使用文档的一条命令完成。已有FAST-LIO运行时，check_only必须报告冲突且不能关闭原节点。隔离回放使用另外的ROS master，只播放白名单`/livox/lidar`、`/livox/imu`，不要回放底盘指令或旧TF。

前端合成测试需要在私有master加载`Lidar_front_end/lidar_type=4`、`point_step=1`、`N_SCANS=4`、`blind=0.5`，启动前端并把`/laser_cloud_flat`重映射为`/r3live/laser_cloud_flat`，然后运行`python3 test_frontend.py`。它检查120个后向点保留、近点/NaN/非法tag和line剔除、11.9ms末点时间以及非法point_num拒绝。

相机测试通过`sensors.launch start_lidar:=false`和`test_camera_config.py`读取真实工厂内参，生成`/tmp/scout_r3live_test_runtime.yaml`。关闭相机测试后，设置私有master的`use_sim_time=true`，启动`estimator.launch runtime_config:=/tmp/scout_r3live_test_runtime.yaml`及`test_observe.py`，再播放bag。后台rosbag play应重定向stdin为`/dev/null`，避免被终端读输入挂起。测试结束只退出这个master的进程。

必须区分：编译成功、输出正常、视觉参与更新、定位精度达标是四个不同验收层级。无同步图像的旧走廊bag只能测试LIO；短时彩色图像测试也不能代替长走廊、暗光、扬尘和真实外参标定验收。当前不发布`map→odom`，不替换导航输入，不提供轮速接管，不将R³LIVE输出直接送入现有地图finalize流程。

2026-09-10端侧验证：ARM64串行编译、OpenCV单版本运行时链接、Mid-360转换合成测试、真实D435i内参/图像/内部TF、已有FAST-LIO冲突拒绝、空闲私有master预检查均通过。旧走廊bag输出422帧有效LIO里程计；最终独立入口联合回放收到178帧里程计、264帧相机位姿和264帧跟踪图像，路径帧和时间检查通过。日志中视觉跟踪点约105～130，几何和光度更新返回成功。这些是短时功能测试，不是精度、运动、掉线降级或长期内存验收。R³LIVE累积地图可能持续增长，长时运行需另测资源占用。

完整测试依据和复现步骤见[`../optional/r3live/TEST_REPORT_20260910.md`](../optional/r3live/TEST_REPORT_20260910.md)。几何/光度返回状态是上游函数返回值，不等于外参或轨迹已正确。现场保留的测试bag和日志不上传GitHub。

## Scout端离线相机—雷达标定开发步骤

源码目录`Scout_mini/optional/r3live/calibration/`，独立workspace `~/lidar_camera_calib_ws`。固定hku-mars/livox_camera_calib提交`061fdaa647fc806e59d73a8505a05a10dfcfdaa1`，依赖Ceres1.14（端侧安装版本）、PCL、Eigen、ROS Noetic及与R³LIVE overlay一致的C++ OpenCV/cv_bridge。Python图像直接解码rgb8/bgr8/mono8，避免在同一Python进程混入cv_bridge的另一版本OpenCV。

完整新增程序和补丁都在仓库中，不需要手工拼接代码片段。按如下顺序落地：

| 文件（相对optional/r3live） | 修改与职责 | 构建/检查 |
|---|---|---|
| calibration/install_calibration.sh | 固定版本、检查应用补丁、安装Ceres、复制工具和R³LIVE配置读取模块 | 在设备bash执行；拒绝覆盖不同提交的脏上游 |
| calibration/scout-calibration.patch | CMake仅构建多场景工具；增加rosbag/Ceres include；上游显示调用改为无GUI保存；小于30匹配、边缘不足、不可用解拒绝；修复多场景粗搜索错误复用最后场景计数、优化遗漏k3、K近邻不足访问 | `catkin_make -j1 -DCMAKE_BUILD_TYPE=Release`，目标lidar_camera_multi_calib |
| calibration/scout_calibrate.sh | 完整环境与CLI，source显式传--extend避免--help传入catkin环境脚本 | bash -n；--help |
| calibration/calibrate.py | sensors、capture、export、prepare、solve、project、accept完整实现 | 实机静态capture/export、独立端口solve、投影与接受保护 |
| scout_r3live_bringup/scripts/calibration_io.py | 矩阵SO(3)检查、内参/尺寸/帧一致性、接受文件读取、雷达到相机矩阵转IMU到相机使用方向 | test_geometry.py |
| scout_r3live_bringup/scripts/session.py | 按calibration_file覆盖近似外参；校验实际相机和雷达帧，保存标定快照，不改默认值 | 原工作空间catkin_make -j1；旧FAST-LIO存在时仍拒绝R³LIVE启动 |
| scout_r3live_bringup/launch/scout_r3live.launch、CMakeLists.txt | calibration_file参数，安装辅助模块 | XML/包检查 |
| calibration/test_geometry.py | 几何方向/杆臂、坏矩阵、参数变更、未接受文件、PCD和名字约束 | python3执行，需脚本目录在PYTHONPATH |
| calibration/test_synthetic_scene.py | 已知单位外参的多平面合成测试数据 | 输出到独立测试目录，不能作为实车标定 |

部署及编译：

```bash
cd ~/github_upload/ugv
bash Scout_mini/optional/r3live/calibration/install_calibration.sh
~/r3live_ws/scout_calibrate.sh --help
```

采集导出约束：driver2 CustomMsg点数一致、源时间非零且单调；保持源雷达坐标和米单位；CameraInfo与原图配对；保留相机内部tf_static；旋转/可用轮速运动门限；最大250万有效输入点；过滤0.5m盲区/20m外点/NaN/tag/line；在初始相机视野外扩半幅范围裁剪并按2cm体素降采样。失败保留.incomplete，场景同名拒绝覆盖。此处理仅用于标定PCD，不修改FAST-LIO输入或正式建图。

prepare建立训练副本和参数快照，统一K/D/帧，拒绝相同点云冒充多个场景。solve使用独立ROS master默认11441，端口占用拒绝，退出只清理其子进程；.solving存在时禁止验收。输出是`p_camera_optical=T_camera_lidar*p_lidar`，R³LIVE转换为`T_imu_camera=T_imu_lidar*inverse(T_camera_lidar)`。默认不发布新TF、不自动启用候选矩阵。project仅固定矩阵投影，近似深度/法向变化边缘距离不是实际外参误差。accept需要独立场景及显式人工确认，输出空间标定文件，不估计时间偏移。

逐条操作、输出目录、恢复方式与全部参数边界见[标定工具完整说明](../optional/r3live/calibration/README.md)。需要改变C++时在固定上游工作树编辑并更新完整patch；只改Python/YAML/launch时重新部署对应完整文件即可。编译成功和合成通过均不代表真实场景精度通过。

### 实测先验约束研究的逐文件开发与部署

源码仍在optional/r3live/calibration。以下是2026-09-11新增完整文件与改动，均已入库；运行系统参数不变。

| 文件 | 写入内容与接口 | 验证 |
|---|---|---|
| scout-extraction.patch | 在已应用旧scout-calibration.patch的C++多场景程序加入SCOUT_CALIB_EXTRACT_ONLY分支，输出逐场景平面交线XYZ和图像边缘uv CSV后退出，不优化 | 编译目标lidar_camera_multi_calib，真实六场景提取 |
| install_calibration.sh | 按顺序幂等应用旧补丁和新增提取补丁；部署完整Python脚本 | 重复安装的reverse --check不重复应用 |
| calibrate.py | solve内部extract_only参数，在私有master只提取，成功移除.solving，不生成外参结果 | 普通solve保持原行为；refine提取退出和日志 |
| scout_calibrate.sh | 首参数refine转入prior_refine.py，其余旧命令不变 | refine --help、原--help、bash -n |
| prior_refine.py | 完整实测基线/中心原点假设、固定样本Chamfer、数据Huber与独立测量先验、三档敏感性、逐一移除训练、固定样本投影、新旧结果对照 | 真实数据完整研究；验证数据不进入拟合 |
| test_prior_refine.py | 斜线切向零垂距、左乘旋转与平移/杆臂方向、含错误边缘的非零真值恢复、无可靠匹配拒绝 | 端侧python3执行 |

顺序：修改上述完整源码→执行`bash Scout_mini/optional/r3live/calibration/install_calibration.sh`→在~/lidar_camera_calib_ws以`catkin_make -j1 -DCMAKE_BUILD_TYPE=Release`构建C++提取器（安装脚本已执行）→加载该workspace环境和bringup/scripts的PYTHONPATH→执行test_prior_refine.py→运行使用手册的refine命令。完整算法代码就在prior_refine.py，无需再到文档外拼接未提供的实现。只改Python时部署对应完整文件即可，不必重复编译C++。

数学约定：T_camera_lidar的旋转用相机系左增量，平移直接在相机系加偏移。对原实测中心误作IMU原点的情况，仅在研究中减去R_camera_lidar*t_imu_lidar，视外壳中心近似雷达原点，不宣称实际原点相同。数据残差为固定几何交线投影到可靠图像边缘的最近欧氏距离/2px，超过20px保留常数惩罚、不删除样本；Huber阈值2，训练场景等权、总等效权重200；测量先验单独二次惩罚。各轴±3倍假设尺度，最终旋转/平移模长达到约3倍尺度、500次求值内未收敛或拟合前后训练匹配不足30均拒绝。旋转和平移三档尺度为0.5°/5mm、1°/10mm、2°/20mm，明确只是敏感性设置。

避免验证偏差：每场景固定基线视野内、2cm采样、上限2000点的真实平面交线；候选丢失投影按50px计入；输出中位/P90/3px和5px比例/可见数。对照旧run时验证hash、相机、frame、内部杆臂和训练/验证隔离。研究保存.study_incomplete直到全部输出完成；没有accepted文件接口，不自动启用新外参。数学恢复测试验证程序行为，不证明实车厘米级精度。上游VPnP方向残差未在旧路径上修改，新研究独立采用固定样本边缘距离；求解容差ftol/xtol/gtol均为1e-6。
