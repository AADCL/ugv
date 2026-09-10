# Scout Mini 自主导航机器人使用文档 V5.1 · BATF-Nav

> 技术名称：`BATF-NavBayesianAccumulationTerrainFusionNavigation`；简称 **BATF-Nav**；英文展开 **Bayesian Accumulation Terrain Fusion Navigation**。见[统一技术说明](../../docs/BATF-Nav_V5.1.md)。本次统一命名不改变V5.1版本、接口或车型参数。

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

### 2.2 一键建图入口（与轮趣一致）

```bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a
```

这个命令启动完整建图链。完成采集后停车，在同一终端按一次`Ctrl+C`，等待mapper退出保存完成，再按第3节生成地图。与轮趣一致，地图采集和最终转换分两步执行。

需要排查在线地形分类时，使用同一个入口并显式打开诊断开关：

```bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a enable_online_terrain_diagnostics:=true
```

诊断开关日常保持关闭。默认建图链启动：

- Livox 与 FAST-LIO；
- Scout CAN 底盘；
- 可逆贝叶斯静态PCD；
- `/fastlio_odom`车体中心轨迹保存；
- TF和pose adapter。

Patchwork++在线分类默认关闭，最终PGM和2.5D在采集结束后通过`finalize_map.py`从同一贝叶斯静态PCD统一重建。启动约2秒后`/scout/static_scan`趋于稳定属于正常现象。

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

### 2.4 停止与保存

车辆完全停稳并释放遥控/teleop后，在建图终端按一次`Ctrl+C`，等待mapper保存完成、roslaunch退出。正常退出会保存`filtered_camera_init.pcd`和`traversed_path_map.pcd`；检查终端没有保存失败或超时强制终止提示，再执行第3节转换。不要使用`kill -9`，也不要在保存期间执行finalize。

执行第3节转换成功后，完整产物应为：

```text
/home/nvidia/livox_fastlio/maps/factory_a/
├── filtered_camera_init.pcd
├── raw_camera_init.pcd
├── traversed_path_map.pcd
├── public_map.pcd
├── terrain_ground_candidates_map.pcd
├── terrain_ground_map.pcd
├── terrain_obstacles_map.pcd
├── map_raw.pgm / map_raw.yaml
├── map.pgm / map.yaml
├── terrain_cost.pgm / terrain_cost.yaml
├── terrain_2p5d.yaml
├── terrain_2p5d_elevation.f32
├── terrain_2p5d_slope_deg.f32
├── terrain_2p5d_roughness.f32
├── terrain_2p5d_step_height.f32
├── terrain_2p5d_cost.u8
├── terrain_2p5d_confidence.u8
└── map_metadata.yaml
```

若地图目录中仍存在 `.finalization_incomplete`，表示该次转换未完整成功；即使目录中已有部分PCD、PGM或高程文件，这张地图也不可用于重定位、导航或交付。根据finalize终端的首个错误排查，重新执行`finalize_map.py`，并确认该标志已消失。

日常不需要手动调用保存服务；采集结束后必须执行一次finalize。旧`scout_mapping_session.py`仅保留兼容，不再作为推荐入口。

## 3. 生成地图资产

等待建图launch退出后执行，与轮趣相同：

```bash
rosrun scout_map_tools finalize_map.py factory_a
```

等待`[DONE] map finalized`后检查。若同名目录已有不同的归档raw，工具会拒绝混用；只有确定替换该地图时才追加`--replace-raw`。

```bash
MAP_DIR=/home/nvidia/livox_fastlio/maps/factory_a
ls -lh "$MAP_DIR"
```

正常目录包含：

```text
filtered_camera_init.pcd
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
terrain_2p5d_elevation.f32
terrain_2p5d_slope_deg.f32
terrain_2p5d_roughness.f32
terrain_2p5d_step_height.f32
terrain_2p5d_cost.u8
terrain_2p5d_confidence.u8
map_metadata.yaml
```

检查完整文件清单的同时，还必须确认目录中不存在
`.finalization_incomplete`；存在该标志时，不能通过复制、改名或手工删除标志的方式将地图强行交付。

`map_raw.yaml`是正式导航静态占据图；`map.yaml`用于定位入口显示兼容。2026-09-09起，新生成PGM的离线障碍膨胀由`0.15 m`减为`0.10 m`，旧地图不会自动变化。车体轨迹只在0.30 m半宽的真实扫掠走廊中补充自由证据，障碍始终覆盖自由证据。

不要单独移动或删除 `terrain_2p5d.yaml` 的`layers`字段所列`.f32/.u8`文件。不替换已归档raw、仅核对同一输入时可省略`--replace-raw`：

```bash
rosrun scout_map_tools finalize_map.py factory_a
```

若出现`filtered_camera_init.pcd.capacity_limited`，说明Jetson内存保护已开始丢弃新体素；工具会拒绝把截断PCD发布为正式地图。不要删除标志强行交付，应提高`mapper.yaml`中的`dynamic_filter/max_voxels`或`map/max_voxels`后重新建图。`--allow-capacity-limited`只用于开发者抢救数据，不能用于正式地图。

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

## 6. 建图 15 cm 与导航 10 cm 膨胀的含义

- PGM `obstacle_inflation_m=0.10 m`：在保存地图时从原始障碍边界向外写入占据格；
- global costmap `inflation_radius=0.10 m`：静态障碍向外扩展的运行距离；
- local costmap `inflation_radius=0.10 m`：实时障碍向外扩展的运行距离；
- TEB `inflation_dist=0.20 m`：轨迹优化的软代价范围，必须大于0.15 m净空；
- footprint：Scout 实际车体 polygon，保持原值；
- TEB `min_obstacle_dist=0.15 m`：车体边界到障碍的期望净空，保持原值。

建图0.10 m与全局运行时0.10 m不是覆盖关系：先在PGM写入占据格，再计算运行时代价，不能把两者之和当成车体安全净空。local costmap的0.10 m作用于实时障碍。footprint表示车体轮廓，padding和TEB净空是另行使用的量，不是全部简单相加。

### 角度晃动首轮调参与旧地图迁移（2026-09-09）

当前配置轮廓为长0.670 m、宽0.590 m：base_link向前0.370 m、向后0.300 m、左右各0.295 m。costmap每侧padding为0.030 m，外包络为0.730×0.650 m；这不是新测量结果，仍需核对实车附件。保留Scout轮廓，不采用轮趣0.500×0.400 m的小车轮廓。

导航入口自动加载TEB新值：`max_vel_theta=0.40 rad/s`（约22.9度/秒），`acc_lim_theta=0.30 rad/s²`。原值分别为1.00和2.50。线速度0.35 m/s、线加速度0.50 m/s²及禁止主动倒车不变。TEB是轨迹优化器，不是可直接调整P增益的PID；本次未改底盘固件。调参时车端ROS未运行，不能宣称已定位或消除晃动。TEB加速度是优化约束，并非独立硬件限幅器；手柄指令也不受这两个TEB参数限制。

下次启动原导航launch即可加载新值，无需附加节点。低速空旷场地测试直行、转弯、终点对齐；现场人员持急停。若仍晃动，使用现有导航日志记录流程：比较`/cmd_vel`、`/scout/odom`、`/Odometry`和`/tf`；指令反复换向查规划，指令平稳而车身晃动查底盘，车身不动而map朝向跳变查重定位和TF。不要继续盲降PID或改外参。

旧地图要采用10 cm离线膨胀，必须停车并停止建图、定位、导航，先备份整个地图目录，再运行：

```bash
source ~/livox_fastlio/devel/setup.bash
# factory_a仅为示例，替换为实际地图名；先确认没有同名备份目录。
cp -a ~/livox_fastlio/maps/factory_a ~/livox_fastlio/maps/factory_a_before_inflation_20260909
rosrun scout_map_tools finalize_map.py factory_a --replace-raw
```

等待转换全部成功、`.finalization_incomplete`不存在，再重启定位和导航。不能对已有PGM直接腐蚀，避免误删真实障碍。当前部署只更新默认配置，不自动覆盖任何现有地图。

## 7. 雷达与轮胎高度

当前Patchwork++`sensor_height=0.48 m`，对应雷达中心到地面的实测垂直高度。刚性雷达Z偏移是0.20 m，因此`base_link_height_above_ground=0.28 m`。轮胎总高0.15 m，正式地图与在线障碍下限取0.08 m。Livox位于`base_link`前方0.25 m，而车头边界为0.37 m，因此当前帧分割最小水平半径取0.12 m：它只在正前方约对应保险杠平面，并不是矩形车体裁剪，侧后方可能包含自身回波。必须在空车静止时检查局部costmap；若有自反射，应增加实测footprint crop，不能继续盲调单一半径。0.10～0.20 m近距回波也必须通过静态障碍测试。

修改文件：

```text
scout_terrain_filter/config/patchworkpp_scout.yaml
scout_terrain_filter/config/terrain_guard_scout.yaml
scout_system_bringup/config/scout_geometry.yaml
```

不要把`base_link -> body`的`z=0.20 m`直接当成雷达离地高度或base_link离地高度。V5.1 Terrain Guard、PGM、2.5D和导航障碍阈值统一为0.08 m；保存高程和当前帧Guard的最大可通行坡度统一为22度。无局部地面参考时，Guard以雷达坐标`z=-0.42 m`作为保守低障碍兜底，并禁止未分类点清空costmap。

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
| finalize失败 | `filtered_camera_init.pcd`、轨迹和新工具是否完整；若有`.capacity_limited`先提高容量并重新建图 |
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
4. 建图模式等待mapper保存及launch退出，再执行finalize并等待`[DONE] map finalized`；
5. 最后关闭 CAN 和整车电源。

## 旁路融合里程计（2026-09-09）

`scout_mapping.launch`与`scout_localization.launch`默认增加`scout_odom_fusion`，不改旧话题、不发布TF、不向NDT或导航回灌。两种入口不能同时启动；`navigation_teb.launch`不重复启动融合。需要关闭时给建图/定位入口添加`enable_shadow_fusion:=false`。

原有系统已经启动时，可以单独运行一次：

```bash
source ~/livox_fastlio/devel/setup.bash
roslaunch scout_odom_fusion fusion.launch
rostopic echo -n 1 /scout/fusion/status
rostopic hz /scout/fused_odom
```

新输出`/scout/fused_odom`为`nav_msgs/Odometry`，`header.frame_id=odom`、`child_frame_id=base_link`，约20 Hz。原有`/scout/odom`、`/Odometry`、`/fastlio_odom`全部保留。坐标遵循前X、左Y、上Z，绕Z逆时针为正；不交换XY或翻转Y。

融合保留FAST-LIO的odom世界原点，不以融合节点启动位置重新归零。轮速只使用base_link下的前向速度，不使用其累计XYZ/航向，因此底盘驱动重启时位置归零不会把融合位置拉回000。底盘速度正负号保留，倒车为负；转弯时增大轮速观测方差，不融合未经验证的轮速角速度。

数据按原始header时间戳进入EKF：仅接收启动后新到且不旧于0.30秒的有效样本；相同时间戳不重复融合，乱序/未来超过0.05秒的样本拒绝。异步数据由EKF时间队列与1秒历史回滚处理，不要求把20 Hz位姿强行配成50 Hz轮速。原始`/Odometry`在camera_init/body下，不能直接替换输入；现有TF适配器已完成odom/base_link刚体转换。它的速度字段是填零的，不参与融合。

出现错误坐标、明显LIO跳变或运行后任一数据源/EKF超过0.50秒未更新时，`/scout/fused_odom`停止发布，状态为ERROR。恢复需先确认源数据正常，再停止并重启整个`fusion.launch`（两个融合节点一起），不能只重启guard；不会自动重置或修改原定位节点。此保护仅控制融合输出，不会给底盘发停车指令，也不会停止原导航。

`SHADOW_OK_NOT_NAVIGATION`仅表示数据链路有效，不代表已验证定位精度。当前为松耦合旁路试验，不能修复FAST-LIO内部或NDT错误更新。日志入口自动增加融合输出、输入与状态；对比时先截取共同时间段，再计算同一base_link参考点的相对运动，不直接拿轮速启动归零的位置与odom绝对位置作差。

## 真车无NDT一键对比测试

先停车，退出原建图、定位、导航和独立fusion.launch。保留CAN接口正常配置，现场持急停。执行：

```bash
source ~/livox_fastlio/devel/setup.bash
rosrun scout_system_bringup scout_fusion_test.py
```

脚本检查ROS节点冲突；发现旧底盘、FAST-LIO、NDT、导航或融合节点时拒绝启动，不杀旧节点。新测试只启动Livox、FAST-LIO、Scout底盘、TF管理、pose adapter、融合与对比，无NDT、map_server、Navigation，不保存PCD、不自动开车。不要绕过脚本直接启动同名测试launch。

等待终端`[COMPARE] READY`，再保持静止约10秒，然后人工遥控直行、停车、转弯或返回。终端每秒输出wheel/lio/fused三行：`dx/dy`为共同起点车体坐标下相对位移，`yaw`为相对航向，`net`为起终点水平直线距离，`path`为采样累计水平路程。`dXY/wheel`和`dYaw/wheel`是相对轮速的差异，不是真实定位误差；累计路程包含静止噪声，航向显示限制在正负180度。

停车后Ctrl+C，脚本退出自己启动的测试链路并保存：

```text
~/livox_fastlio/logs/fusion_tests/<日期_时间_唯一后缀>/
  samples.csv       同时间戳的三路相对轨迹和差异，约10 Hz
  summary.json      最后结果、最大/RMS差异、拒绝样本数和失效原因
  comparison.png    XY轨迹、相对航向、相对轮速位置差三图
```

每次自动创建新目录，不覆盖旧结果；CSV持续刷新，正常退出生成汇总和图片。突然断电可能缺少最后数据、汇总和图片。该入口默认不录bag，需要原始记录时另用现有导航日志记录入口（无move_base会提示但可继续录制）；本CSV并非完整原始ROS消息。图片最多显示最近20000个对齐样本，CSV保留完整会话。

比较按同一时刻插值并分别计算`T_start^-1 * T_current`，完整三维旋转参与转换，不直接相减不同原点。出现断流、明显归零跳变或frame变化时显示INVALID并停止累计，不自动拼接新原点；此时只保存故障前有效结果，不会发送车辆停止指令，应由现场人员停车并重启测试。

若只想观察已经运行的三路里程计而不启动任何硬件，可单独执行`rosrun scout_odom_fusion compare_odometry.py`，但必须自行确认没有重复对比节点；这不能保证当前系统中NDT已关闭。`scout_fusion_test.py --check-only`只执行启动前检查，不启动硬件。
