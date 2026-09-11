# Scout Mini 自主导航机器人使用文档 V5.1 · BATF-Nav

> 技术名称：`BATF-NavBayesianAccumulationTerrainFusionNavigation`；简称 **BATF-Nav**；英文展开 **Bayesian Accumulation Terrain Fusion Navigation**。见[统一技术说明](../../docs/BATF-Nav_V5.1.md)。本次统一命名不改变V5.1版本、接口或车型参数。

> 本文用于日常建图、地图生成、重定位和导航。V5.1 使用实测雷达中心离地 `0.48 m` 和轮胎总高 `0.15 m`。

## 0. 120 端目录更新（2026-09-11）

三个独立空间已迁入 `~/livox_fastlio/optional/{r3live_ws,realsense_ws,lidar_camera_calib_ws}`，旧 home 路径保留为兼容软链接。CCS、导航参数及正式地图保留。历史 bag 和测试派生数据已删除。当前试用外参在 `~/r3live_ws/config/accepted_20260911/`，新试用记录在 `~/r3live_ws/logs/indoor_tests/`。入口仍为 `~/r3live_ws/start_scout_r3live_test.sh`。完整布局见[工作空间指南](../tools/workspace/WORKSPACE_GUIDE.md)。

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

2026-09-10起，上述无NDT测试入口默认选择`wheel_priority`。原版对照用
`rosrun scout_system_bringup scout_fusion_test.py --profile baseline`；显式选择新版用
`rosrun scout_system_bringup scout_fusion_test.py --profile wheel_priority`。
每次先结束上一轮测试，再启动另一配置。正式建图/定位入口及独立`fusion.launch`仍默认`baseline`，导航参数和输入不变。

新版强化车体前向轮速，弱化FAST-LIO的XYZ位置观测，继续使用其三维姿态。
本轮位置权重设得极弱，主要用于验证“轮速积分＋LIO姿态”，正常环境下也会牺牲LIO平移纠偏能力，不自动识别是否进入走廊。
前向指当前`base_link`的X轴，绝不是上电时`odom`的X轴：水平情况下，世界速度为
`vx_odom = v_forward*cos(yaw)`、`vy_odom = v_forward*sin(yaw)`；坡道按完整三维姿态转换。
所以可以先转向再进入任意方向的走廊，倒车保留负速度。轮速累计位置和航向不参与融合。

测试请包含不同朝向直行、转弯、倒车、静止及回到起点，并记录实测起终点距离。
新配置不是故障时的轮速独立接管：FAST-LIO断流/明显跳变仍停止融合输出；姿态漂移、打滑及长期弱位置约束仍可能造成误差。
对轮速更接近只证明权重变化，不能代替实测精度验收。

脚本检查ROS节点冲突；发现旧底盘、FAST-LIO、NDT、导航或融合节点时拒绝启动，不杀旧节点。新测试只启动Livox、FAST-LIO、Scout底盘、TF管理、pose adapter、融合与对比，无NDT、map_server、Navigation，不保存PCD、不自动开车。不要绕过脚本直接启动同名测试launch。

等待终端`[COMPARE] READY`，再保持静止约10秒，然后人工遥控直行、停车、转弯或返回。终端每秒输出wheel/lio/fused三行：`dx/dy`为共同起点车体坐标下相对位移，`yaw`为相对航向，`net`为起终点水平直线距离，`path`为采样累计水平路程。`dXY/wheel`和`dYaw/wheel`是相对轮速的差异，不是真实定位误差；累计路程包含静止噪声，航向显示限制在正负180度。

停车后Ctrl+C，脚本退出自己启动的测试链路并保存：

```text
~/livox_fastlio/logs/fusion_tests/<日期_时间_唯一后缀>/
  samples.csv       同时间戳的三路相对轨迹和差异，约10 Hz
  summary.json      最后结果、最大/RMS差异、拒绝样本数和失效原因
  comparison.png    XY轨迹、相对航向、相对轮速位置差三图
  configuration.json 启动时实际guard/EKF参数，含profile；summary也包含该快照
```

每次自动创建新目录，不覆盖旧结果；CSV持续刷新，正常退出生成汇总和图片。突然断电可能缺少最后数据、汇总和图片。该入口默认不录bag，需要原始记录时另用现有导航日志记录入口（无move_base会提示但可继续录制）；本CSV并非完整原始ROS消息。图片最多显示最近20000个对齐样本，CSV保留完整会话。

比较按同一时刻插值并分别计算`T_start^-1 * T_current`，完整三维旋转参与转换，不直接相减不同原点。出现断流、明显归零跳变或frame变化时显示INVALID并停止累计，不自动拼接新原点；此时只保存故障前有效结果，不会发送车辆停止指令，应由现场人员停车并重启测试。

若只想观察已经运行的三路里程计而不启动任何硬件，可单独执行`rosrun scout_odom_fusion compare_odometry.py`，但必须自行确认没有重复对比节点；这不能保证当前系统中NDT已关闭。`scout_fusion_test.py --check-only`只执行启动前检查，不启动硬件。

## 独立 R³LIVE 试验入口（2026-09-10）

本入口是官方 **R³LIVE** 的 Scout/ARM64/Mid-360 适配版，不是 R³LIVE++。FAST-LIO 的原入口继续保留，按场景选一套运行。R³LIVE 使用自己的激光惯性与视觉估计，不是在当前 FAST-LIO 输出上再接一个相机节点。当前接入的是设备实连的 **D435i 彩色流**，不是奥比深度相机；本分支不使用深度图。

先停车，退出原来拥有 FAST-LIO、雷达、轮速融合、NDT或导航的启动终端。新入口检测到这些节点会拒绝启动，不会替你关闭它们。然后执行：

```bash
~/r3live_ws/start_scout_r3live.sh
```

这一条命令启动 Mid-360、D435i 彩色流、R³LIVE及工程接口适配（`project_interface:=true`默认开启）；连续检查传感器时间，读取相机内参、内部光学TF，生成本次运行配置，再检查估计输出及`odom→base_link`。在水平地面保持静止，等待终端提示 `READY`，再由现场人员决定是否低速测试。READY后默认600秒结束（test_duration允许30～1800秒）。该入口不启动底盘、遥控、NDT、轮速融合、地图转换或导航，也不会发布速度指令。

只检查包、几何配置和节点冲突、不启动传感器：

```bash
~/r3live_ws/start_scout_r3live.sh check_only:=true
```

正常运行后，可在已加载独立环境的终端观察：

```bash
source ~/r3live_ws/devel/setup.bash
rostopic hz /r3live/odometry
rostopic hz /r3live/camera_odometry
rostopic hz /r3live/track_image
rosrun tf tf_echo r3live_world r3live_imu
rostopic hz /fastlio_odom
rostopic hz /cloud_registered_base
rosrun tf tf_echo odom base_link
```

RViz 的 Fixed Frame 设为 `odom`，点云选 `/cloud_registered`，车体位姿选 `/fastlio_odom`，原始轨迹仍选 `/r3live/path`，TF会完成显示转换。`/r3live/odometry`仍对应原始IMU原点；`/fastlio_odom`已转换为`odom/base_link`，此名称是旧工程的兼容接口，在当前模式下数据来源是R³LIVE。首帧车体在odom下归零，不能据此与另一轮启动的数据直接相减。

适配默认自动输出`/Odometry`（camera_init/body）、`/cloud_registered`（camera_init）、`/cloud_registered_body`（body）、`/cloud_registered_base`（base_link）和`/fastlio_odom`（odom/base_link）。这些接口供现有Bayesian建图、地形过滤和NDT消费；只适配当前配准扫描，不将彩色累计地图当成静态地图。适配不额外降采样，原始R³LIVE算法自身的采样配置保留。位姿输出不提供可用twist，导航速度继续使用原`/scout/odom`。

完整转换和TF所有权见详细信息表。相机、雷达和原始R³LIVE帧现在均接入工程TF树。不要再同时启动FAST-LIO、`tf_manager.launch`、旧pose/cloud adapter或原`scout_localization.launch`；它们会争用同一接口。入口在启动前检查既有发布者。需要旧纯独立观察模式时传`project_interface:=false`，并将RViz切回`r3live_world`及`/r3live/cloud_registered`。

120桌面保存`Scout_R3LIVE.rviz`及`Scout_R3LIVE.desktop`（Scout R3LIVE Viewer快捷入口）。已启动定位后，可双击快捷入口，或执行`~/r3live_ws/view_scout_r3live.sh ~/Desktop/Scout_R3LIVE.rviz`。已有RViz可通过File → Open Config打开桌面的.rviz。不要使用上游示例的/world、/path、/aft_mapped_to_init、/RGB_map_N话题。

配置默认显示当前配准点云（不累积旧扫描）、青色LIO轨迹、紫红色相机轨迹、两路原点坐标轴、视觉跟踪图和1米网格；关闭未经验证的里程计协方差显示。轨迹分别对应IMU与相机原点，两条线间的杆臂偏移不是定位误差。初始视角固定世界坐标，Views可选择俯视或跟随IMU；后者会包含雷达安装倾角。车离开视野时用FocusCamera或切换跟随视角。默认不启用RGB全局地图以减轻Jetson负载，需要时勾选RGB map chunks 0-4组；只显示这5个分块，更大地图须按实际/r3live/RGB_map_N增加显示项。配置没有2D Pose Estimate或导航目标工具，因为当前R³LIVE不消费这些重定位/导航命令。

当前R³LIVE不提供旧地图全局重定位或完整回环后端；重启产生新的局部原点，也不能承诺跟踪丢失后自动找回地图位置。接口已满足原NDT的`/cloud_registered_base`与`/fastlio_odom`输入约定，但本入口不启动NDT，也不发布`map→odom`。后续全局定位仍由原NDT负责；本次通过的是接口合成测试，不代表R³LIVE+NDT实车导航已验收。不要用原`scout_localization.launch`来补NDT，该入口会同时启动FAST-LIO。

停车后 Ctrl+C。每次日志位于 `~/r3live_ws/logs/<日期时间_唯一后缀>/`：`sensors.log`、`estimator.log`、`runtime.yaml`、`rig_snapshot.yaml`、`sensor_snapshot.json`、`ready.json`、`health.json`和`session_result.json`。通用start_scout_r3live.sh默认不录bag，下面的专用test入口自动录制。不保存可用于现有导航的地图，不启用上游离线网格重建；`output/` 是上游工作目录，不代表已经生成可用地图。

**不传calibration_file时，外参仍是安装尺寸初值。** 2026-09-11用户选定六场景loose候选用于室内试用，使用下方专用入口才会加载该矩阵。内参来自真实相机，空间精度及时间偏移尚未独立验收。可以验证启动、图像跟踪和输出连续性；尚不能据此宣布比 FAST-LIO 精度高，或认为暗光、扬尘、无纹理走廊已经解决。配置文件及标定关系见开发文档新增章节；参数、全部试验话题与排错见详细信息表。原 BATF-Nav 的建图、保存和导航流程不变。

### 已选外参的室内试用（2026-09-11）

原rig.yaml、estimator.yaml和启动入口已备份到`~/r3live_ws/config/accepted_20260911/backup/`。试用选取此前六场景画廊的loose候选，不使用后来墙角批次结果。退出其他定位入口、停车后运行：

```bash
~/r3live_ws/start_scout_r3live_test.sh
```

该入口自动带入trial_calibration.yaml并录bag，旧trial目录已删除，当前外参读取`config/accepted_20260911/trial_calibration.yaml`，输出写入`logs/indoor_tests`。每次使用新会话目录，保存runtime.yaml及calibration_snapshot.yaml，不覆盖原配置。停车等待READY，再遥控低速短距离直行、转弯、返回。请记录实际行驶距离与是否回到起点；轨迹平滑或两路输出一致不等于定位准确。

启动自动完成两阶段检查：原图、雷达和IMU连续正常至少5秒；随后两路位姿也连续正常至少5秒，再显示READY。各阶段最多等60秒。检查非零/单调源时间、坐标帧、位姿有限值、四元数及接收延迟。READY后持续检查：断流超过2秒、时间倒退/重复、系统时间跳变或非法位姿会锁定失效，停止本次估计并关闭bag。不会自动重启或发送停车指令；看到RUNTIME_INVALID请人工停车。

启动脚本使用进程锁，重复执行时退出码75，不会通过同名session节点顶掉正在运行的测试。不要删除锁文件来并行启动；停车后退出原入口，再启动新测试。

默认READY后录测10分钟自动退出；可用`test_duration:=60`做1分钟试验。测试时长参数允许30～1800秒。停车Ctrl+C可提前结束。启动前按24MiB/s×（时长+240秒）再加3GiB余量检查磁盘；录制时低于3GiB结束测试。不要在这个专用入口旁再启动第二份相同录包命令。

会话目录新增`sensors_*.bag`、`rosbag.log`、`ready.json`、每5秒更新的`health.json`以及退出后的`session_result.json`。后者说明duration_complete/operator_stopped/failed、故障原因、正常和未完成bag列表。异常退出或断电留下.bag.active不能视为完整记录；即使roslaunch返回0，也应以READY及session_result.json确认本次测试状态。

如提示缺少R3LIVE camera overlay，按开发文档重新执行install_scout_r3live.sh。相机进程exit code -11时先排查OpenCV混用，不要通过反复更换外参或增加等待时间掩盖崩溃。通用入口默认不录bag，专用test入口自动录制，录制状态应以实际rosbag进程和输出文件为准。

另一个终端读取30秒对比结果（每次使用新的输出文件名）：

```bash
source ~/r3live_ws/devel/setup.bash
python3 ~/r3live_ws/observe_indoor.py --seconds 30 --output /tmp/r3live_check_01.json
```

`continuous_finite_outputs`检查连续、有限、时间单调及四元数；`sensor_clock_sanity`检查每个样本接收时刻与图像/IMU时间的粗偏差（-0.1～0.5秒），不能代替时间标定。报告保留最小/最大/中位接收年龄，单次异常不能被中位数掩盖。`max_displacement_from_first_m`和`max_rotation_from_first_deg`是各自物理原点相对首帧的变化：停车时观察稳定性，行驶时只代表运动量，不能当作真值误差或直接相减。跟踪图的header使用发布时刻，不能用于图像时间对齐。

需要运动复盘时可额外录制原始输入与两路输出，最多10分钟、每1GiB分包；本命令不启动车辆，Ctrl+C可以提前结束。例子会生成带日期的bag，仍应每次使用新的试验前缀：

```bash
mkdir -p ~/r3live_ws/logs/manual_bags
timeout --signal=INT --kill-after=20s 600s rosbag record --split --size=1024 --buffsize=128 \
  -o ~/r3live_ws/logs/manual_bags/manual_test \
  /livox/lidar /livox/imu /r3live_camera/color/image_raw \
  /r3live_camera/color/camera_info /r3live_camera/color/metadata /tf_static \
  /r3live/odometry /r3live/camera_odometry /tf /r3live/tf_raw /Odometry /fastlio_odom /rosout
```

录制结束后用`rosbag info <实际bag路径>`检查消息数量与时长；.bag.active仍表示未完成关闭。raw图像约14MB/s，测试前检查剩余空间。回放只选择必要传感器和内部静态TF，不将记录的旧估计输出重新喂入同名在线估计器。

## Scout端相机—雷达标定工具

已增加独立`~/r3live_ws/scout_calibrate.sh`入口，按需复用现有雷达和相机，不关闭原定位。使用hku-mars/livox_camera_calib多场景算法；只需在几个有墙角、门框、柜子等真实三维边缘的地方停车采集，不用拆传感器或抬车，不使用印刷标定板纹理作为雷达边缘。

终端A启动缺少的传感器：

```bash
~/r3live_ws/scout_calibrate.sh sensors
```

终端B在第一个位置停车、放开遥控、确认场景静止后执行，输入YES：

```bash
~/r3live_ws/scout_calibrate.sh capture scene_00
```

默认录制6秒并自动导出BMP/PCD，收到CAPTURE_OK后才能移动车辆。在不同位置依次采集scene_01、scene_02，另外采集validation_00用于独立验证。每次单独停车执行capture，不要连续运行命令冒充不同场景。记录IMU和可用轮速检查，但程序没有车辆控制权，不能替人停车。

```bash
~/r3live_ws/scout_calibrate.sh prepare run_01 scene_00 scene_01 scene_02
~/r3live_ws/scout_calibrate.sh solve run_01
~/r3live_ws/scout_calibrate.sh project run_01 validation_00
```

结果在`~/r3live_ws/calibration/runs/run_01/`。查看validation/validation_00/projection.png和edges.png，检查不同距离和图像区域的对应结构。候选矩阵不自动启用。人工认可独立验证后才能执行：

```bash
~/r3live_ws/scout_calibrate.sh accept run_01 --confirm-validation
```

停止原定位入口后，显式给R³LIVE传入生成的accepted_calibration.yaml；不传仍沿用安装初值：

```bash
~/r3live_ws/start_scout_r3live.sh \
  calibration_file:=/home/nvidia/r3live_ws/calibration/runs/run_01/accepted_calibration.yaml
```

该过程只估计空间外参，未估计时间偏移，不接入NDT或导航。测试目录calibration_test、calibration_synthetic_20260910*里的数据和矩阵不能用于本车正式标定。详细采集规则、旧bag导出、文件结构和错误处理见[标定操作与开发说明](../optional/r3live/calibration/README.md)。

### 实测参数约束研究入口（2026-09-11）

若不同场景求出的外参不稳定，先比较实测基线，不直接采用无约束结果。用户确认雷达位置量到外壳/几何中心；旧rig.yaml却以base_to_imu解释，物理原点尚未精确对应。下面以“测量中心近似为雷达原点”作假设，保留旧解释和旧两次求解对照，不改现有TF、FAST-LIO或导航：

```bash
~/r3live_ws/scout_calibrate.sh refine prior_robust_final_20260911 \
  --training scene_00 scene_01 scene_02 validation_02 \
  --validation validation_00 validation_03 \
  --mount-reference lidar-center-hypothesis \
  --compare-runs run_20260911_01 run_20260911_02
```

首次使用前按标定说明运行install_calibration.sh。以上为已执行研究的名字，复跑改新名字；可以用自己的场景名替换。refine后的`--root`指定数据根目录，默认~/r3live_ws/calibration。训练和验证不重叠；先前叫validation_02的门框数据在本次已用于训练，不再作为独立验收。

程序比较实测基线、旧解释、历史候选和三档先验约束，另做逐一移除训练场景测试。查看`runs/<研究名>/study.yaml`和`comparison/<候选>/<场景>/geometric_edges.png`；绿色为图像边缘，红色为真实平面交线投影。应同时检查多个方向和独立场景，不能仅凭中位像素距离、优化收敛或先验限制后的“小变化”认可精度。失败留下.study_incomplete；保留旧目录，新名字重跑。

该研究入口只生成比较文件，不生成accepted_calibration.yaml，不能直接传给R³LIVE。旋转0.5°/1°/2°和位置5/10/20mm是试验用先验尺度，不是实测精度；外壳中心与厂家雷达原点的偏差仍需核对。完整参数、测试和结果见[标定说明第8节](../optional/r3live/calibration/README.md#8-基于实测安装参数的约束求解与对照2026-09-11)。
