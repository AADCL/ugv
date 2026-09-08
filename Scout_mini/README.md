# Scout Mini 自主导航机器人

Scout Mini 项目运行于 Ubuntu 20.04 / ROS Noetic，硬件包括 AgileX Scout Mini、NVIDIA Jetson、Livox Mid-360 和 Intel RealSense D435i。

当前 V5.1 链路与 WheelTech 项目采用相同的地形处理思路，但底盘驱动、CAN 接口、TF 外参、footprint、速度限制和离地高度均由 Scout 独立配置，不能跨车型直接复制参数。

## V5.1 更新说明（2026-09-08）

- 贝叶斯静态体素晋升增加命中率门槛，已晋升体素只有在持续自由射线满足次数和时长后才整代撤销，减少行人和其他机器人轨迹残影；
- mapper新增`/fastlio_odom`车体中心轨迹保存，PGM只在真实走过的0.30 m半宽走廊中补充自由证据，障碍证据始终优先；
- 默认建图不再在线累积Patchwork++标签，`finalize_map.py`直接从最终贝叶斯PCD执行PMF种子、鲁棒局部平面生长和相对地面障碍重建；
- 导航局部障碍改为当前帧Patchwork++与Terrain Guard输出，不再查询保存高程，避免定位修正导致局部障碍漂移；
- 增加可选`StartEscapeRecovery`，完成实车后向覆盖验证前默认关闭；启用后也只有全局起点确定为致命占据、局部倒车走廊无碰撞且雷达后方覆盖新鲜时才允许0.05 m/s低速逃逸；
- 新增明确的`base_link_height_above_ground=0.28 m`，离线地面种子不再误用0.20 m刚性雷达Z偏移。
- 新增`scout_mapping_session.py`单命令会话入口，在Ctrl+C后先保持零速并显式保存，再停launch、自动生成和校验全部地图资产；
- 当前帧感知最小水平半径为0.12 m（只在正前方约对应车头边界），无参考低障碍保守兜底，地面平面拟合失败时不允许清空costmap；
- 离线和当前帧最大可通行坡度统一为22度，并为mapper与离线稠密网格增加Jetson容量硬门；
- 按已确认需求将PGM建图膨胀设为0.15 m，已正常工作的move_base/TEB参数保持不变；
- finalize全过程由`.finalization_incomplete`标志保护，定位和导航入口常驻地图guard，拒绝未完成、截断或缺文件的地图包。

## 当前功能

- Scout CAN 通信、`/cmd_vel` 控制和 `/scout/odom` 轮速里程计；
- Livox Mid-360 与 FAST-LIO 激光惯性里程计；
- 半径滤波、三维贝叶斯静态点判定和动态人员残影清除；
- 单一建图入口保存贝叶斯静态PCD和车体轨迹，收尾工具统一生成分类PCD、PGM和2.5D高程坡度资产；
- NDT-OMP 全局重定位与唯一 `map -> odom` 发布；
- `map_raw`静态占据、保存坡度软代价和当前帧地面/障碍分割的组合导航；
- 带后向点云覆盖门控的起点安全逃逸恢复；
- GlobalPlanner 全局规划和 TEB 局部规划；
- D435i 彩色、深度、CameraInfo 和 TF 独立接入；
- 导航 rosbag 记录和自动分析。

## 数据流

```text
建图（推荐入口 scout_mapping_session.py；底层 scout_mapping.launch）
Mid-360 -> FAST-LIO -> /cloud_registered
                         |
                         +-> 可逆贝叶斯静态点 -> filtered_camera_init.pcd
                         +-> /fastlio_odom -> traversed_path_map.pcd
                                              |
                                              +-> finalize_map.py
                                                   -> 鲁棒离线地面重建
                                                   -> public_map + PGM + 2.5D

定位（scout_localization.launch，独立持续运行）
public_map.pcd + 实时点云 -> NDT -> map -> odom

导航（navigation_teb.launch，只启动导航层）
map_raw静态占据 + 保存的坡度软代价 -> GlobalPlanner
实时点云 -> 重力对齐 -> Patchwork++ -> Terrain Guard
                                   -> marking/clearing -> local costmap
                                   -> TEB -> /cmd_vel
```

FAST-LIO始终使用原始Livox数据，任何过滤结果都不回灌FAST-LIO。正式建图默认只运行轻量贝叶斯mapper，Patchwork++在线分类仅在`enable_online_terrain_diagnostics:=true`时启用；正式导航则使用当前帧Patchwork++和Terrain Guard做局部碰撞感知，保存高程只参与全局坡度软代价。

## 目录

```text
Scout_mini/
├── docs/
└── src/                       # Git 仓库中的 ROS 包目录
    ├── scout_ros/                  # Scout CAN 底盘
    ├── scout_system_bringup/       # 建图、定位等系统入口
    ├── scout_tf_manager/           # 唯一静态 TF
    ├── scout_pointcloud_mapper/    # 贝叶斯静态 PCD
    ├── scout_terrain_filter/       # Patchwork++ 与当前帧 Terrain Guard
    ├── scout_2p5d_navigation/      # 高程坡度层和安全逃逸恢复
    ├── scout_map_tools/            # PCD/PGM/2.5D 最终生成
    ├── scout_cloud_adapter/
    ├── scout_pose_adapter/
    ├── fast_lio_localization/
    └── scout_navigation/           # GlobalPlanner + TEB
```

仓库保存适配代码和必要底盘包，不重复镜像 Patchwork++、FAST-LIO、Livox ROS Driver 2、Livox-SDK2 和 UGV SDK 的完整上游仓库。第三方依赖按开发文档放入 `~/livox_fastlio/src`。

## 编译

```bash
source /opt/ros/noetic/setup.bash
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
```

Jetson 默认使用 `-j1`，避免内存压力和包间并行编译竞态。

## 快速使用

```bash
# CAN
rosrun scout_bringup bringup_can2usb.bash

# 1. 一键建图；车辆停稳后按一次Ctrl+C，自动保存并生成全部地图资产
rosrun scout_system_bringup scout_mapping_session.py factory_a

# 2. 重定位，保持该终端持续运行
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a

# 3. NDT 收敛后，另开终端只启动导航层
roslaunch scout_navigation navigation_teb.launch map_name:=factory_a
```

`navigation_teb.launch` 不包含 Livox、FAST-LIO、NDT 或底盘节点，因此不会抢占或重启已经运行的重定位链。
两个运行入口启动时由guard检查地图事务/容量标志和各自需要的文件；guard报错会让对应launch整体停止，必须重新成功执行地图收尾，不能手工删除标志绕过。

## 关键参数

- global costmap `inflation_radius: 0.10 m`；
- local costmap `inflation_radius: 0.10 m`；
- TEB `inflation_dist: 0.10 m`；
- PGM离线建图 `obstacle_inflation_m: 0.15 m`，与全局运行时0.10 m依次生效；
- Scout 真实 polygon footprint 和原有速度限制保持不变；
- Patchwork++ `sensor_height: 0.48 m`，对应实测雷达中心离地高度；
- `base_link_height_above_ground: 0.28 m`，由0.48 m雷达高度减去0.20 m刚性Z偏移得到；
- 轮胎总高 `0.15 m`，正式相对地面障碍阈值取 `0.08 m`。
- 当前帧最小水平距离`0.12 m`，保存/当前帧最大可通行坡度`22 deg`。

障碍膨胀是从障碍边界向外扩展，footprint 是车体真实轮廓，二者含义不同。TEB 的 `min_obstacle_dist: 0.15 m` 也仍是独立的轨迹净空约束。

## 文档

- [开发实施文档 V5.1](docs/Scout_Mini_开发实施文档_V5.1.md)
- [使用文档 V5.1](docs/Scout_Mini_使用文档_V5.1.md)
- [详细信息表 V5.1](docs/Scout_Mini_详细信息表_V5.1.md)
- [GitHub 上传配置与安全说明 V5.1](docs/GitHub_上传配置与安全说明_V5.1.md)

## 安全约束

- 每条 TF 边只能有一个发布者，`map -> odom` 只由 NDT 发布；
- 底盘保持 `pub_tf=false`；
- 正式导航使用 `/scout/odom` 速度反馈；
- `.finalization_incomplete`或`filtered_camera_init.pcd.capacity_limited`存在时，禁止定位、导航和地图交付；
- 不向 Git 提交私钥、密码、Token、地图、PCD、rosbag、日志、`build/` 或 `devel/`；
- 仓库版本需要在 Scout 实车编译通过后再启动建图或导航。
