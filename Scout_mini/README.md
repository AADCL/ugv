# Scout Mini 自主导航机器人

Scout Mini 项目运行于 Ubuntu 20.04 / ROS Noetic，硬件包括 AgileX Scout Mini、NVIDIA Jetson、Livox Mid-360 和 Intel RealSense D435i。

当前 V4.3 链路与 WheelTech 项目采用相同的地形处理思路，但底盘驱动、CAN 接口、TF 外参、footprint、速度限制和离地高度均由 Scout 独立配置，不能跨车型直接复制参数。

## 当前功能

- Scout CAN 通信、`/cmd_vel` 控制和 `/scout/odom` 轮速里程计；
- Livox Mid-360 与 FAST-LIO 激光惯性里程计；
- 半径滤波、三维贝叶斯静态点判定和动态人员残影清除；
- 单一建图入口同时生成完整 PCD、分类 PCD、PGM 和 2.5D 高程坡度资产；
- NDT-OMP 全局重定位与唯一 `map -> odom` 发布；
- `map_raw` 静态占据、保存坡度代价和实时相对地面障碍的组合导航；
- GlobalPlanner 全局规划和 TEB 局部规划；
- D435i 彩色、深度、CameraInfo 和 TF 独立接入；
- 导航 rosbag 记录和自动分析。

## 数据流

```text
建图（唯一入口 scout_mapping.launch）
Mid-360 -> FAST-LIO -> /cloud_registered
                         |
                         +-> 贝叶斯静态点判定 -> filtered_camera_init.pcd
                                                  +-> /scout/static_scan
                                                       -> 重力对齐
                                                       -> Patchwork++
                                                       -> ground/nonground PCD
                                                       -> PGM + 2.5D高程/坡度

定位（scout_localization.launch，独立持续运行）
public_map.pcd + 实时点云 -> NDT -> map -> odom

导航（navigation_teb.launch，只启动导航层）
map_raw静态占据 + 保存的坡度软代价 -> GlobalPlanner
实时点云 - 保存的同位置地面高程 -> local costmap -> TEB -> /cmd_vel
```

FAST-LIO 始终使用原始 Livox 数据。贝叶斯过滤和 Patchwork++ 都位于 FAST-LIO 输出后处理支路，不回灌 FAST-LIO。正式导航阶段不再运行 Patchwork++，实时障碍由点云相对保存地面高程判定，以适应平地、全局斜坡和局部斜坡组合环境。

## 目录

```text
Scout_mini/
├── docs/
└── src/                       # Git 仓库中的 ROS 包目录
    ├── scout_ros/                  # Scout CAN 底盘
    ├── scout_system_bringup/       # 建图、定位等系统入口
    ├── scout_tf_manager/           # 唯一静态 TF
    ├── scout_pointcloud_mapper/    # 贝叶斯静态 PCD
    ├── scout_terrain_filter/       # Patchwork++ 接入和分类点累积
    ├── scout_2p5d_navigation/      # 高程地图、坡度层、相对地面障碍
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

# 1. 建图；正常 Ctrl+C 后自动保存完整和分类 PCD
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a

# 2. 一次生成定位 PCD、PGM、分类地图和 2.5D 地形资产
rosrun scout_map_tools finalize_map.py factory_a

# 3. 重定位，保持该终端持续运行
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a

# 4. NDT 收敛后，另开终端只启动导航层
roslaunch scout_navigation navigation_teb.launch map_name:=factory_a
```

`navigation_teb.launch` 不包含 Livox、FAST-LIO、NDT 或底盘节点，因此不会抢占或重启已经运行的重定位链。

## 关键参数

- global costmap `inflation_radius: 0.10 m`；
- local costmap `inflation_radius: 0.10 m`；
- TEB `inflation_dist: 0.10 m`；
- Scout 真实 polygon footprint 和原有速度限制保持不变；
- Patchwork++ `sensor_height: 0.48 m`，对应实测雷达中心离地高度；
- 轮胎总高 `0.15 m`，正式相对地面障碍阈值取 `0.08 m`。

障碍膨胀是从障碍边界向外扩展，footprint 是车体真实轮廓，二者含义不同。TEB 的 `min_obstacle_dist: 0.15 m` 也仍是独立的轨迹净空约束。

## 文档

- [开发实施文档 V4.3](docs/Scout_Mini_开发实施文档_V4.3.md)
- [使用文档 V4.3](docs/Scout_Mini_使用文档_V4.3.md)
- [详细信息表 V4.3](docs/Scout_Mini_详细信息表_V4.3.md)
- [GitHub 上传配置与安全说明 V4.3](docs/GitHub_上传配置与安全说明_V4.3.md)

## 安全约束

- 每条 TF 边只能有一个发布者，`map -> odom` 只由 NDT 发布；
- 底盘保持 `pub_tf=false`；
- 正式导航使用 `/scout/odom` 速度反馈；
- 不向 Git 提交私钥、密码、Token、地图、PCD、rosbag、日志、`build/` 或 `devel/`；
- 仓库版本需要在 Scout 实车编译通过后再启动建图或导航。
