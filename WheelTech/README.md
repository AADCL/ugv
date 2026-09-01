# 轮趣四轮差速机器人：Livox Mid-360 + FAST-LIO2

本目录保存轮趣四轮差速底盘在 Jetson NX、ROS Noetic 环境下的建图、定位和导航适配代码及中文文档。软件数据流尽量与 Scout 版本保持一致，但底盘驱动、车体尺寸、外参和运动限制均使用轮趣实车参数。

## 已验证硬件与坐标约定

- 计算平台：NVIDIA Jetson NX，Ubuntu 20.04，ROS Noetic。
- 雷达：Livox Mid-360，雷达地址 `192.168.1.165`，NX 有线地址 `192.168.1.5/24`。
- RGB-D 相机：Orbbec Gemini 336L，USB 3.2；驱动、话题和 TF 见 `wheeltec_stack/CAMERA_ORBBEC336L_README.md`。
- 底盘：四轮差速，外形尺寸约 `0.50 m × 0.40 m`。
- 坐标系：右手系、前左上；`base_link -> body` 平移为前 `0.10 m`、上 `0.15 m`，Pitch 为 `+20°`。
- 导航软件限制：前进 `0.35 m/s`、后退 `0.15 m/s`、角速度 `0.80 rad/s`；低于底盘资料物理上限。

## 目录

```text
WheelTech/
├── README.md
├── wheeltec_stack/                 # 轮趣自研 ROS 包
│   ├── wheeltec_system_bringup/    # 建图、定位和系统启动入口
│   ├── wheeltec_tf_manager/        # 外参与 TF
│   ├── wheeltec_pose_adapter/      # FAST-LIO 位姿适配
│   ├── wheeltec_cloud_adapter/     # 点云坐标适配
│   ├── wheeltec_pointcloud_mapper/ # 贝叶斯静态点云建图
│   ├── wheeltec_map_tools/         # PCD/PGM/YAML 地图收尾工具
│   ├── wheeltec_terrain_filter/    # Patchwork++ 后处理、局部地形安全链
│   ├── wheeltec_2p5d_navigation/   # 2.5D 地图、全局规划与 costmap 插件
│   ├── wheeltec_navigation/        # TEB 参数与导航日志
│   └── fast_lio_localization/      # FAST-LIO 地图定位
└── docs/
    ├── 轮趣四轮差速机器人_开发实施文档_V4.0.md
    ├── 轮趣四轮差速机器人_使用文档_V4.0.md
    └── 轮趣四轮差速机器人_详细信息表_V4.0.md
```

`inspect/`、压缩包、构建产物和 SSH 文件只用于本地开发，不应上传到 GitHub。

## 工作空间与依赖

目标工作空间固定为：

```bash
/home/nrc19/livox_fastlio
```

除本目录中的 ROS 包外，工作空间还需要：

- `livox_ros_driver2` 与 `Livox-SDK2`
- `FAST_LIO`（ROS 包名为 `fast_lio`）
- `OrbbecSDK_ROS1` `v2-main`（ROS 包名为 `orbbec_camera`，固定提交 `a2838b3`）
- 轮趣原厂 `turn_on_wheeltec_robot` 和 `wheeltec_robot_rc`
- ROS Noetic 的 `move_base`、`teb_local_planner`、PCL、NDT 等依赖

将 `wheeltec_stack` 下各包复制到工作空间 `src/`，不要把 `wheeltec_stack` 本身作为额外目录层级复制进去。

## 编译

```bash
cd /home/nrc19/livox_fastlio
source /opt/ros/noetic/setup.bash
catkin_make -j1
source devel/setup.bash
```

Jetson NX 建议使用 `-j1`，避免 PCL、FAST-LIO 同时编译造成内存压力。

## RGB-D 相机

```bash
roslaunch wheeltec_system_bringup wheeltec_orbbec336l.launch
```

该入口启动 Gemini 336L 的 RGB、深度、点云、相机内部 TF 和 `base_link -> camera_link` 安装 TF。正式安装外参为前 `0.16 m`、左 `0 m`、上 `0.08 m`，相机与车体同姿态，roll、pitch、yaw 均为 `0`。

## 建图

```bash
roslaunch wheeltec_system_bringup wheeltec_mapping_terrain.launch \
  map_name:=factory_a
```

映射器使用两级体素结构：`0.20 m` 三维贝叶斯状态栅格负责临时障碍判定与自由空间射线清除，`0.05 m` 精细栅格保存最终静态点云。为控制 NX 负载，每四个滤波点追踪一条射线，清除距离限制为 `20 m`。过滤点云每 30 秒自动保存，正常退出时再次保存。

完成采集后在建图终端按一次 `Ctrl+C`，映射器会在正常退出时再次保存 `filtered_camera_init.pcd`。随后单独生成交付地图：

```bash
rosrun wheeltec_map_tools finalize_map.py factory_a --terrain
```

该工具读取保存的原始与分类 PCD，生成 NDT/PGM 兼容资产以及 `terrain_2p5d.yaml` 和高程、坡度、粗糙度、台阶、代价、置信度六层地图。地图保存不依赖手动 finish 服务。

## 定位与导航

```bash
roslaunch wheeltec_system_bringup wheeltec_navigation_2p5d_teb.launch \
  map_name:=factory_a
```

一键入口保留原 PCD/NDT 定位，使用 2.5D 全局规划和 TEB 局部规划。导航使用轮趣实测 `0.50 m × 0.40 m` footprint 和保守运动限制，不能替换为 Scout 的 footprint、CAN 驱动或速度参数。

## 安全要求

- 首次启动先检查 TF、雷达、里程计和急停，不直接发送速度。
- 任何会让车辆移动的测试都应先通知现场人员，并清空车辆周围区域。
- `FAST-LIO` 的 `/Odometry` 与底盘 `/odom` 不是同一个话题，不得互相覆盖。
- 点云过滤结果只用于地图交付，不反馈到 FAST-LIO 前端。
- `self_filter` 在实测边界确认前保持关闭。

完整的逐文件复制、修改、编译和验收流程见 `docs/轮趣四轮差速机器人_开发实施文档_V4.0.md`；日常命令见使用文档，全部参数见详细信息表。V4.0 正式导航使用持久化 2.5D 地图、TerrainGlobalPlanner 与 TEB，不再提供 DWA。
