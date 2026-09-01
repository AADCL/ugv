# Scout Mini 自主导航机器人

Scout Mini项目基于AgileX Scout Mini底盘、NVIDIA Jetson、Livox Mid-360和Intel RealSense D435i，运行Ubuntu 20.04与ROS Noetic。

当前功能：

- Scout Mini底盘CAN通信、速度控制和轮速里程计；
- Livox Mid-360点云与IMU接入；
- FAST-LIO本地里程计与激光点云配准；
- 注册点云预处理、离群点过滤和三维贝叶斯动态目标清除；
- 自动保存静态PCD地图并生成定位/导航地图资产；
- NDT-OMP全局重定位与唯一`map → odom`发布；
- ROS Navigation、GlobalPlanner和TEB自主导航；
- D435i彩色、深度、CameraInfo与TF接入；
- 全局路径测试、导航rosbag记录和自动分析。

### 系统数据链

```text
Scout CAN ───────────────→ /scout/odom ─────────────┐
                                                    ├→ move_base + TEB → /cmd_vel
Mid-360 + IMU → FAST-LIO → 注册点云与本地里程计 ───┤
                         ├→ 贝叶斯静态地图 → PCD ──┤
                         └→ NDT重定位 → map→odom ──┘

D435i → RGB + Depth + CameraInfo + TF
```

FAST-LIO继续使用原始Livox数据计算里程计。本项目的点云预处理位于FAST-LIO输出端，仅用于交付静态地图，不把处理后的点云回灌FAST-LIO。FAST-LIO原生PCD保存保持关闭，最终PCD由`scout_pointcloud_mapper`生成。

### 目录

```text
Scout_mini/
├── docs/                      # V3.0项目文档
└── src/                       # ROS catkin源码
    ├── scout_ros/
    ├── scout_system_bringup/
    ├── scout_tf_manager/
    ├── scout_pointcloud_mapper/
    ├── scout_map_tools/
    ├── scout_cloud_adapter/
    ├── scout_pose_adapter/
    ├── fast_lio_localization/
    └── scout_navigation/
```

仓库只保存两车的适配代码与必要底盘包，不镜像 FAST-LIO、Livox ROS Driver 2、Livox-SDK2 和 UGV SDK 的完整上游仓库。首次部署时按开发文档固定版本获取这些第三方依赖，再与本目录源码一起放入 catkin 工作空间。

### 环境与编译

```bash
source /opt/ros/noetic/setup.bash
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
```

默认使用`catkin_make -j1`，避免Jetson内存压力以及部分包的并行编译依赖竞态。

### 快速使用

建立CAN接口：

```bash
rosrun scout_bringup bringup_can2usb.bash
ip -details link show can0
```

一键建图：

```bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a
```

建图正常结束后生成地图资产：

```bash
rosrun scout_map_tools finalize_map.py factory_a --replace-raw
```

启动重定位：

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a
```

NDT收敛后启动TEB导航：

```bash
roslaunch scout_navigation navigation_teb.launch map_name:=factory_a
```

启动D435i：

```bash
roslaunch scout_system_bringup D435I.launch
```

记录导航测试：

```bash
roslaunch scout_navigation nav_logging.launch tag:=factory_a_teb_01
```

### 文档

- [完整开发文档 V3.0](docs/Scout_Mini_开发文档_完整版_V3.0.md)
- [自主导航机器人使用手册 V3.0](docs/Scout_Mini_自主导航机器人使用手册_V3.0.md)
- [启动文件、话题、TF与常见问题表 V3.1](docs/scout_话题_启动文件_tftree_常见问题表_V3.1.md)
- [GitHub上传配置与安全说明 V3.0](docs/GitHub_上传配置与安全说明_V3.0.md)

### 关键约束

- 每条TF边只能有一个发布者；
- `map → odom`只由NDT重定位节点发布；
- 底盘驱动保持`pub_tf=false`；
- `/fastlio_odom`的twist为零，TEB速度反馈使用`/scout/odom`；
- 正式导航加载`map_raw.yaml`，当前已验证导航参数未经明确要求不得修改；
- 不向Git提交私钥、密码、Token、PCD、rosbag、地图、日志、`build/`或`devel/`。

详细开发、部署、验证和排错流程以 `docs/` 中的文档为准。
