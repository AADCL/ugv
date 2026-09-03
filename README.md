# UGV Scout Mini / WheelTech

本仓库统一维护两套并列、独立部署的 UGV 自主导航工程：Scout Mini 与 WheelTech 四轮差速机器人。

## 项目简介

| 项目 | 平台与传感器 | 主要功能 | 项目入口 |
|---|---|---|---|
| Scout Mini | AgileX Scout Mini、Jetson、Livox Mid-360、RealSense D435i | 底盘 CAN 控制、FAST-LIO 建图、NDT 重定位、ROS Navigation/TEB 导航及 RGB-D 接入 | [项目 README](Scout_mini/README.md) · [项目文档](Scout_mini/docs/) |
| WheelTech | 轮趣四轮差速底盘、Jetson NX、Livox Mid-360、Orbbec Gemini 336L | 串口底盘控制、统一静态点建图、NDT 重定位、PGM+坡度代价、GlobalPlanner、TEB 与 RGB-D 接入 | [项目 README](WheelTech/README.md) · [V4.1 文档](WheelTech/docs/) |

两套工程采用相近的软件数据流：

```text
底盘里程计 + Livox 点云/IMU
          ↓
       FAST-LIO
          ↓
静态地图生成 → NDT 重定位 → 全局规划 + TEB → 底盘速度控制
```

## 目录

```text
ugv_scout_wheeltech/
├── Scout_mini/   # Scout Mini 源码、配置与文档
└── WheelTech/    # 轮趣四轮差速机器人源码、配置与文档
```

两个项目的底盘驱动、通信接口、车体尺寸、TF 外参和导航参数不同，不得直接混用。安装、编译、启动、测试和故障排查方法以各项目目录中的 README 与 `docs/` 为准。

仓库保存项目适配层、必要底盘源码和文档，不重复镜像 FAST-LIO、Livox 驱动/SDK、Patchwork++、Orbbec ROS 驱动等上游仓库；开发文档给出了固定版本和工作空间组装流程。

仓库不提交私钥、密码、Token、地图、PCD、rosbag、运行日志以及 `build/`、`devel/` 等生成文件。
