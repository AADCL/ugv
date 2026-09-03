# AADCL UGV

本仓库统一维护 Scout Mini 与 WheelTech 四轮差速机器人两套 ROS1 自主导航工程。两车共享 Livox、FAST-LIO、静态点判定、地形高程、NDT、GlobalPlanner 和 TEB 的总体技术路线，但底盘驱动、通信接口、TF 外参、车体尺寸、速度限制和离地高度分别配置。

| 项目 | 平台与传感器 | 当前链路 | 项目入口 |
|---|---|---|---|
| Scout Mini | AgileX Scout Mini、Jetson、Livox Mid-360、RealSense D435i | CAN + FAST-LIO + 贝叶斯静态点 + Patchwork++ 建图 + PGM/高程坡度 + NDT + GlobalPlanner/TEB | [README](Scout_mini/README.md) · [文档](Scout_mini/docs/) |
| WheelTech | WheelTech 四轮差速底盘、Jetson NX、Livox Mid-360、Orbbec Gemini 336L | 串口底盘 + FAST-LIO + 贝叶斯静态点 + Patchwork++ 建图 + PGM/高程坡度 + NDT + GlobalPlanner/TEB | [README](WheelTech/README.md) · [文档](WheelTech/文档/) |

总体数据流：

```text
建图：Livox -> FAST-LIO -> 贝叶斯静态点 -> 完整PCD + 地面分类 -> PGM + 高程坡度
定位：public_map.pcd + 实时点云 -> NDT -> map -> odom
导航：PGM静态占据 + 保存坡度代价 + 实时相对地面障碍 -> GlobalPlanner + TEB -> 底盘
```

两车都只保留一个建图入口；重定位入口独立持续运行，导航 launch 只启动地图、costmap 和规划器，不重启传感器、FAST-LIO、NDT 或底盘。PGM 用于静态占据与未知区，高程图作为坡度代价和实时相对地面障碍的参考，不是第二套启动入口。

仓库保存项目适配层、必要底盘源码和文档，不重复镜像 FAST-LIO、Livox 驱动/SDK、Patchwork++ 或相机驱动等上游仓库。安装、固定版本、编译、启动与测试步骤以各项目文档为准。

禁止提交私钥、密码、Token、地图、PCD、rosbag、运行日志以及 `build/`、`devel/` 等生成文件。
