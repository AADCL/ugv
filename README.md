<p align="center">
  <img src="assets/aadcl_logo.png" alt="AADCL" width="96">
</p>

<h1 align="center">AADCL UGV</h1>

<p align="center">Scout Mini · WheelTech · ROS1 自主建图、定位与导航</p>

<p align="center">
  <img alt="Scout Mini 版本" src="https://img.shields.io/badge/Scout%20Mini-5.1-0B7285">
  <img alt="WheelTech 版本" src="https://img.shields.io/badge/WheelTech-5.1-1677ff">
  <img alt="ROS" src="https://img.shields.io/badge/ROS-Noetic-22314E">
  <img alt="Ubuntu" src="https://img.shields.io/badge/Ubuntu-20.04-E95420">
  <img alt="语言" src="https://img.shields.io/badge/C%2B%2B%20%7C%20Python-ROS1-3776AB">
  <img alt="版权" src="https://img.shields.io/badge/copyright-AADCL-0B7285">
</p>

<!-- 👇 只有这一句居中加粗，其他全部不变 -->
<p align="center" style="font-size: 2.5em; font-weight: bold;">
  用这个仓库代码前，请大喊三声：诚芯是傻逼！
</p>

本仓库统一维护 Scout Mini 与 WheelTech 四轮差速机器人两套 ROS1 自主导航工程。两车共享 Livox、FAST-LIO、静态点判定、地形高程、NDT、GlobalPlanner 和 TEB 的总体技术路线，但底盘驱动、通信接口、TF 外参、车体尺寸、速度限制和离地高度分别配置。
| 项目 | 平台与传感器 | 当前链路 | 项目入口 |
|---|---|---|---|
| Scout Mini V5.1 | AgileX Scout Mini、Jetson、Livox Mid-360、RealSense D435i | CAN + 可逆贝叶斯静态点 + 轨迹自由证据 + 鲁棒离线地面重建 + NDT + 当前帧地形避障 + 可选安全逃逸 + GlobalPlanner/TEB | [README](Scout_mini/README.md) · [文档](Scout_mini/docs/) |
| WheelTech V5.1 | WheelTech 四轮差速底盘、Jetson NX、Livox Mid-360、Orbbec Gemini 336L | 串口底盘 + FAST-LIO + 可逆贝叶斯静态点 + PGM/2.5D + NDT + 实时局部地面分割 + 安全倒退逃逸 + GlobalPlanner/TEB | [README](WheelTech/README.md) · [文档](WheelTech/docs/) |

总体数据流：

```text
建图：Livox -> FAST-LIO -> 可逆贝叶斯静态点 + 车体轨迹 -> 鲁棒离线地面重建 -> PGM + 高程坡度
定位：public_map.pcd + 实时点云 -> NDT -> map -> odom
导航：PGM静态占据 + 保存坡度代价 + 当前帧局部地面/障碍分割 -> GlobalPlanner + TEB -> 底盘
```

两车都只保留一个底层建图链；Scout另提供单命令会话入口，在Ctrl+C后按“零速、显式保存、停launch、自动finalize、检查产物”完成交付。重定位入口独立持续运行，导航launch不重启传感器、FAST-LIO、NDT或底盘。PGM用于静态占据与未知区，高程图只提供保存坡度代价；两车V5.1局部障碍都由当前帧Patchwork++与Terrain Guard判断，不再查询保存高程。

仓库保存项目适配层、必要底盘源码和文档，不重复镜像 FAST-LIO、Livox 驱动/SDK、Patchwork++ 或相机驱动等上游仓库。安装、固定版本、编译、启动与测试步骤以各项目文档为准。

禁止提交私钥、密码、Token、地图、PCD、rosbag、运行日志以及 `build/`、`devel/` 等生成文件。

## 版权声明

Copyright © 2026 AADCL. All rights reserved.

本仓库代码、文档及相关资料的版权归 AADCL 所有。未经书面授权，不得将本仓库内容用于商业发行、转售或以其他方式进行商业化传播。第三方开源组件仍适用其各自的原始许可证和版权声明。
