<p align="center">
  <img src="../assets/aadcl_logo.png" alt="AADCL" width="96">
</p>

<h1 align="center">轮趣四轮差速机器人</h1>

<p align="center">Livox Mid-360 · FAST-LIO2 · 2.5D 地形感知 · GlobalPlanner/TEB</p>
<p align="center">
  <img alt="版本" src="https://img.shields.io/badge/version-5.1-1677ff">
  <img alt="ROS" src="https://img.shields.io/badge/ROS-Noetic-22314E">
  <img alt="Ubuntu" src="https://img.shields.io/badge/Ubuntu-20.04-E95420">
  <img alt="平台" src="https://img.shields.io/badge/platform-Jetson%20NX-76B900">
  <img alt="雷达" src="https://img.shields.io/badge/LiDAR-Livox%20Mid--360-00A6A6">
  <img alt="版权" src="https://img.shields.io/badge/copyright-AADCL-0B7285">
</p>

本目录保存轮趣四轮差速底盘在 Jetson NX、ROS Noetic 环境下的建图、定位和导航适配代码及中文文档。软件数据流尽量与 Scout 版本保持一致，但底盘驱动、车体尺寸、外参和运动限制均使用轮趣实车参数。
> **⚠️ 用这个仓库代码前，请大喊三声：诚芯是傻逼！着陆页是傻逼！**

## V5.1 更新说明（2026-09-08）

- 局部避障取消实时点查询保存 `terrain_2p5d` 高程的耦合链路，改为 Patchwork++ 当前帧地面分割与 Terrain Guard；局部障碍不再随 `map -> odom` 定位修正直接漂移。
- 局部 costmap 改订阅 `/terrain/obstacle_points` 与 `/terrain/clearing_points`，历史观测保持由 `0.20 s` 改为 `0 s`，避免门口旧点拖影。
- 60 cm门洞模式使用真实 `0.50 m × 0.40 m` footprint、`footprint_padding=0`、全局膨胀 `0.15 m`、局部膨胀 `0.22 m`；TEB额外净空由3 cm微调为4 cm，减少后轮擦碰。
- 新增 `StartEscapeRecovery`：除全局/局部代价检查外还强制要求新鲜后向点云覆盖。实测当前雷达在车后0.15～0.70 m近场为0点，因此自动倒退会安全拒绝；接入后置传感器后才可启用实际倒退。
- 新增《点云处理与导航链路 V5.1》Markdown技术文档，逐级给出贝叶斯、射线清除、鲁棒地面、坡度代价、NDT、实时地形分类和逃逸安全门的公式、目的、ROS包、节点、源码、参数与验收方法。

V5.1继续保留V5.0的以下地图质量改进：

- 在线建图以 `0.20 m` 可逆贝叶斯体素作为唯一动态/静态权威；静态晋升调整为至少 12 次命中、跨越 2 秒且命中率不低于 60%，已晋升体素只有在至少 8 次自由射线、持续 0.75 秒后才清除。
- 保存 `base_link` 轨迹并在 PGM 转换时生成 `0.20 m` 扫掠自由走廊，改善相同路线重复建图时的稀疏点云和未探索空洞；障碍证据始终覆盖轨迹自由证据。
- 地面重建改为保守 PMF 种子、分布式锚点和鲁棒局部平面连续生长，在保留上下坡面的同时限制墙面、顶棚和孤立边缘误入地面。
- PGM 是全局静态障碍的唯一权威；`TerrainCostmapLayer` 只叠加坡度代价，默认不再把同源 nonground 点以粗栅格重复写成致命障碍。
- V5.0曾把全局和局部costmap膨胀统一调整为 `0.22 m`；V5.1进一步将全局改为 `0.15 m`、局部保留 `0.22 m`，并取消额外padding。
- 使用 129 秒 `indoor_slope_v2.bag` 回归：静态 PGM 的 6494 个致命格全部保留，最终全局图为 6573 个致命格；地形层额外致命格由修改前约 5994 个降为 79 个。

## 已验证硬件与坐标约定

- 计算平台：NVIDIA Jetson NX，Ubuntu 20.04，ROS Noetic。
- 雷达：Livox Mid-360，雷达地址 `192.168.1.165`，NX 有线地址 `192.168.1.5/24`。
- RGB-D 相机：Orbbec Gemini 336L，USB 3.2；驱动、话题和 TF 见 `wheeltec_stack/CAMERA_ORBBEC336L_README.md`。
- 底盘：四轮差速，外形尺寸约 `0.50 m × 0.40 m`。
- 坐标系：右手系、前左上；`base_link -> body` 平移为前 `0.10 m`、上 `0.15 m`，Pitch 为 `+20°`；`base_link` 离地 `0.15 m`，Livox 离地 `0.30 m`。
- 导航软件限制：前进 `0.20 m/s`、后退 `0.15 m/s`、角速度 `0.40 rad/s`；低于底盘资料物理上限。

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
│   ├── wheeltec_terrain_filter/    # 可选 Patchwork++/地形诊断（默认建图不启动）
│   ├── wheeltec_2p5d_navigation/   # 保存高程/坡度、坡度层与起点安全逃逸
│   ├── wheeltec_navigation/        # TEB 参数与导航日志
│   └── fast_lio_localization/      # FAST-LIO 地图定位
└── docs/
    ├── 轮趣四轮差速机器人_开发实施文档_V5.1.md
    ├── 轮趣四轮差速机器人_使用文档_V5.1.md
    ├── 轮趣四轮差速机器人_详细信息表_V5.1.md
    └── 轮趣四轮差速机器人_点云处理与导航链路_V5.1.md
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
roslaunch wheeltec_system_bringup wheeltec_mapping.launch \
  map_name:=factory_a
```

映射器使用两级体素结构：`0.20 m` 三维贝叶斯状态栅格是唯一动态/静态权威，`0.05 m` 精细栅格保存最终静态点云。静态晋升要求至少 12 次命中、跨越 2 秒且命中率不低于 60%；已晋升体素只有在至少 8 次自由射线并持续 0.75 秒后才清除。自由空间使用三维 DDA 完整遍历每条选中射线穿过的体素，NX 默认每两个滤波点取一条射线，清除距离限制为 `20 m`。过滤点云和 `base_link` 轨迹每 30 秒自动保存，正常退出时再次保存。

正式建图默认只维护这一份贝叶斯静态 PCD，不再在线运行 Patchwork++ 和地形累积器。需要回归旧分类链路时，才显式设置 `enable_online_terrain_diagnostics:=true`；`terrain_guard` 仍只是可选诊断。导航局部避障继续读取实时点云，以保留对移动障碍的响应。

完成采集后在建图终端按一次 `Ctrl+C`，映射器会在正常退出时再次保存 `filtered_camera_init.pcd`。随后单独生成交付地图：

```bash
rosrun wheeltec_map_tools finalize_map.py factory_a
```

该工具先确认 `filtered_camera_init.pcd` 与归档的 `raw_camera_init.pcd` 属于同一次建图，再直接以最终贝叶斯 `public_map.pcd` 为唯一来源：大窗口 PMF 只产生保守地面种子，鲁棒局部平面把种子沿连续坡面扩展，并按相对地面 `0.04-1.50 m` 重新提取障碍。窄墙链、台阶和断开的顶棚不能仅凭逐格高度差进入地面；地面插值还要求附近存在原点云 XY 观测。随后生成 NDT/PGM 兼容资产以及 `terrain_2p5d.yaml` 和六层地形文件。轨迹扫掠区域作为显式自由证据，PGM 障碍仍有最高优先级；2.5D 文件只保存高程和坡度代价，不重复融合静态障碍。地图保存不依赖手动 finish 服务。

## 定位与导航

```bash
roslaunch wheeltec_system_bringup wheeltec_localization.launch \
  map_name:=factory_a

# 定位稳定后另开终端
roslaunch wheeltec_navigation navigation_teb.launch \
  map_name:=factory_a
```

定位和导航入口职责分离：定位入口只启动一次并持续提供传感器、NDT、TF、底盘与 `/odom`；导航入口只叠加地图、当前帧局部地面分割、GlobalPlanner、TEB 和 move_base，不会抢占重定位或底盘节点。PGM负责固定障碍和未知区域，保存高程只提供全局坡度软代价，局部障碍不查询保存高程。

## 安全要求

- 首次启动先检查 TF、雷达、里程计和急停，不直接发送速度。
- 任何会让车辆移动的测试都应先通知现场人员，并清空车辆周围区域。
- `FAST-LIO` 的 `/Odometry` 与底盘 `/odom` 不是同一个话题，不得互相覆盖。
- 点云过滤结果只用于地图交付，不反馈到 FAST-LIO 前端。
- `self_filter` 在实测边界确认前保持关闭。

完整的逐文件复制、修改、编译和验收流程见 `docs/轮趣四轮差速机器人_开发实施文档_V5.1.md`；日常命令见使用文档，全部参数见详细信息表，点云全链路及图示见 `docs/轮趣四轮差速机器人_点云处理与导航链路_V5.1.md`。V5.1正式链路为PGM唯一静态占据、保存高程坡度代价、当前帧局部地面分割、GlobalPlanner、TEB与安全倒退逃逸，不提供DWA。
