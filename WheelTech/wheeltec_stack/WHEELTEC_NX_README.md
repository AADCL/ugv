# 轮趣 NX：Livox + FAST-LIO2 使用说明

## 当前硬件与坐标

- ROS Noetic 工作空间：`/home/nrc19/livox_fastlio`
- Livox Mid-360：`192.168.1.165`，NX `eth0`：`192.168.1.5/24`
- 坐标系采用 ROS 前、左、上。
- `base_link -> body`：前 `0.10 m`、左 `0 m`、上 `0.15 m`、绕 Y 轴 `+20 deg`（雷达朝车头方向压低 20°）。
- 底盘：四轮差速，外形长 `0.50 m`、宽 `0.40 m`。
- 规划 footprint：`[[0.25,0.20],[0.25,-0.20],[-0.25,-0.20],[-0.25,0.20]]`，padding `0.03 m`。

## 数据流

`/livox/lidar`、`/livox/imu` -> FAST-LIO2 -> `/Odometry`、`/cloud_registered` -> 点云/位姿适配 -> NDT 定位 (`map -> odom`) -> move_base + TEB -> `/cmd_vel` -> 轮趣底盘。

底盘反馈保持轮趣原生话题：`/odom`、`/imu`、`/PowerVoltage`；不使用 Scout 底盘驱动，也不包含 RGB 相机。

## 建图

```bash
source /home/nrc19/livox_fastlio/devel/setup.bash
roslaunch wheeltec_system_bringup wheeltec_mapping.launch map_name:=site_01
```

车辆完成低速巡视后，在建图终端按一次 `Ctrl+C`，等待映射器正常退出并完成最终自动保存。随后生成全部地图制品：

```bash
source /home/nrc19/livox_fastlio/devel/setup.bash
rosrun wheeltec_map_tools finalize_map.py site_01
```

文件位于 `/home/nrc19/livox_fastlio/maps/site_01/`，包括原始/公共 PCD、原始/导航 PGM+YAML 和元数据。

## 定位

```bash
roslaunch wheeltec_system_bringup wheeltec_localization.launch map_name:=site_01
```

需要手工重定位时，可在 RViz 发布 `/initialpose`。核心 TF 链为 `map -> odom -> camera_init -> body -> base_link`。

## 导航

先启动定位，再启动导航：

```bash
roslaunch wheeltec_system_bringup wheeltec_navigation_2p5d_teb.launch map_name:=site_01
```

初始软件限制采用保守值：前进 `0.30 m/s`、倒车 `0.15 m/s`、角速度 `0.8 rad/s`、线加速度 `0.2 m/s^2`、角加速度 `0.3 rad/s^2`。底盘驱动资料给出的物理上限为前后 `0.5 m/s`、角速度 `1.5 rad/s`；导航参数不得超过物理上限。

## 上位机 ROS 控制

上位机与 NX 在同一网络时，将 ROS master 指向 NX：

```bash
export ROS_MASTER_URI=http://192.168.50.122:11311
export ROS_IP=<上位机在同网段的地址>
```

读取底盘状态：

```bash
rostopic echo /odom
rostopic echo /imu
rostopic echo /PowerVoltage
```

人工测试控制前请架空车轮或清空车辆周围，低速、短时发送：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

按 `Ctrl-C` 后再发布一次零速度。四轮差速只使用 `linear.x` 与 `angular.z`，`linear.y` 必须为零。

## 验收检查

```bash
ls -l /dev/wheeltec_controller
rostopic hz /livox/lidar /livox/imu /Odometry /cloud_registered /odom
rosrun tf tf_echo base_link body
```

预期 Livox 点云约 `10 Hz`、IMU 约 `200 Hz`、FAST-LIO 输出约 `10 Hz`、底盘 `/odom` 约 `20 Hz`。

安装验收时底盘 USB 控制器曾从 `lsusb` 消失，因此最终实车速度闭环与导航行走测试须在恢复底盘 USB 供电/连接后进行。
