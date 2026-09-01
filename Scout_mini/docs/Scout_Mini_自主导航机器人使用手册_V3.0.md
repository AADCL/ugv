# Scout Mini 自主导航机器人使用手册

> 本文只提供日常建图、定位和导航操作。参数原理见《开发文档》，接口和故障定位见《接口与排错表》。

## 1. 使用前检查

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_fastlio/devel/setup.bash
ip link show can0
rospack find scout_system_bringup
```

确认Mid-360、D435i、急停和遥控器正常。mapping与localization不能同时启动。

## 2. 建立点云地图

### 2.1 启动CAN

```bash
rosrun scout_bringup bringup_can2usb.bash
```

### 2.2 一键启动

```bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a
```

launch自动启动Mid-360、FAST-LIO、点云过滤和静态地图累积，持续发布过滤扫描及累积地图，每30秒保存一次，并在正常停止时最终保存。无需调用`/finish_mapping`或mapper保存服务。

### 2.3 建图过程

- 低速、平稳驾驶，避免急转和碰撞；
- 从不同方向覆盖门口、拐角和走廊；
- 人员、推车或其他机器人离开后，再次驶过或回看原区域，让空闲射线清除残留；
- 长时间静止目标仍可能暂时进入地图，确认其离开并被重新观测后再结束建图。

```bash
rostopic hz /cloud_registered
rostopic hz /scout/static_scan
rostopic hz /scout/static_map_cloud
ls -lh ~/livox_fastlio/maps/factory_a/filtered_camera_init.pcd
```

### 2.4 停止

在建图终端按一次`Ctrl+C`，等待节点正常退出和最终保存日志。不要使用`kill -9`或直接断电。

### 2.5 生成定位/导航资产

PCD预处理和建图已经自动完成。首次用于定位/导航时转换一次：

```bash
rosrun scout_map_tools finalize_map.py factory_a --replace-raw
ls -lh ~/livox_fastlio/maps/factory_a/
```

应包含`filtered_camera_init.pcd`、`public_map.pcd`、两套PGM/YAML和元数据。

`map.pgm`是供`/map_2d`查看的参考图，固化膨胀为`0.15 m`。当前`navigation.launch`和`navigation_teb.launch`实际加载`map_raw.yaml`（固化膨胀为0），再由costmap按既有配置生成运行时代价，因此当前导航链中两者不会叠加。导航现状正常，不要因调整参考图而修改move_base、costmap或TEB参数。

## 3. 重定位

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a
```

等待FAST-LIO初始化，在RViz用`2D Pose Estimate`给出较准确初值。确认终端出现NDT成功信息、实时点云与历史地图重合，并且完整TF链连通。

```bash
rostopic hz /cloud_registered_base
rosrun tf tf_echo map base_link
```

启动最初数秒“TF树尚未连接”通常是FAST-LIO尚未完成IMU初始化；持续出现才需要排错。

## 4. 导航

重定位成功后运行：

```bash
roslaunch scout_navigation navigation_teb.launch map_name:=factory_a
```

发送`2D Nav Goal`前检查：

```bash
rostopic hz /scout/odom
rostopic hz /cmd_vel
rosrun tf tf_echo map base_link
```

不要在NDT未成功、TF断开或地图名不一致时发送目标。

## 5. D435i

```bash
roslaunch scout_system_bringup D435I.launch
```

D435i当前提供RGB、Depth和相机TF，不作为FAST-LIO/NDT输入。默认不生成D435i PointCloud2以降低Jetson负载，不要重复启动同一相机。

## 6. 正常停止顺序

1. 停止发送导航目标；
2. 停止navigation；
3. 停止localization；
4. mapping模式等待mapper最终保存后停止；
5. 最后关闭CAN和电源。

## 7. 快速排错

| 现象 | 首先检查 |
|---|---|
| 无雷达点云 | `/livox/lidar`、雷达IP、网口 |
| 无过滤地图 | `/cloud_registered`、`/Odometry`、mapper日志 |
| 地面规则孔洞 | 确认使用新版0.05 m细地图体素并重新建图 |
| 重定位点云无输出 | `/cloud_registered_body`和`body → base_link` |
| 启动时TF短暂断开 | 等待FAST-LIO初始化 |
| TF持续断开 | 检查唯一发布者和外参launch |
| NDT不收敛 | 地图名、初值、实时点云、外参 |
| future extrapolation | `map → odom`时间戳和localizer参数 |
| 有路径但车不动 | `/cmd_vel`、CAN、急停/遥控状态 |

## 8. 导航测试日志

日志不会修改导航参数。先启动定位和导航，再开日志：

```bash
roslaunch scout_navigation nav_logging.launch tag:=factory_a_teb_01
```

完成测试后在日志终端按`Ctrl+C`，脚本会正常结束rosbag并自动分析。结果位于：

```bash
cat ~/livox_fastlio/logs/navigation/LAST_RUN
RUN_DIR=$(cat ~/livox_fastlio/logs/navigation/LAST_RUN)
cat "$RUN_DIR/summary.txt"
```

若没有`summary.txt`，检查目录中是否只有`.bag.active`、磁盘是否已满，以及`analysis_console.txt`。完整开发步骤、记录话题和CSV说明见《Scout Mini 自主导航机器人开发文档》第17.13.1节。

## 9. 文档分工

- 本手册：操作者启动、停止、保存和快速排错；
- 《开发文档》：所有必需功能包逐文件开发、编译和验证；
- 《启动文件、节点、话题、TF与常见问题完整表》：运行接口和诊断基准。
