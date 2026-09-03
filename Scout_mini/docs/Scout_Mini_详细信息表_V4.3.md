# Scout Mini 自主导航机器人详细信息表 V4.3

> 本表对应 2026-09-03 本地源码和 Scout 部署基线。当前正式架构为贝叶斯静态点、分类 PGM、保存高程坡度代价、实时高程相对障碍、GlobalPlanner 和 TEB。

## 1. 平台信息

| 项目 | 当前值 |
|---|---|
| 车型 | AgileX Scout Mini，四轮滑移转向 |
| 操作系统 | Ubuntu 20.04 |
| ROS | Noetic |
| 计算平台 | NVIDIA Jetson |
| Scout 登录 | `nvidia@192.168.50.120` |
| ROS 工作空间 | `/home/nvidia/livox_fastlio` |
| 激光雷达 | Livox Mid-360 |
| 主机雷达网口 | 通常 `192.168.1.5/24` |
| RGB-D | Intel RealSense D435i，当前不进入导航链 |
| 底盘接口 | CAN / `can0` |
| 底盘里程计 | `/scout/odom` |
| 底盘速度命令 | `/cmd_vel` |
| 组织仓库 | `git@github.com:AADCL/ugv.git` |
| 组织仓库 NX 克隆 | `/home/nrc19/github_upload/ugv_scout_wheeltech` |

## 2. 车体、外参与高度状态

### 2.1 已保留的 Scout 参数

| 参数 | 当前值 | 说明 |
|---|---:|---|
| footprint 前端 | `x=0.370 m` | `costmap_common.yaml` |
| footprint 后端 | `x=-0.300 m` | 同上 |
| footprint 左右 | `y=+/-0.295 m` | 同上 |
| footprint padding | `0.03 m` | 未修改 |
| Livox 相对 base_link 前向 | `0.25 m` | 原 Scout 外参 |
| Livox 相对 base_link 上向 | `0.20 m` | 刚体外参，不等于离地高度 |
| Livox pitch | `+45 deg` | 原 Scout 配置 |
| 地形坐标原点 | `[0.25, 0, 0.20] m` | 与 Livox 同原点、姿态与车体一致 |

### 2.2 实测行走相关高度

| 参数 | 当前值 | 状态 |
|---|---:|---|
| Patchwork++ `sensor_height` | `0.48 m` | 雷达中心绝对离地实测 |
| 轮胎总高 | `0.15 m` | 实测 |
| 正式相对障碍下限 | `0.08 m` | 约为轮半径并向上取整 |

`sensor_height` 只表达雷达中心绝对离地高度。除非重新测得刚体安装位置变化，否则不改 `extrinsics.yaml` 的 `0.25/0.20/45 deg`。

## 3. TF 所有权

```text
map
└── odom
    └── camera_init
        └── body
            └── base_link
                ├── terrain_sensor
                ├── camera_link
                └── Scout 车体/轮系内部 frame
```

| TF 边 | 类型 | 唯一发布者 | 用途 |
|---|---|---|---|
| `map -> odom` | 动态 | NDT 全局定位节点 | 全局校正 |
| `odom -> camera_init` | 静态/几何 | `scout_geometry_tf_publisher` | 与地图导出参数一致 |
| `camera_init -> body` | 动态 | FAST-LIO | 激光惯性位姿 |
| `body -> base_link` | 静态 | `scout_tf_manager` | Livox 到车体的逆变换 |
| `base_link -> terrain_sensor` | 静态 | `scout_tf_manager` | 重力对齐的地形分割坐标 |
| D435i 内部 TF | 静态 | RealSense 驱动 | 相机内部外参 |

Scout 底盘 launch 必须设置 `pub_tf=false`。同一 TF 边出现两个发布者会造成点云跳变、NDT 不稳定和规划异常。

## 4. 正式入口

| 模式 | 命令 | 是否包含底盘 | 说明 |
|---|---|---:|---|
| 建图 | `roslaunch scout_system_bringup scout_mapping.launch map_name:=NAME` | 是 | 唯一建图入口，同时保存完整与分类 PCD |
| 地图最终生成 | `rosrun scout_map_tools finalize_map.py NAME` | 否 | 一次生成 PCD、PGM、2.5D 高程坡度资产 |
| 重定位 | `roslaunch scout_system_bringup scout_localization.launch map_name:=NAME` | 是 | Livox、FAST-LIO、NDT、TF、底盘，持续运行 |
| 正式导航层 | `roslaunch scout_navigation navigation_teb.launch map_name:=NAME` | 否 | 必须复用同名地图的定位入口 |
| 全局规划测试 | `roslaunch scout_navigation global_planning_test.launch map_name:=NAME` | 否 | 屏蔽实际速度输出 |
| 导航日志 | `roslaunch scout_navigation nav_logging.launch tag:=TAG` | 否 | 只记录和分析 |
| D435i | `roslaunch scout_system_bringup D435I.launch` | 否 | 相机独立入口 |

已经删除的旧入口和配置包括 `navigation.launch`、DWA 参数和非坡度版本的 move_base/costmap 配置。正式局部规划器只保留 TEB。

## 5. 建图数据流

```text
/livox/lidar + /livox/imu
  -> FAST-LIO
     +-> /Odometry
     +-> /cloud_registered
     +-> /cloud_registered_body
     +-> camera_init -> body

/cloud_registered + /Odometry
  -> scout_pointcloud_mapper
     +-> 半径离群点过滤
     +-> 3D 贝叶斯静态点判定
     +-> /scout/static_scan
     +-> filtered_camera_init.pcd

/scout/static_scan
  -> scout_terrain_cloud_adapter -> /cloud_registered_terrain (terrain_sensor)
  -> Patchwork++
     +-> /terrain/patchwork_ground
     +-> /terrain/patchwork_nonground
  -> scout_terrain_map_accumulator
     +-> terrain_ground_camera_init.pcd
     +-> terrain_obstacles_camera_init.pcd
```

完整 PCD 和地形分类共用 `/scout/static_scan` 的贝叶斯静态判定。导航阶段则直接使用实时 `/cloud_registered_body`，以免移动障碍被建图期静态滤波丢弃。

## 6. 地图最终生成流

```text
filtered_camera_init.pcd
  -> raw_camera_init.pcd
  -> public_map.pcd

terrain_*_camera_init.pcd
  -> terrain_*_map.pcd
  +-> map_raw.pgm/yaml
  +-> map.pgm/yaml
  +-> terrain_cost.pgm/yaml
  +-> terrain_2p5d.yaml + 六层二进制文件
```

| 文件 | 坐标/类型 | 消费者 |
|---|---|---|
| `filtered_camera_init.pcd` | `camera_init` | mapper 最终完整 PCD |
| `raw_camera_init.pcd` | `camera_init` | finalize 归档 |
| `public_map.pcd` | `map` | NDT 定位 |
| `terrain_ground_camera_init.pcd` | `camera_init` | 建图期分类累积 |
| `terrain_obstacles_camera_init.pcd` | `camera_init` | 建图期分类累积 |
| `terrain_ground_map.pcd` | `map` | PGM 和高程构建 |
| `terrain_obstacles_map.pcd` | `map` | PGM 和高程障碍融合 |
| `map_raw.yaml` | 2D OccupancyGrid | 正式导航静态层 |
| `map.yaml` | 2D OccupancyGrid | 定位显示兼容 |
| `terrain_2p5d.yaml` | 高程地图索引 | 地形服务、坡度层和在线相对障碍 |
| `terrain_cost.yaml` | 2D 诊断图 | 坡度/障碍可视化 |
| `map_metadata.yaml` | YAML | 外参和地图生成参数快照 |

PGM 不做离线障碍膨胀，避免与 move_base 的运行时膨胀叠加。

## 7. 导航数据流

```text
scout_localization.launch（持续运行）
public_map.pcd + 实时 base 点云 -> NDT -> map -> odom
/scout/odom -------------------------------> TEB速度反馈

navigation_teb.launch（纯导航层）
map_raw.yaml -> /nav_static_map -> global static layer
terrain_2p5d.yaml -> TerrainCostmapLayer -> global slope cost
/cloud_registered_body -> terrain_sensor
  -> 保存高程相对障碍/清除点
  -> local obstacle layer
GlobalPlanner -> TEB -> /cmd_vel
```

导航入口不包含重定位，因此不会销毁或替换已经运行的 `map -> odom`。

## 8. 关键节点

模式：M=建图，L=定位，N=导航，C=相机，O=离线工具。

| 节点 | 模式 | 主要输入 | 主要输出 |
|---|---|---|---|
| `/laserMapping` | M/L/N | Livox 点云与 IMU | `/Odometry`、注册点云、FAST-LIO TF |
| Scout base 节点 | M/L/N | CAN、`/cmd_vel` | `/scout/odom` |
| `/scout_pointcloud_mapper` | M | 注册点云、`/Odometry` | `/scout/static_scan`、完整 PCD |
| `/scout_terrain_cloud_adapter` | M/N | M:静态扫描；N:实时 body 点云 | `/cloud_registered_terrain` |
| `/scout_patchworkpp` | M | 重力对齐点云 | ground/nonground |
| `/scout_terrain_guard` | M | 分类点 | 诊断、安全点云和状态 |
| `/scout_terrain_map_accumulator` | M | ground/nonground、`/Odometry` | 两份分类 PCD |
| `terrain_map_builder_node` | O | map 坐标分类 PCD | `terrain_2p5d.*` |
| `/scout_geometry_tf_publisher` | M/L/N | 几何配置 | `odom -> camera_init` |
| `/scout_tf_manager` | M/L/N | `extrinsics.yaml` | 静态车体 TF |
| `/scout_pose_adapter` | M/L/N | TF | `/fastlio_odom` |
| `/scout_map_loader` | L/N | `public_map.pcd` | `/map_cloud` |
| Scout NDT localizer | L/N | 地图、实时点云、初值 | `map -> odom` |
| `/scout_navigation_map_server` | N | `map_raw.yaml` | `/nav_static_map` |
| `/scout_terrain_map_server` | N | `terrain_2p5d.yaml` | 高程、坡度、代价话题 |
| `/scout_terrain_relative_obstacle_filter` | N | 实时点云、保存高程 | 在线 obstacle/clearing 点云 |
| `/move_base` | N | 静态图、坡度、局部障碍、TF | 路径和 `/cmd_vel` |
| RealSense 节点 | C | D435i USB | RGB、深度、CameraInfo、TF |

## 9. 关键话题

| 话题 | 类型 | 含义 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Mid-360 点云 |
| `/livox/imu` | `sensor_msgs/Imu` | Mid-360 IMU |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO `camera_init/body` 位姿 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | `camera_init` 注册点云 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | FAST-LIO 当前 body 点云 |
| `/cloud_registered_base` | `sensor_msgs/PointCloud2` | NDT 实时输入 |
| `/scout/static_scan` | `sensor_msgs/PointCloud2` | 贝叶斯静态扫描，建图公共输入 |
| `/cloud_registered_terrain` | `sensor_msgs/PointCloud2` | 重力对齐地形点云 |
| `/terrain/patchwork_ground` | `sensor_msgs/PointCloud2` | Patchwork++ 地面分类 |
| `/terrain/patchwork_nonground` | `sensor_msgs/PointCloud2` | Patchwork++ 非地面分类 |
| `/terrain/elevation_obstacle_points` | `sensor_msgs/PointCloud2` | 相对保存地面高 0.08-1.50 m |
| `/terrain/elevation_clearing_points` | `sensor_msgs/PointCloud2` | local costmap 射线清除点 |
| `/nav_static_map` | `nav_msgs/OccupancyGrid` | 正式全局静态图 |
| `/terrain/elevation` | `nav_msgs/OccupancyGrid` | 高程可视化 |
| `/terrain/slope` | `nav_msgs/OccupancyGrid` | 坡度可视化 |
| `/terrain/traversability` | `nav_msgs/OccupancyGrid` | 通行代价可视化 |
| `/scout/odom` | `nav_msgs/Odometry` | 底盘速度反馈 |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘控制指令 |

## 10. 地图与障碍参数

### 10.1 Patchwork++ 和分类累积

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `sensor_height` | `0.48 m` | 雷达中心绝对离地实测 |
| Patchwork 最小/最大距离 | `0.25 / 12.0 m` | 单帧分割范围 |
| 分类 PCD 体素 | `0.05 m` | 累积分辨率 |
| 障碍最少帧 | `3` | 动态残影抑制 |
| 最小观察跨度 | `0.4 s` | 动态残影抑制 |
| 正式障碍相对高度 | `0.08-1.50 m` | 轮胎直径 0.15 m |
| guard 障碍诊断下限 | `0.06 m` | 建图期保守诊断，不直接作为正式地图阈值 |
| guard 最大台阶参考 | `0.08 m` | 约轮半径 |

### 10.2 2.5D 高程坡度

| 参数 | 当前值 |
|---|---:|
| resolution | `0.10 m` |
| ground percentile | `0.20` |
| fill radius | `4 cells` |
| fit radius | `0.30 m` |
| preferred slope | `5 deg` |
| max slope | `25 deg` |
| max soft slope cost | `80` |
| obstacle relative height | `0.08-1.50 m` |

坡度是软代价，不会简单把所有坡面设为墙。未知区仍由 PGM 保持未知/不可通行语义。

### 10.3 Costmap 与 TEB

| 参数 | 当前值 | 说明 |
|---|---:|---|
| global inflation radius | `0.10 m` | 本次修改 |
| local inflation radius | `0.10 m` | 本次修改 |
| cost scaling factor | `5.0` | 全局/局部相同 |
| TEB inflation distance | `0.10 m` | 本次修改，软代价 |
| TEB min obstacle distance | `0.15 m` | 保留 Scout 原值 |
| footprint padding | `0.03 m` | 保留 Scout 原值 |
| local costmap | `6 x 6 m`、`0.05 m/cell` | odom 滚动窗口 |

### 10.4 Scout TEB 运动参数

| 参数 | 当前值 |
|---|---:|
| max forward speed | `0.35 m/s` |
| max backward speed | `0.00 m/s` |
| max angular speed | `1.00 rad/s` |
| linear acceleration | `0.50 m/s2` |
| angular acceleration | `2.50 rad/s2` |
| minimum turning radius | `0.0 m` |
| homotopy planning | `false` |

这些参数保持 Scout 既有配置，没有套用 WheelTech 的速度和加速度。

## 11. 代码位置

| 功能 | 文件 |
|---|---|
| 唯一建图入口 | `scout_system_bringup/launch/scout_mapping.launch` |
| 独立定位入口 | `scout_system_bringup/launch/scout_localization.launch` |
| TF 外参 | `scout_tf_manager/config/extrinsics.yaml` |
| 绝对雷达高度 | `scout_terrain_filter/config/patchworkpp_scout.yaml` |
| Patchwork++ 接入 | `scout_terrain_filter/launch/scout_terrain_filter.launch` |
| 分类点累积 | `scout_terrain_filter/src/terrain_map_accumulator_node.cpp` |
| 地图最终生成 | `scout_map_tools/scripts/finalize_map.py` |
| PGM 生成 | `scout_map_tools/src/pcd_to_pgm.cpp` |
| PGM 参数 | `scout_map_tools/config/scout_raw.yaml`、`scout_nav.yaml` |
| 高程坡度构建 | `scout_2p5d_navigation/src/terrain_map_builder_node.cpp` |
| 高程参数 | `scout_2p5d_navigation/config/terrain_builder.yaml` |
| 坡度 costmap 插件 | `scout_2p5d_navigation/src/terrain_costmap_layer.cpp` |
| 在线相对地面障碍 | `scout_2p5d_navigation/src/terrain_relative_obstacle_node.cpp` |
| 纯导航入口 | `scout_navigation/launch/navigation_teb.launch` |
| 全局/局部 costmap | `scout_navigation/config/global_costmap_slope.yaml`、`local_costmap_slope.yaml` |
| GlobalPlanner | `scout_navigation/config/global_planner_slope.yaml` |
| TEB | `scout_navigation/config/teb_local_planner.yaml` |
| footprint | `scout_navigation/config/costmap_common.yaml` |
| 导航日志 | `scout_navigation/launch/nav_logging.launch` |

## 12. 故障定位基准

| 现象 | 先检查 | 判断标准 |
|---|---|---|
| 地形分类全错 | `sensor_height`、`terrain_sensor` TF | 平地大多数近地回波进入 ground |
| 建图两条结果不一致 | `terrain_input_topic` | 必须为 `/scout/static_scan` |
| 人员残影明显 | 贝叶斯静态判定、障碍最少帧 | 人离开并复扫后应逐步清除 |
| PGM 障碍缺失 | nonground PCD、0.08 m 阈值 | 先确认目标高于 Scout 可跨越阈值 |
| 全屋不可通行 | `terrain_2p5d` confidence、PGM unknown | 坡度层不能把未知变自由，也不应覆盖全图为 lethal |
| 定位启动无 map TF | NDT 初值和 PCD | 收敛后才发布 `map -> odom` |
| 导航一开定位消失 | `roslaunch --nodes` | 导航入口不应包含 NDT/FAST-LIO/底盘 |
| 有路径不走 | `/cmd_vel`、`/scout/odom`、TEB、CAN | 逐段确认命令和反馈 |
| 局部 costmap 被占满 | 相对障碍点与 terrain map_name | 实时点必须和同一保存地面比较 |
| 狭窄通道仍拒绝 | footprint、padding、TEB 净空 | 10 cm 膨胀不会取消车体碰撞约束 |

## 13. 当前交付边界

- 本地 Scout 代码、组织 Git 仓库与 `192.168.50.120` 部署保持同版；
- 雷达中心离地 `0.48 m`、轮胎总高 `0.15 m`；
- 不把 WheelTech 串口驱动、footprint、外参或速度参数带入 Scout；
- 不上传地图、PCD、bag、日志或密钥；
- 实车验证前必须在平地先确认 Patchwork++ ground/nonground 和 TF，再进行坡道测试。
