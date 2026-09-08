# Scout Mini 自主导航机器人详细信息表 V5.1

> 本表对应2026-09-08源码和Scout V5.1部署基线。正式架构为可逆贝叶斯静态点、轨迹自由证据、离线鲁棒地面重建、PGM静态占据、保存坡度软代价、当前帧Terrain Guard、GlobalPlanner、TEB和安全逃逸恢复。

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
| 组织仓库车端克隆 | 以`git remote -v`与实机目录为准 |

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
| `base_link`离地高度 | `0.28 m` | 0.48 m减去重力对齐刚性Z偏移0.20 m |
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
| 建图 | `roslaunch scout_system_bringup scout_mapping.launch map_name:=NAME` | 是 | 唯一入口，默认保存贝叶斯静态PCD与车体轨迹 |
| 建图兼容别名 | `roslaunch scout_system_bringup scout_system.launch map_name:=NAME` | 是 | 只include上述V5.1入口，不再保留旧FAST-LIO直连链 |
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
     +-> 可逆3D贝叶斯静态点判定
     +-> /scout/static_scan
     +-> filtered_camera_init.pcd
/fastlio_odom -> traversed_path_map.pcd
```

默认建图不启动Patchwork++与分类累积。需要旧链路诊断时使用`enable_online_terrain_diagnostics:=true`，但最终交付仍以贝叶斯静态PCD为唯一静态权威。

## 6. 地图最终生成流

```text
filtered_camera_init.pcd
  -> raw_camera_init.pcd
  -> public_map.pcd
      + traversed_path_map.pcd
  -> PMF保守种子 + 鲁棒局部平面连续生长
  -> terrain_ground_map.pcd + terrain_obstacles_map.pcd
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
| `traversed_path_map.pcd` | `map/odom` | 0.30 m半宽轨迹自由证据 |
| `terrain_ground_candidates_map.pcd` | `map` | PMF与平面验证候选诊断 |
| `terrain_ground_map.pcd` | `map` | PGM 和高程构建 |
| `terrain_obstacles_map.pcd` | `map` | PGM 和高程障碍融合 |
| `map_raw.yaml` | 2D OccupancyGrid | 正式导航静态层 |
| `map.yaml` | 2D OccupancyGrid | 定位显示兼容 |
| `terrain_2p5d.yaml` | 高程地图索引 | 地形服务和全局坡度层 |
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
  -> Patchwork++当前帧ground/nonground
  -> Terrain Guard障碍/清除点
  -> local obstacle layer
GlobalPlanner -> TEB -> /cmd_vel
```

导航入口不包含重定位，因此不会销毁或替换已经运行的 `map -> odom`。

## 8. 关键节点

模式：M=建图，L=定位，N=导航，C=相机，O=离线工具。

| 节点 | 模式 | 主要输入 | 主要输出 |
|---|---|---|---|
| `/laserMapping` | M/L | Livox 点云与 IMU | `/Odometry`、注册点云、FAST-LIO TF；导航期间由L入口继续运行 |
| Scout base 节点 | M/L/N | CAN、`/cmd_vel` | `/scout/odom` |
| `/scout_pointcloud_mapper` | M | 注册点云、`/Odometry`、`/fastlio_odom` | 静态扫描、静态PCD、轨迹PCD |
| `/scout_terrain_cloud_adapter` | N | 实时body点云 | `/cloud_registered_terrain` |
| `/scout_navigation_patchworkpp` | N | 重力对齐当前帧 | ground/nonground |
| `/scout_navigation_terrain_guard` | N | 当前帧分类点 | obstacle/clearing与状态 |
| `/scout_terrain_map_accumulator` | M诊断 | ground/nonground、`/Odometry` | 旧分类PCD |
| `terrain_reclassify_node` | O | `public_map.pcd` | 候选、地面和障碍PCD |
| `terrain_map_builder_node` | O | map 坐标分类 PCD | `terrain_2p5d.*` |
| `/scout_geometry_tf_publisher` | M/L/N | 几何配置 | `odom -> camera_init` |
| `/scout_tf_manager` | M/L/N | `extrinsics.yaml` | 静态车体 TF |
| `/scout_pose_adapter` | M/L/N | TF | `/fastlio_odom` |
| `/scout_map_loader` | L/N | `public_map.pcd` | `/map_cloud` |
| Scout NDT localizer | L/N | 地图、实时点云、初值 | `map -> odom` |
| `/scout_navigation_map_server` | N | `map_raw.yaml` | `/nav_static_map` |
| `/scout_terrain_map_server` | N | `terrain_2p5d.yaml` | 高程、坡度、代价话题 |
| `/move_base` | N | 静态图、坡度、局部障碍、TF | 路径和 `/cmd_vel` |
| `StartEscapeRecovery` | N可选 | 全局/局部costmap、后向点云 | 已安装但默认关闭；实车验证后方可启用 |
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
| `/scout/static_map_cloud` | `sensor_msgs/PointCloud2` | mapper累计静态图，latched发布 |
| `/scout/dynamic_points` | `sensor_msgs/PointCloud2` | 可选动态候选调试，默认关闭 |
| `/cloud_registered_terrain` | `sensor_msgs/PointCloud2` | 重力对齐地形点云 |
| `/terrain/patchwork_ground` | `sensor_msgs/PointCloud2` | Patchwork++ 地面分类 |
| `/terrain/patchwork_nonground` | `sensor_msgs/PointCloud2` | Patchwork++ 非地面分类 |
| `/terrain/obstacle_points` | `sensor_msgs/PointCloud2` | 当前帧Terrain Guard marking点 |
| `/terrain/clearing_points` | `sensor_msgs/PointCloud2` | 当前帧raytrace clearing点 |
| `/terrain/status` | `diagnostic_msgs/DiagnosticArray` | 当前帧地形状态诊断 |
| `/nav_static_map` | `nav_msgs/OccupancyGrid` | 正式全局静态图 |
| `/terrain_2p5d/elevation_cloud` | `sensor_msgs/PointCloud2` | 保存高程点云可视化 |
| `/terrain_2p5d/slope` | `nav_msgs/OccupancyGrid` | 保存坡度可视化 |
| `/terrain_2p5d/traversability_cost` | `nav_msgs/OccupancyGrid` | 保存通行代价可视化 |
| `/map_cloud` | `sensor_msgs/PointCloud2` | NDT使用的静态PCD，latched发布 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz重定位初值 |
| `/map_2d` | `nav_msgs/OccupancyGrid` | 定位入口的2D地图显示 |
| `/scout/odom` | `nav_msgs/Odometry` | 底盘速度反馈 |
| `/scout_status` | `scout_msgs/ScoutStatus` | 底盘基础状态 |
| `/BMS_status` | `scout_msgs/ScoutBmsStatus` | BMS原始状态；不保证存在SOC百分比 |
| `/rs_status` | `scout_msgs/ScoutRsStatus` | 遥控/控制状态 |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘控制指令 |

mapper私有服务为`/scout_pointcloud_mapper/save_map`和`/scout_pointcloud_mapper/reset_map`；诊断累积器启用时还提供`/scout_terrain_map_accumulator/save_map`与`reset_map`。`move_base`标准 action、规划服务以及动态参数话题由ROS Navigation插件自动提供，现场应以`rostopic list -v`和`rosservice list`快照为准。

## 10. 地图与障碍参数

### 10.1 贝叶斯静态判定、轨迹与当前帧地形

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `sensor_height` | `0.48 m` | 雷达中心绝对离地实测 |
| Patchwork 最小/最大距离 | `0.25 / 12.0 m` | 单帧分割范围 |
| 贝叶斯/精细体素 | `0.20 / 0.05 m` | 动静态权威/输出几何 |
| 静态晋升 | `12次、2.0 s、命中率0.60` | 三条件同时满足 |
| 静态撤销 | `8次自由射线、0.75 s` | 整个generation失效 |
| hit/miss概率 | `0.65 / 0.30` | 占据与自由证据 |
| occupied/clearing阈值 | `0.75 / 0.35` | 晋升与清除概率 |
| 射线步长/距离 | `2 / 20 m` | 3D DDA，控制Jetson负载 |
| 轨迹采样/自由半宽 | `0.05 / 0.30 m` | base_link中心轨迹/PGM扫掠走廊 |
| 正式障碍相对高度 | `0.08-1.50 m` | 轮胎直径 0.15 m |
| guard 障碍下限 | `0.08 m` | 导航当前帧正式阈值 |
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
| fuse static obstacles | `false` |

坡度是软代价，不会简单把所有坡面设为墙。未知区仍由 PGM 保持未知/不可通行语义。

### 10.3 Costmap 与 TEB

| 参数 | 当前值 | 说明 |
|---|---:|---|
| global inflation radius | `0.10 m` | 保留Scout已验证基线 |
| local inflation radius | `0.10 m` | 保留Scout已验证基线 |
| cost scaling factor | `5.0` | 全局/局部相同 |
| TEB inflation distance | `0.10 m` | 保留Scout已验证基线，软代价 |
| TEB min obstacle distance | `0.15 m` | 保留 Scout 原值 |
| footprint padding | `0.03 m` | 保留 Scout 原值 |
| local costmap | `6 x 6 m`、`0.05 m/cell` | odom 滚动窗口 |
| local observation | marking=`/terrain/obstacle_points`；clearing=`/terrain/clearing_points` | persistence=0 |
| start escape | `0.05 m/s`、`0.30 m` | 默认关闭；实车验证后才允许启用 |
| rear coverage box | X=`-1.05~-0.55 m`、半宽`0.36 m` | terrain_sensor坐标；需20点、X跨度0.20 m、左右各5点、0.25 s内 |
| escape stall gate | `1.5 s / 0.02 m` | 倒车无进展时提前停止，不等待8 s总时限 |

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
| 离线地面重建 | `scout_map_tools/src/terrain_reclassify.cpp`、`config/terrain_reclassify.yaml` |
| 旧分类静态门 | `scout_map_tools/src/pcd_static_gate.cpp`，只供兼容模式 |
| PGM 生成 | `scout_map_tools/src/pcd_to_pgm.cpp` |
| PGM 参数 | `scout_map_tools/config/scout_raw.yaml`、`scout_nav.yaml` |
| 高程坡度构建 | `scout_2p5d_navigation/src/terrain_map_builder_node.cpp` |
| 高程参数 | `scout_2p5d_navigation/config/terrain_builder.yaml` |
| 坡度 costmap 插件 | `scout_2p5d_navigation/src/terrain_costmap_layer.cpp` |
| 当前帧地形障碍 | `scout_terrain_filter/src/terrain_guard_node.cpp` |
| 起点安全逃逸 | `scout_2p5d_navigation/src/start_escape_recovery.cpp` |
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
| 默认建图无Patchwork输出 | `enable_online_terrain_diagnostics` | false时正常，不影响最终地图 |
| 人员残影明显 | 12次/2秒/60%晋升、8次/0.75秒清除 | 人离开并复扫后整代逐步清除 |
| PGM 障碍缺失 | 离线重建PCD、0.08 m阈值 | 先确认目标高于Scout可跨越阈值 |
| 全屋不可通行 | `terrain_2p5d` confidence、PGM unknown | 坡度层不能把未知变自由，也不应覆盖全图为 lethal |
| 定位启动无 map TF | NDT 初值和 PCD | 收敛后才发布 `map -> odom` |
| 导航一开定位消失 | `roslaunch --nodes` | 导航入口不应包含 NDT/FAST-LIO/底盘 |
| 有路径不走 | `/cmd_vel`、`/scout/odom`、TEB、CAN | 逐段确认命令和反馈 |
| 局部costmap被占满 | 当前帧ground/nonground、TF、Terrain Guard | 不再检查保存terrain map_name |
| 逃逸恢复拒绝 | 后向点数与时间、局部走廊 | 无新鲜后向覆盖时拒绝是正确行为 |
| 狭窄通道仍拒绝 | footprint、padding、TEB 净空 | 10 cm 膨胀不会取消车体碰撞约束 |

## 13. 当前交付边界

- 本地 Scout 代码、组织 Git 仓库与 `192.168.50.120` 部署保持同版；
- 雷达中心离地 `0.48 m`、轮胎总高 `0.15 m`；
- base_link离地`0.28 m`；轨迹自由走廊半宽`0.30 m`；
- 不把 WheelTech 串口驱动、footprint、外参或速度参数带入 Scout；
- 不上传地图、PCD、bag、日志或密钥；
- 实车验证前必须在平地先确认 Patchwork++ ground/nonground 和 TF，再进行坡道测试。
