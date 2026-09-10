# Scout Mini 自主导航机器人详细信息表 V5.1 · BATF-Nav

> 技术名称：`BATF-NavBayesianAccumulationTerrainFusionNavigation`；简称 **BATF-Nav**；英文展开 **Bayesian Accumulation Terrain Fusion Navigation**。见[统一技术说明](../../docs/BATF-Nav_V5.1.md)。本次统一命名不改变V5.1版本、接口或车型参数。

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
| 正式一键建图 | `roslaunch scout_system_bringup scout_mapping.launch map_name:=NAME` | 是 | 与轮趣一致；停车Ctrl+C后等待PCD保存，随后单独finalize |
| 建图兼容别名 | `roslaunch scout_system_bringup scout_system.launch map_name:=NAME` | 是 | 只include上述V5.1入口，不再保留旧FAST-LIO直连链 |
| 地图最终生成 | `rosrun scout_map_tools finalize_map.py NAME` | 否 | 每次采集完成后执行；确认替换同名旧地图时才加`--replace-raw` |
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

2026-09-09起，新生成PGM预先膨胀`0.10 m`（原0.15 m）；move_base全局运行时膨胀仍为`0.10 m`，两者依次作用、不是覆盖关系，也不能直接相加作为车体净空。旧PGM需备份后重新finalize。局部实时障碍使用local costmap的`0.10 m`运行时膨胀。

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
| `/scout_localization_map_bundle_guard` | L | 地图目录、完成/容量标志 | 定位地图无效时以非零状态退出并联动关闭定位launch |
| `/scout_navigation_map_bundle_guard` | N | 地图目录、PGM与六层2.5D文件 | 导航地图无效时以非零状态退出并联动关闭导航launch |
| `/scout_planning_test_map_bundle_guard` | N测试 | 地图目录、PGM与六层2.5D文件 | 全局规划测试使用的同类安全门 |
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
| `/tf` | `tf2_msgs/TFMessage` | 动态TF：FAST-LIO与NDT |
| `/tf_static` | `tf2_msgs/TFMessage` | Scout几何、车体与相机静态TF |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO `camera_init/body` 位姿；导航日志同时录制，供姿态晃动对比 |
| `/fastlio_odom` | `nav_msgs/Odometry` | pose adapter输出的`odom/base_link`位姿与mapper轨迹输入 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | `camera_init` 注册点云 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | FAST-LIO 当前 body 点云 |
| `/cloud_registered_base` | `sensor_msgs/PointCloud2` | NDT 实时输入 |
| `/scout/static_scan` | `sensor_msgs/PointCloud2` | 贝叶斯静态扫描，建图公共输入 |
| `/scout/static_map_cloud` | `sensor_msgs/PointCloud2` | mapper累计静态图，latched发布 |
| `/scout/dynamic_points` | `sensor_msgs/PointCloud2` | 可选动态候选调试，默认关闭 |
| `/cloud_registered_terrain` | `sensor_msgs/PointCloud2` | 重力对齐地形点云 |
| `/terrain/patchwork_ground` | `sensor_msgs/PointCloud2` | Patchwork++ 地面分类 |
| `/terrain/patchwork_nonground` | `sensor_msgs/PointCloud2` | Patchwork++ 非地面分类 |
| `/terrain/patchwork_input` | `sensor_msgs/PointCloud2` | Patchwork++输入调试副本，是否发布由上游节点实现决定 |
| `/terrain/ground_points` | `sensor_msgs/PointCloud2` | Terrain Guard安全地面调试，默认不发布数据 |
| `/terrain/unsafe_ground_points` | `sensor_msgs/PointCloud2` | 不安全地面调试，默认不发布数据 |
| `/terrain/unknown_points` | `sensor_msgs/PointCloud2` | 未分类点调试，默认不发布数据 |
| `/terrain/obstacle_points` | `sensor_msgs/PointCloud2` | 当前帧Terrain Guard marking点 |
| `/terrain/clearing_points` | `sensor_msgs/PointCloud2` | 当前帧raytrace clearing点 |
| `/terrain/status` | `diagnostic_msgs/DiagnosticArray` | 当前帧地形状态诊断 |
| `/nav_static_map` | `nav_msgs/OccupancyGrid` | 正式全局静态图 |
| `/nav_static_map_metadata` | `nav_msgs/MapMetaData` | 正式全局静态图元数据 |
| `/terrain_2p5d/elevation_cloud` | `sensor_msgs/PointCloud2` | 保存高程点云可视化 |
| `/terrain_2p5d/slope` | `nav_msgs/OccupancyGrid` | 保存坡度可视化 |
| `/terrain_2p5d/traversability_cost` | `nav_msgs/OccupancyGrid` | 保存通行代价可视化 |
| `/map_cloud` | `sensor_msgs/PointCloud2` | NDT使用的静态PCD，latched发布 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz重定位初值 |
| `/map_2d` | `nav_msgs/OccupancyGrid` | 定位入口的2D地图显示 |
| `/map_metadata` | `nav_msgs/MapMetaData` | 定位入口2D地图元数据 |
| `/scout/odom` | `nav_msgs/Odometry` | 底盘速度反馈 |
| `/scout_status` | `scout_msgs/ScoutStatus` | 底盘基础状态 |
| `/BMS_status` | `scout_msgs/ScoutBmsStatus` | BMS原始状态；不保证存在SOC百分比 |
| `/rs_status` | `scout_msgs/ScoutRsStatus` | 遥控/控制状态 |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘控制指令 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz `2D Nav Goal`输入 |
| `/move_base/goal` | `move_base_msgs/MoveBaseActionGoal` | move_base action目标 |
| `/move_base/cancel` | `actionlib_msgs/GoalID` | move_base action取消 |
| `/move_base/status` | `actionlib_msgs/GoalStatusArray` | move_base action状态 |
| `/move_base/feedback` | `move_base_msgs/MoveBaseActionFeedback` | move_base action反馈 |
| `/move_base/result` | `move_base_msgs/MoveBaseActionResult` | move_base action结果 |
| `/move_base/GlobalPlanner/plan` | `nav_msgs/Path` | 正式全局路径 |
| `/move_base/TebLocalPlannerROS/global_plan` | `nav_msgs/Path` | TEB接收/变换后的全局路径 |
| `/move_base/TebLocalPlannerROS/local_plan` | `nav_msgs/Path` | TEB局部轨迹 |
| `/move_base/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | 全局融合代价地图 |
| `/move_base/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | 局部滚动代价地图 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | D435i彩色图像（相机独立启动时） |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | D435i彩色内参 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | D435i原始校正深度图 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | D435i深度内参 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 对齐到彩色的深度图 |
| `/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | 对齐深度对应内参 |

mapper私有服务为`/scout_pointcloud_mapper/save_map`和`/scout_pointcloud_mapper/reset_map`；诊断累积器启用时还提供`/scout_terrain_map_accumulator/save_map`与`reset_map`。`move_base`标准 action、规划服务以及动态参数话题由ROS Navigation插件自动提供，现场应以`rostopic list -v`和`rosservice list`快照为准。

## 10. 地图与障碍参数

### 10.1 贝叶斯静态判定、轨迹与当前帧地形

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `sensor_height` | `0.48 m` | 雷达中心绝对离地实测 |
| Patchwork 最小/最大水平半径 | `0.12 / 12.0 m` | 仅正前方对应车头边界；侧后方自反射需实测crop |
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
| guard无参考绝对下限 | `terrain_sensor z=-0.42 m` | 雷达离地0.48 m，约6 cm障碍保守兜底 |
| 离线网格硬限制 | `100 m / 2,000,000 cells` | 远端XY离群点触发明确失败，避免Jetson OOM |
| mapper容量硬限制 | `2,000,000 / 5,000,000 voxels` | 超限写`.capacity_limited`并拒绝正式finalize |
| 地图收尾事务标志 | `.finalization_incomplete` | finalize开始时原子写入，完整校验通过后才删除；定位/导航守卫持续拒绝该目录 |
| mapper恢复检查点 | `120 s` | 同步磁盘保存；正式结束在mapper正常退出时再次保存 |
| 全图调试发布 | `2 s（仅有订阅者时）` | 无RViz订阅时不遍历/序列化整张精细点云 |
| 失效精细体素清理 | `10 s` | 与全图发布解耦，候选generation过期后仍回收内存 |

### 10.2 2.5D 高程坡度

| 参数 | 当前值 |
|---|---:|
| resolution | `0.10 m` |
| ground percentile | `0.20` |
| fill radius | `4 cells` |
| fit radius | `0.30 m` |
| preferred slope | `5 deg` |
| max slope | `22 deg` |
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
| TEB inflation distance | `0.20 m` | 软代价范围，修正原0.10小于0.15净空的不合理配置 |
| TEB min obstacle distance | `0.15 m` | 保留 Scout 原值 |
| footprint padding | `0.03 m` | 保留 Scout 原值 |
| footprint envelope | `0.670 x 0.590 m` | base_link前0.370、后0.300、左右各0.295；配置值，需核对附件 |
| padded costmap envelope | `0.730 x 0.650 m` | 每侧增加0.03；TEB polygon仍使用未padding轮廓 |
| local costmap | `6 x 6 m`、`0.05 m/cell` | odom 滚动窗口 |
| local observation | marking=`/terrain/obstacle_points`；clearing=`/terrain/clearing_points` | persistence=0 |
| offline PGM inflation | `0.10 m` | 2026-09-09小幅降低；旧PGM需重新生成 |
| start escape | `0.05 m/s`、`0.30 m` | 默认关闭；实车验证后才允许启用 |
| rear coverage box | X=`-1.05~-0.55 m`、半宽`0.36 m` | 必须是terrain_sensor帧且header/回调均在0.25 s内；需20点、X跨度0.20 m、左右各5点 |
| escape stall gate | `1.5 s / 0.02 m` | 倒车无进展时提前停止，不等待8 s总时限 |

### 10.4 Scout TEB 运动参数

| 参数 | 当前值 |
|---|---:|
| max forward speed | `0.35 m/s` |
| max backward speed | `0.00 m/s` |
| max angular speed | `0.40 rad/s`（原1.00，约22.9度/秒） |
| linear acceleration | `0.50 m/s2` |
| angular acceleration | `0.30 rad/s2`（原2.50） |
| minimum turning radius | `0.0 m` |
| homotopy planning | `false` |

2026-09-09仅参考轮趣降低角速度和角加速度，Scout线速度、线加速度、轮廓保持不变。TEB是轨迹优化器，不是PID；角加速度为优化约束，不是独立硬件限幅，手动遥控不受TEB限速约束。车端检查时ROS未运行，以上为待实车验证的首轮调参，不是已确认的晃动根因修复。若仍晃动，按使用文档比较指令、底盘里程计、FAST-LIO与map到odom的TF，区分控制摆动和定位跳变。

## 11. 代码位置

| 功能 | 文件 |
|---|---|
| 唯一底层建图链 | `scout_system_bringup/launch/scout_mapping.launch` |
| 独立定位入口 | `scout_system_bringup/launch/scout_localization.launch` |
| TF 外参 | `scout_tf_manager/config/extrinsics.yaml` |
| 绝对雷达高度 | `scout_terrain_filter/config/patchworkpp_scout.yaml` |
| Patchwork++ 接入 | `scout_terrain_filter/launch/scout_terrain_filter.launch` |
| 分类点累积 | `scout_terrain_filter/src/terrain_map_accumulator_node.cpp` |
| 地图最终生成 | `scout_map_tools/scripts/finalize_map.py` |
| 地图交付/启动守卫 | `scout_system_bringup/scripts/map_bundle_guard.py` |
| 离线地面重建 | `scout_map_tools/src/terrain_reclassify.cpp`、`config/terrain_reclassify.yaml` |
| 旧分类静态门 | `scout_map_tools/src/pcd_static_gate.cpp`，只供兼容模式 |
| PGM 生成 | `scout_map_tools/src/pcd_to_pgm.cpp` |
| PGM 参数 | `scout_map_tools/config/scout_raw.yaml`、`scout_nav.yaml` |
| 高程坡度构建 | `scout_2p5d_navigation/src/terrain_map_builder_node.cpp` |
| 旧会话监督（兼容工具） | `scout_system_bringup/scripts/scout_mapping_session.py`，不再作为正式入口 |
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
| finalize拒绝capacity-limited | `.capacity_limited`及mapper日志 | 不得删标志交付；提高容量并重新建图 |
| 定位/导航launch立刻整体退出 | `.finalization_incomplete`、`.capacity_limited`、缺失/空文件及guard fatal日志 | 地图正在收尾或上次收尾失败；修复首个错误并重新运行finalize，不得手删标志绕过 |
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

## 旁路融合新增信息（2026-09-09）

| 入口/节点 | 职责 | TF |
|---|---|---|
| `scout_odom_fusion/fusion.launch` | 独立启动两节点；建图/定位入口默认include，参数enable_shadow_fusion | 不发布 |
| `/scout_shadow_ekf` | robot_localization EKF，20 Hz，3D位姿＋车体vx | publish_tf=false |
| `/scout_fusion_guard` | 原始输入校验、时序截取、协方差赋值、输出有效性与诊断 | 不发布 |

| 话题 | 类型 | 发布者 → 订阅者 | 帧/字段 |
|---|---|---|---|
| `/scout/odom` | nav_msgs/Odometry | scout_base_node → guard及原消费者 | 原样保留；仅取base_link的vx，累计pose归零不影响融合 |
| `/fastlio_odom` | nav_msgs/Odometry | scout_pose_adapter → guard及原消费者 | odom/base_link；仅使用XYZ/RPY，不使用零twist |
| `/Odometry` | nav_msgs/Odometry | laserMapping → 原消费者 | camera_init/body；保留，不直接输入EKF |
| `/scout/fusion/lio_input` | nav_msgs/Odometry | guard → EKF、日志 | odom/base_link；真实采样时间；显式位姿协方差 |
| `/scout/fusion/wheel_input` | nav_msgs/Odometry | guard → EKF、日志 | 仅车体vx有效；pose是未使用占位，不可用于画轨迹 |
| `/scout/fusion/ekf_raw` | nav_msgs/Odometry | EKF → guard | 内部预测输出，断流时仍可能存在，不供业务使用 |
| `/scout/fused_odom` | nav_msgs/Odometry | guard → 日志/人工比较 | odom/base_link，约20 Hz，不接NDT/导航 |
| `/scout/fusion/status` | diagnostic_msgs/DiagnosticArray | guard → 日志/监测 | 约1 Hz，WAITING/SHADOW_OK/ERROR及拒绝计数 |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | EKF及其他诊断节点 → 监测工具 | EKF自身诊断；不替代guard输出有效性状态 |

新融合没有新增TF边，原TF树和唯一发布者不变。轮速和融合的位置原点不同，虽然可能同标odom也不能直接相减；只对齐同时间窗下base_link的相对运动。前X、左Y、上Z，正yaw为逆时针，倒车速度为负。

| 故障 | 检查 | 处理 |
|---|---|---|
| 无融合输出、WAITING | 原话题、header帧、时间戳、EKF是否首先收到LIO位姿 | 等待两路有效输入；检查robot_localization依赖 |
| LIO frame错误 | 是否误接原始/Odometry | 恢复/fastlio_odom；禁止直接修改frame标签 |
| 断流/时钟回跳/LIO重置ERROR | status原因、源频率、消息年龄 | 修复源后重启整个fusion.launch；旧导航不受guard停止影响 |
| 数据仍在但不是fused_odom | 是否订阅内部ekf_raw | 只用受保护的/scout/fused_odom进行比较 |
| 初始化位置不是000 | 与/fastlio_odom的绝对位姿比较 | 正常，保留LIO odom原点；轮速只融合vx |
| 节点重复启动 | 建图/定位入口已有默认include | 不额外启动独立fusion.launch；需要禁用用enable_shadow_fusion:=false |

## 无NDT实车测试入口与结果

| 项目 | 值 |
|---|---|
| 推荐命令 | `rosrun scout_system_bringup scout_fusion_test.py --profile wheel_priority`；省略profile也默认新版 |
| 原版对照 | `rosrun scout_system_bringup scout_fusion_test.py --profile baseline` |
| 只检查不启动 | 同命令添加`--check-only`；存在旧节点返回2 |
| 内部launch | `scout_system_bringup/scout_fusion_test.launch`；不要绕过启动冲突检查 |
| 新节点 | `/scout_odom_comparison`，只订阅不发布ROS话题或TF |
| 输入 | `/scout/odom`、`/fastlio_odom`、`/scout/fused_odom`，nav_msgs/Odometry |
| 对齐 | 同一header时刻插值，各自共同起点SE(3)归一化，保留前左上 |
| 输出频率 | 终端1 Hz、CSV约10 Hz，无新增TF |
| 数据目录 | `~/livox_fastlio/logs/fusion_tests/<时间_随机后缀>/` |
| 文件 | samples.csv、summary.json、comparison.png、configuration.json（实际guard/EKF参数快照）；不默认录bag |
| 对比指标 | 相对XYZ/yaw、净水平位移、累计水平路程、相对轮速XY/yaw差、最大/RMS参考差 |
| 图形 | 三路XY、三路相对航向、LIO/融合对轮速位置差；最多最近20000点 |
| READY | 三路获得共同有效时间点，初始化起点，之后可人工运动 |
| INVALID | 断流、帧错或明显源重置，停止更新比较，不自动发停车指令 |

测试链TF仍为odom→camera_init→body→base_link及原静态传感器边，没有map→odom发布者。无NDT、map_server、Navigation或地图文件依赖。脚本不控制CAN接口上线，需保留原CAN配置；底盘驱动仍只输出轮速而不发布odom→base_link。

### 轮速优先配置差异（2026-09-10）

| 参数/接口 | baseline | wheel_priority |
|---|---|---|
| guard覆盖文件 | 无，guard.yaml | config/guard_wheel_priority.yaml |
| 运行LIO XYZ方差 m² | 0.01 / 0.01 / 0.04 | 10000 / 10000 / 10000 |
| LIO RPY方差 rad² | 各0.0025 | 各0.0025 |
| 初始化XYZ方差 m² | 0.01 / 0.01 / 0.04 | 0.01 / 0.01 / 0.04，持续到收到EKF初始化输出 |
| 轮速前向方差 (m/s)² | 0.0025 | 0.0004 |
| 转弯乘数 | min(25,1+(omega/0.30)²) | 相同；只影响轮速vx权重 |
| 独立fusion.launch参数 | profile:=baseline，默认 | profile:=wheel_priority |
| 测试launch参数 | fusion_profile:=baseline | fusion_profile:=wheel_priority，默认 |
| 正式建图/定位入口 | 默认baseline | 本次不切换正式入口 |
| 诊断新增字段 | profile=baseline | profile=wheel_priority |

没有新增话题、消息类型或TF边；完整输入输出仍见上方旁路融合表。前向是当前base_link的X轴，世界速度由融合姿态旋转，和启动方向、走廊朝向无关。位姿header.frame_id=odom，twist属于child_frame_id=base_link；不能按header把twist当成世界速度。

| 现象 | 检查 | 处理/边界 |
|---|---|---|
| 不确定启动的是新版还是原版 | `/scout_fusion_guard/profile`及configuration.json | 按profile区分，不能只看文件名或启动日期 |
| 前进主要改变odom的Y | 当前yaw、输出child_frame_id | yaw约90°时正常，不应强制改为世界X |
| 新版仍存在位置误差 | 航向差、打滑、LIO残余位置拉偏、实测端点 | 该配置只增加轮速平移贡献，不修正姿态漂移或保证绝对精度 |
| LIO断流后无融合数据 | `/scout/fusion/status` | 仍为锁止保护；本版无轮速独立续航功能 |
| 初始化后位置不是000 | 首个有效LIO位姿与EKF输出 | 保留LIO原点；启动时保持静止，等待READY |
