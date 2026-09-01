# Scout Mini 启动文件、节点、话题、TF与常见问题完整表 V3.1

> 本表按当前源码逐项整理。车端工作空间：`~/livox_fastlio`。
> 正式建图入口：`scout_mapping.launch`；正式定位入口：`scout_localization.launch`；正式导航入口：`navigation_teb.launch`。
> 导航当前工作正常，本表只记录现状，不要求修改navigation参数。
> 审计范围：Scout正式链路、项目自研辅助包、仓库内会影响Scout运行的上游/示例Launch；通用ROS基础话题（如每个节点都有的`/rosout`）单独说明，不冒充业务接口。
> V3.1：完成源码反向审计，补齐上游/示例Launch边界、条件性话题、NDT重映射、Action/服务、动态参数、完整配置值和实机复核流程。

## 1. 工作模式与禁止同时启动项

| 模式 | 必需启动 | 主要输出 | 禁止同时启动 |
|---|---|---|---|
| 底盘/雷达基础检查 | `scout_livox_base.launch` | Livox数据、Scout状态和轮速里程计 | 其他会重复启动雷达或底盘的总launch |
| 建图 | `scout_mapping.launch` | FAST-LIO位姿、贝叶斯静态点云、过滤PCD | `scout_localization.launch`、旧`scout_system.launch` |
| 重定位 | `scout_localization.launch` | `map → odom`、地图点云、`/map_2d` | 建图总launch、其他`map → odom`发布者 |
| 导航 | 先定位，再`navigation_teb.launch` | costmap、路径、`/cmd_vel` | 第二个move_base、第二个底盘驱动 |
| 全局规划安全测试 | 定位后`global_planning_test.launch` | make_plan结果，不向底盘发速度 | 正式navigation launch |
| 相机 | `D435I.launch` | RGB、对齐深度、camera_info、TF | 第二个RealSense驱动实例 |

## 2. Launch文件完整清单

### 2.1 系统级Launch

| 文件 | 参数 | 实际启动内容 | 使用状态 |
|---|---|---|---|
| `scout_system_bringup/launch/scout_mapping.launch` | `map_name`，默认`current_mapping` | Mid-360、FAST-LIO、mapper、TF manager、pose adapter、Scout底盘 | 正式建图唯一入口 |
| `scout_system_bringup/launch/scout_localization.launch` | `map_name`、`map_dir`、`map_pcd`、`map_yaml` | Mid-360、FAST-LIO local odom、TF、pose/cloud adapter、map loader、NDT、map_server、底盘 | 正式重定位入口 |
| `scout_system_bringup/launch/scout_relocalization.launch` | `map_pcd` | `map_loader`和NDT localizer | 被localization包含，不单独启动整机 |
| `scout_system_bringup/launch/fastlio_mapping_scout.launch` | `rviz=false` | 参数加载、`laserMapping`，FAST-LIO PCD保存关闭 | 被mapping包含 |
| `scout_system_bringup/launch/fastlio_local_odom.launch` | `rviz=false` | 与建图相同的FAST-LIO里程计输出，PCD保存关闭 | 被localization包含 |
| `scout_system_bringup/launch/scout_livox_base.launch` | 无 | Mid-360和底盘，不启动FAST-LIO | 硬件基础检查 |
| `scout_system_bringup/launch/D435I.launch` | 由rs_camera提供 | D435i彩色、深度、对齐、内部TF及安装TF | 相机独立启动 |
| `scout_system_bringup/launch/scout_system.launch` | 无 | 上游`mapping_mid360.launch`、TF、底盘 | 旧入口；没有新mapper，正式建图禁用 |

### 2.2 功能包Launch

| 文件 | 参数/默认值 | 节点或行为 |
|---|---|---|
| `scout_pointcloud_mapper/launch/pointcloud_mapper.launch` | `map_name=current_mapping`、`input_cloud=/cloud_registered`、`input_odom=/Odometry`、`output_path=~/livox_fastlio/maps/<name>/filtered_camera_init.pcd` | `scout_pointcloud_mapper`，required=true |
| `scout_tf_manager/launch/tf_manager.launch` | 从两个YAML加载 | `scout_tf_manager`发布`body → base_link`；`scout_geometry_tf_publisher`发布`odom → camera_init` |
| `scout_pose_adapter/launch/pose_adapter.launch` | parent=`odom`、child=`base_link`、topic=`/fastlio_odom`、20 Hz | TF转Odometry |
| `scout_cloud_adapter/launch/cloud_adapter.launch` | input=`/cloud_registered_body`、output=`/cloud_registered_base`、target=`base_link` | 点云frame转换 |
| `scout_navigation/launch/navigation.launch` | `map_name`、`odom_topic=/scout/odom`、`cmd_vel_topic=/cmd_vel`、`obstacle_cloud_topic=/cloud_registered_body` | `map_raw.yaml` + move_base + GlobalPlanner + DWA |
| `scout_navigation/launch/navigation_teb.launch` | 同上 | `map_raw.yaml` + move_base + GlobalPlanner + TEB；当前正式导航入口 |
| `scout_navigation/launch/global_planning_test.launch` | `map_name` | 关闭local obstacle layer，cmd_vel重映射到blocked话题，RViz clicked point请求make_plan |
| `scout_navigation/launch/nav_logging.launch` | `tag=nav_test` | 启动导航rosbag记录脚本 |
| `scout_bringup/launch/scout_mini_robot_base.launch` | `port_name=can0`、`simulated_robot=false`、`odom_topic_name`、`pub_tf=false` | `scout_base_node`；正式系统始终覆盖odom话题为`/scout/odom` |

### 2.3 仓库内上游、示例和显示Launch

下表不是推荐启动顺序，而是防止看到仓库里还有Launch却误以为漏写。正式系统只使用前两节标明的入口。

| 文件 | 内容 | 与正式系统的关系 |
|---|---|---|
| `scout_base/launch/scout_base.launch` | 通用Scout底盘节点，支持普通/mini/omni参数 | 被上层bringup包含；不要再单独启动第二个底盘节点 |
| `scout_base/launch/scout_mini_base.launch` | Mini底盘节点 | 上游直接入口；正式系统已通过bringup启动 |
| `scout_base/launch/scout_mini_omni.launch` | Mini Omni底盘节点 | 本车非Omni时不用 |
| `scout_base/launch/display_model.launch` | URDF/robot_state_publisher/RViz显示链 | 仅模型显示，不属于实车导航启动链 |
| `scout_bringup/launch/scout_robot_base.launch` | 普通Scout底盘包装 | 非本项目正式Mini入口 |
| `scout_bringup/launch/scout_miniomni_robot_base.launch` | Mini Omni包装 | 非本车入口 |
| `scout_bringup/launch/scout_simulated_robot.launch` | 模拟底盘，默认`pub_tf=true` | 实车禁止混用，会形成重复TF/里程计 |
| `scout_bringup/launch/scout_teleop_keyboard.launch` | 键盘发布`/cmd_vel` | 仅人工遥控；导航时会与move_base争用控制话题 |
| `fast_lio_localization/launch/map_loader.launch` | 加载包内示例PCD并发布地图云 | 正式系统用`scout_relocalization.launch`，不直接用它 |
| `fast_lio_localization/launch/fast_lio_localization.launch` | 示例地图、示例NDT、RViz及`body → velodyne`静态TF | 禁止用于正式系统；参数、frame和节点名均与本项目不一致 |
| `FAST_LIO/launch/mapping_mid360.launch` | FAST-LIO上游Mid-360入口 | 旧`scout_system.launch`使用；不会启动新mapper |
| `livox_ros_driver2/launch_ROS1/msg_MID360.launch` | Mid-360驱动 | 已被系统总Launch包含，禁止重复启动 |
| `scout_description/launch/*.launch` | URDF、Gazebo或RViz模型显示 | 调试/仿真用途，不是实车功能链 |

### 2.4 正式Launch的节点展开

#### 建图

```text
/livox_lidar_publisher2
/laserMapping
/scout_pointcloud_mapper
/scout_tf_manager
/scout_geometry_tf_publisher
/scout_pose_adapter
/scout_base_node
```

检查：

```bash
roslaunch --nodes scout_system_bringup scout_mapping.launch map_name:=check_map
```

#### 重定位

```text
/livox_lidar_publisher2
/laserMapping
/scout_tf_manager
/scout_geometry_tf_publisher
/scout_pose_adapter
/scout_cloud_adapter
/scout_map_loader
/scout_global_localizer
/scout_map_server
/scout_base_node
```

#### 导航TEB

```text
/scout_navigation_map_server
/move_base
```

定位launch必须继续运行，导航launch不重复启动雷达、FAST-LIO、TF、NDT或底盘。

#### 导航DWA、全局规划测试和日志

| Launch | 固定节点 | 条件/说明 |
|---|---|---|
| `navigation.launch` | `/scout_navigation_map_server`、`/move_base` | DWA兼容入口；不能和TEB入口同时启动 |
| `global_planning_test.launch` | `/scout_navigation_map_server`、`/move_base`、`/scout_global_plan_tester` | move_base速度被重映射到`/scout_navigation/cmd_vel_blocked` |
| `nav_logging.launch` | `/nav_log_session`，内部再启动`rosbag record` | 只记录与分析，不发布控制命令、不改参数 |
| FAST-LIO包装Launch且`rviz:=true` | `/fastlio_rviz` | 默认false，因此正式启动通常没有该节点 |

D435i驱动内部节点、nodelet和图像传输插件名称随`realsense2_camera`版本变化，不把某一版本的内部名字写成固定接口。实机精确展开使用：

```bash
roslaunch --nodes scout_system_bringup D435I.launch
```

## 3. 节点职责与输入输出

| 节点 | 包 | 订阅/输入 | 发布/输出 | 是否发TF |
|---|---|---|---|---|
| `livox_lidar_publisher2` | `livox_ros_driver2` | Mid-360 UDP | `/livox/lidar`、`/livox/imu` | 否 |
| `laserMapping` | `fast_lio` | Livox点云、IMU | 注册点云、`/Odometry`、FAST-LIO TF | `camera_init → body` |
| `scout_pointcloud_mapper` | `scout_pointcloud_mapper` | `/cloud_registered`、`/Odometry` | 静态扫描、静态地图云、PCD | 否 |
| `scout_tf_manager` | `scout_tf_manager` | `extrinsics.yaml` | `/tf_static` | `body → base_link` |
| `scout_geometry_tf_publisher` | `scout_tf_manager` | `scout_geometry.yaml` | `/tf_static` | `odom → camera_init` |
| `scout_pose_adapter` | `scout_pose_adapter` | TF `odom → base_link` | `/fastlio_odom` | 否 |
| `scout_cloud_adapter` | `scout_cloud_adapter` | body点云和TF | `/cloud_registered_base` | 否 |
| `scout_map_loader` | `fast_lio_localization` | `public_map.pcd` | `/map_cloud` | 否 |
| `scout_global_localizer` | `fast_lio_localization` | 地图、实时base点云、FAST-LIO位姿、初始位姿 | NDT结果 | `map → odom` |
| `scout_map_server` | `map_server` | `map.yaml` | `/map_2d`、地图元数据 | 否 |
| `scout_navigation_map_server` | `map_server` | `map_raw.yaml` | `/nav_static_map`、元数据 | 否 |
| `move_base` | `move_base` | 地图、TF、轮速里程计、实时障碍、目标 | 全局/局部路径、速度 | 否 |
| `scout_base_node` | `scout_base` | CAN、`/cmd_vel`、灯光命令 | `/scout/odom`和状态 | 正式配置为否 |
| `scout_global_plan_tester` | `scout_navigation` | `/clicked_point`、TF、make_plan服务 | `/scout_global_plan_test` | 否 |
| `nav_log_session` | `scout_navigation` | 当前ROS图、参数和指定话题 | bag、CSV、摘要、配置快照 | 否 |
| `realsense2_camera` | `realsense2_camera` | D435i USB | RGB、深度、camera_info、相机TF | 相机内部TF |
| `base_to_d435i` | `tf2_ros` | launch参数 | `/tf_static` | `base_link → camera_link` |
| `fastlio_rviz` | `rviz` | FAST-LIO话题和TF | 可视化窗口 | 否；仅`rviz:=true` |
| `teleop_keybord` | `teleop_twist_keyboard` | 键盘 | `/cmd_vel` | 否；只用于人工遥控 |

## 4. 话题完整表

### 4.1 Livox与FAST-LIO

| 话题 | 类型 | 发布者 | 主要订阅者 | header frame | 阶段/说明 |
|---|---|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Livox驱动 | FAST-LIO | `livox_frame`或驱动配置值 | 原始雷达数据 |
| `/livox/imu` | `sensor_msgs/Imu` | Livox驱动 | FAST-LIO | Livox IMU frame | IMU初始化必需 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO | mapper、RViz | `camera_init` | 世界系当前注册扫描，不是完整累计图 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | FAST-LIO | cloud adapter、local costmap | `body` | 车体系当前扫描 |
| `/cloud_registered_base` | `sensor_msgs/PointCloud2` | cloud adapter | NDT localizer | `base_link` | 重定位匹配输入 |
| `/cloud_effected` | `sensor_msgs/PointCloud2` | FAST-LIO | 调试 | `camera_init` | 有效匹配特征 |
| `/Laser_map` | `sensor_msgs/PointCloud2` | FAST-LIO | 调试/RViz | `camera_init` | FAST-LIO内部局部地图输出 |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO | mapper | frame=`camera_init`，child=`body` | mapper时间同步和传感器位置 |
| `/path` | `nav_msgs/Path` | FAST-LIO | RViz | `camera_init` | 当前`path_en=false`，通常无数据 |

FAST-LIO源码还创建上述发布器；参数关闭意味着发布器可能能在`rostopic list`中看到，但没有有效消息。判断功能是否工作必须用`rostopic hz`，不能只看名字是否存在。

### 4.2 点云预处理与地图累积

| 话题/服务 | 类型 | 发布/服务节点 | 消费者 | frame/行为 |
|---|---|---|---|---|
| `/scout/static_scan` | `sensor_msgs/PointCloud2` | mapper | RViz/诊断 | 与`/cloud_registered`相同，通常`camera_init`；仅当前满足静态条件的点 |
| `/scout/static_map_cloud` | `sensor_msgs/PointCloud2` | mapper | RViz/诊断 | `camera_init`；latched累计有效细地图 |
| `/scout/dynamic_points` | `sensor_msgs/PointCloud2` | mapper | 调试 | 默认不发布，`publish_dynamic_points=false` |
| `/scout_pointcloud_mapper/save_map` | `std_srvs/Trigger` | mapper | 人工诊断 | 立即保存当前有效PCD；正常流程不用 |
| `/scout_pointcloud_mapper/reset_map` | `std_srvs/Empty` | mapper | 人工诊断 | 清空占据与细地图，谨慎调用 |

mapper日志字段：

| 字段 | 含义 |
|---|---|
| `input` | FAST-LIO输入点数 |
| `filtered` | 距离、体素和离群处理后点数 |
| `static_scan` | 当前扫描中已满足静态条件的点数 |
| `map` | 内存中的0.05 m细体素数，包含尚未输出的当前generation候选 |
| `occupancy` | 0.20 m贝叶斯占据状态数量 |
| `bayes_cleared` | 因空闲概率达到阈值而清除的粗体素累计数 |

### 4.3 地图和重定位

| 话题 | 类型 | 发布者 | 订阅者 | frame/说明 |
|---|---|---|---|---|
| `/map_cloud` | `sensor_msgs/PointCloud2` | `scout_map_loader` | NDT、RViz | `map`，latched；来自`public_map.pcd` |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz 2D Pose Estimate | NDT | 应为`map` frame |
| `/fastlio_odom` | `nav_msgs/Odometry` | pose adapter | NDT | frame=`odom`，child=`base_link`；twist全0 |
| `/map_2d` | `nav_msgs/OccupancyGrid` | localization map_server | RViz/其他参考消费者 | `map`；来自`map.yaml` |
| `/map_metadata` | `nav_msgs/MapMetaData` | localization map_server | 地图消费者 | localization launch只重映射`map`，因此元数据保持此名 |
| `/tf` | `tf2_msgs/TFMessage` | FAST-LIO、NDT | 全系统 | 动态TF |
| `/tf_static` | `tf2_msgs/TFMessage` | TF manager、D435i等 | 全系统 | 静态TF |

NDT节点当前没有单独的“定位结果Pose”话题，最终结果通过TF `map → odom`生效。

NDT源码接口名与正式Launch重映射关系：

| NDT源码订阅名 | 正式运行名 | 类型 | 同步方式 |
|---|---|---|---|
| `/map_cloud` | `/map_cloud` | `sensor_msgs/PointCloud2` | 独立地图回调，latched地图 |
| `/velodyne_points` | `/cloud_registered_base` | `sensor_msgs/PointCloud2` | 与里程计ApproximateTime同步 |
| `/odom_lio` | `/fastlio_odom` | `nav_msgs/Odometry` | 与点云ApproximateTime同步 |
| `/initialpose` | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 独立初始位姿回调 |

点云和里程计的message filter同步队列为10；两路底层订阅队列为1。若其中一路低频、时间戳差距过大或frame转换失败，NDT不会进入一次完整匹配。

### 4.4 Scout底盘

| 话题 | 类型 | 发布/订阅 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | move_base发布，`scout_base_node`订阅 | `linear.x`和`angular.z`控制底盘 |
| `/scout/odom` | `nav_msgs/Odometry` | `scout_base_node`发布 | frame=`odom`，child=`base_link`；正式系统速度反馈源 |
| `/scout_status` | `scout_msgs/ScoutStatus` | 底盘发布 | 车辆状态、电压等协议字段 |
| `/BMS_status` | `scout_msgs/ScoutBmsStatus` | 底盘发布 | BMS消息；不代表当前CAN一定有可靠SOC |
| `/rs_status` | `scout_msgs/ScoutRsStatus` | 底盘发布 | 遥控/状态相关消息 |
| `/scout_light_control` | `scout_msgs/ScoutLightCmd` | 用户发布，底盘订阅 | 灯光控制 |

正式launch中`pub_tf=false`，所以`/scout/odom`有消息但底盘不广播`odom → base_link`。

### 4.5 Navigation与move_base

| 话题 | 类型 | 发布者/订阅者 | 说明 |
|---|---|---|---|
| `/nav_static_map` | `nav_msgs/OccupancyGrid` | navigation map_server发布，global static layer订阅 | 来自`map_raw.yaml`，固化膨胀为0 |
| `/nav_static_map_metadata` | `nav_msgs/MapMetaData` | navigation map_server发布 | navigation地图元数据 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz发布，move_base订阅 | 2D Nav Goal入口 |
| `/move_base/goal` | `move_base_msgs/MoveBaseActionGoal` | action客户端→move_base | action目标 |
| `/move_base/cancel` | `actionlib_msgs/GoalID` | action客户端→move_base | 取消一个或全部目标 |
| `/move_base/status` | `actionlib_msgs/GoalStatusArray` | move_base | action状态 |
| `/move_base/result` | `move_base_msgs/MoveBaseActionResult` | move_base | action结果 |
| `/move_base/feedback` | `move_base_msgs/MoveBaseActionFeedback` | move_base | action反馈 |
| `/move_base/current_goal` | `geometry_msgs/PoseStamped` | move_base | 当前执行目标 |
| `/move_base/recovery_status` | `move_base_msgs/RecoveryStatus` | move_base | recovery状态；当前外部recovery关闭，通常不活跃 |
| `/move_base/GlobalPlanner/plan` | `nav_msgs/Path` | GlobalPlanner | 全局路径 |
| `/move_base/TebLocalPlannerROS/global_plan` | `nav_msgs/Path` | TEB | 交给TEB的局部截取全局路径 |
| `/move_base/TebLocalPlannerROS/local_plan` | `nav_msgs/Path` | TEB | 局部轨迹 |
| `/move_base/TebLocalPlannerROS/teb_poses` | `geometry_msgs/PoseArray` | TEB | 优化轨迹姿态序列 |
| `/move_base/TebLocalPlannerROS/teb_markers` | `visualization_msgs/Marker` | TEB | TEB可视化标记 |
| `/move_base/TebLocalPlannerROS/teb_feedback` | `teb_local_planner/FeedbackMsg` | TEB | `publish_feedback=true`时发布轨迹反馈 |
| `/move_base/TebLocalPlannerROS/obstacles` | `costmap_converter/ObstacleArrayMsg` | 外部节点→TEB | 可选自定义障碍输入；当前无发布者 |
| `/move_base/TebLocalPlannerROS/via_points` | `nav_msgs/Path` | 外部节点→TEB | 可选途经点；当前无发布者且权重为0 |
| `/move_base/TebLocalPlannerROS/parameter_updates` | `dynamic_reconfigure/Config` | TEB | 动态参数变更回显 |
| `/move_base/DWAPlannerROS/global_plan` | `nav_msgs/Path` | DWA | 仅DWA入口 |
| `/move_base/DWAPlannerROS/local_plan` | `nav_msgs/Path` | DWA | 仅DWA入口 |
| `/move_base/DWAPlannerROS/trajectory_cloud` | `sensor_msgs/PointCloud2` | DWA | `publish_traj_pc=true`时的候选轨迹 |
| `/move_base/DWAPlannerROS/cost_cloud` | `sensor_msgs/PointCloud2` | DWA | `publish_cost_grid_pc=true`时的评分栅格 |
| `/move_base/DWAPlannerROS/parameter_updates` | `dynamic_reconfigure/Config` | DWA | 仅DWA入口 |
| `/move_base/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | global costmap | 全局代价地图 |
| `/move_base/global_costmap/costmap_updates` | `map_msgs/OccupancyGridUpdate` | global costmap | 增量更新 |
| `/move_base/global_costmap/footprint` | `geometry_msgs/PolygonStamped` | global costmap | 当前带padding footprint |
| `/move_base/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | local costmap | 局部滚动代价地图 |
| `/move_base/local_costmap/costmap_updates` | `map_msgs/OccupancyGridUpdate` | local costmap | 增量更新 |
| `/move_base/local_costmap/footprint` | `geometry_msgs/PolygonStamped` | local costmap | 当前带padding footprint |
| `/move_base/parameter_updates` | `dynamic_reconfigure/Config` | move_base | move_base动态参数回显 |
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz Publish Point | global plan tester输入 |
| `/scout_global_plan_test` | `nav_msgs/Path` | global plan tester | 安全全局规划测试结果 |
| `/scout_navigation/cmd_vel_blocked` | `geometry_msgs/Twist` | global planning test中的move_base | 安全锁话题，不接底盘 |

move_base常用服务：

| 服务 | 类型 | 用途 |
|---|---|---|
| `/move_base/make_plan` | `nav_msgs/GetPlan` | 给定起终点请求全局路径 |
| `/move_base/clear_costmaps` | `std_srvs/Empty` | 清理可清除costmap层；车辆安全确认后使用 |

### 4.6 服务、Action和动态参数接口

| 接口 | 类型 | 提供者 | 注意事项 |
|---|---|---|---|
| `/move_base` | `move_base_msgs/MoveBaseAction` | move_base | 一个Action由`goal/cancel/status/feedback/result`五个话题组成 |
| `/move_base/make_plan` | `nav_msgs/GetPlan` | move_base | 只规划，不执行 |
| `/move_base/clear_costmaps` | `std_srvs/Empty` | move_base | 会清理可清除层，不会改静态PGM |
| `/scout_pointcloud_mapper/save_map` | `std_srvs/Trigger` | mapper | 立即覆盖当前输出PCD |
| `/scout_pointcloud_mapper/reset_map` | `std_srvs/Empty` | mapper | 清空内存地图，不可撤销；建图中慎用 |
| `/static_map` | `nav_msgs/GetMap` | ROS map_server | 定位和导航各启动一个map_server时服务同名，调用前必须`rosservice info /static_map`确认当前提供者；区分地图应使用`/map_2d`和`/nav_static_map`话题 |
| `*/set_parameters` | `dynamic_reconfigure/Reconfigure` | move_base、planner、costmap及插件 | 实际名称随已加载插件生成；当前导航正常，不通过它临时改参 |

动态参数常见实际路径包括`/move_base/set_parameters`、`/move_base/TebLocalPlannerROS/set_parameters`、`/move_base/{global_costmap,local_costmap}/set_parameters`以及各层的`set_parameters`。精确清单用：

```bash
rosservice list | grep -E 'move_base|static_map|scout_pointcloud_mapper' | sort
rostopic list | grep -E 'parameter_(updates|descriptions)' | sort
```

每个roscpp/rospy节点普遍还带`get_loggers`和`set_logger_level`，每个节点也会使用`/rosout`；这些是ROS通用管理接口，不逐行重复进业务话题表。

### 4.7 D435i

当前launch参数：color=true、depth=true、align_depth=true、accel/gyro=false、pointcloud=false、publish_tf=true。

典型RealSense ROS1话题如下；具体命名受驱动版本影响，最终以`rostopic list | sort`为准。

| 话题 | 类型 | frame/用途 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色原图，通常`camera_color_optical_frame` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色内参 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 原深度图 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度内参 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 对齐到彩色的深度 |
| `/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | 对齐深度内参，驱动版本可能复用color info |
| `/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | 仅在image_transport插件启用时存在 |
| `/camera/color/image_raw/compressedDepth` | `sensor_msgs/CompressedImage` | 仅对应插件启用时存在，通常不需要 |
| `/camera/depth/image_rect_raw/compressedDepth` | `sensor_msgs/CompressedImage` | 深度压缩传输插件启用时存在 |
| `/camera/aligned_depth_to_color/image_raw/compressedDepth` | `sensor_msgs/CompressedImage` | 对齐深度压缩传输插件启用时存在 |

当前不应出现D435i IMU和PointCloud2流；若出现，检查launch参数是否被其他入口覆盖。

## 5. TF树与唯一发布者

### 5.1 建图TF树

```text
odom
└─ camera_init                     static
   └─ body                         dynamic
      └─ base_link                 static
         └─ camera_link            static（仅D435I.launch启动时）
            ├─ camera_color_frame  RealSense static
            │  └─ camera_color_optical_frame
            └─ camera_depth_frame  RealSense static
               └─ camera_depth_optical_frame
```

### 5.2 定位/导航TF树

```text
map
└─ odom                            dynamic
   └─ camera_init                  static
      └─ body                      dynamic
         └─ base_link              static
            └─ camera_link ...
```

### 5.3 TF责任表

| TF边 | 类型 | 唯一发布者 | 参数/源码 |
|---|---|---|---|
| `map → odom` | dynamic | `scout_global_localizer` | `fast_lio_localization.cpp`，`tf_postdate_sec=0.50` |
| `odom → camera_init` | static | `scout_geometry_tf_publisher` | `scout_geometry.yaml` |
| `camera_init → body` | dynamic | FAST-LIO `laserMapping` | FAST-LIO状态估计 |
| `body → base_link` | static | `scout_tf_manager` | `extrinsics.yaml`，完整刚体逆变换 |
| `base_link → camera_link` | static | `base_to_d435i` | `D435I.launch`实际参数`0.27 0 0.10 0 0 0` |
| RealSense内部frame | static/dynamic | `realsense2_camera` | 驱动标定 |

禁止项：

- 底盘`pub_tf=true`；
- 第二个NDT/AMCL发布`map → odom`；
- static_transform_publisher重复`camera_init → body`；
- 在多个launch中手写同一外参；
- 同时启动两个RealSense实例。

### 5.4 TF检查

```bash
rosrun tf tf_echo map odom
rosrun tf tf_echo odom camera_init
rosrun tf tf_echo camera_init body
rosrun tf tf_echo body base_link
rosrun tf tf_echo map base_link
rosrun tf tf_monitor
rosrun rqt_tf_tree rqt_tf_tree
```

FAST-LIO完成IMU初始化前，`camera_init → body`暂时不存在，启动早期一次“two or more unconnected trees”可以是正常等待；初始化后持续出现才是故障。

## 6. 关键参数完整表

### 6.1 mapper：`scout_pointcloud_mapper/config/mapper.yaml`

| 参数 | 当前值 | 作用 | 调大影响 | 调小影响 |
|---|---:|---|---|---|
| `min_range` | 0.50 m | 删除近距离点 | 删除更多近点 | 保留更多车体/近噪声 |
| `max_range` | 50 m | 最大建图距离 | 点数和噪声增加 | 远处结构缺失 |
| `scan_voxel_size` | 0.05 m | 单帧降采样 | 更稀疏、更省CPU | 更密、更耗CPU |
| `radius_filter/enable` | true | 启用半径离群点剔除 | — | false时保留离群点 |
| `radius_filter/radius` | 0.15 m | 离群邻域半径 | 更易保留稀疏群 | 更易删除稀疏点 |
| `radius_filter/min_neighbors` | 2 | 最少邻点 | 更严格 | 更宽松 |
| `self_filter/enable` | false | 自车包围盒 | 开启前必须实测 | 关闭可能保留车体点 |
| `self_filter/min_x..max_x` | -0.55～0.55 m | 自车X包围范围 | 包围更大 | 包围更小 |
| `self_filter/min_y..max_y` | -0.45～0.45 m | 自车Y包围范围 | 包围更大 | 包围更小 |
| `self_filter/min_z..max_z` | -0.40～0.25 m | 自车Z包围范围 | 包围更大 | 包围更小 |
| `dynamic_filter/enable` | true | 启用贝叶斯命中/清除 | — | false时不做动态确认 |
| `dynamic_filter/voxel_size` | 0.20 m | 贝叶斯粗占据体素 | 动态判断更粗 | 状态数量/CPU增加 |
| `hit_probability` | 0.70 | 命中log-odds | 更快确认，也更难清 | 更慢确认 |
| `miss_probability` | 0.40 | 穿过射线的空闲证据 | 越接近0.5清除越慢 | 越小清除越快 |
| `occupied_probability` | 0.72 | 输出占据阈值 | 更严格 | 更宽松 |
| `clearing_probability` | 0.35 | 删除粗体素阈值 | 更早彻底失效 | 需要更多空闲证据 |
| `min_hit_scans` | 8 | 最少命中扫描 | 动态抑制更强 | 静态确认更快 |
| `min_observation_span` | 2.0 s | 最短稳定时间 | 行人更难进入，建图变慢 | 慢目标更易进入 |
| `ray_stride` | 4 | 每N点取一条清除射线 | CPU下降、清除变慢 | CPU增加、清除更密 |
| `max_clearing_range` | 20 m | 最大清除距离 | CPU和远处清除增加 | 远处旧点不易清除 |
| `ray_endpoint_margin` | 0.30 m | 端点前保护距离 | 表面更安全、清除死区大 | 易误清表面 |
| `candidate_timeout` | 5 s | 非静态候选超时 | 状态保留更久 | 临时候选更快删除 |
| `cleanup_period` | 1 s | 过期候选清理周期 | 清理更慢 | 更频繁占CPU |
| `dynamic_filter/max_voxels` | 2,000,000 | 粗占据状态上限 | 内存上限提高 | 更早淘汰状态 |
| `map/voxel_size` | 0.05 m | 最终PCD细体素 | 地图稀疏 | 地图密、内存高 |
| `map/max_voxels` | 5,000,000 | 细地图体素上限 | 内存上限提高 | 更早停止接纳新体素 |
| `map/autosave_period` | 30 s | 自动保存周期 | 数据丢失窗口大、磁盘写少 | 写盘更频繁 |
| `map/save_on_shutdown` | true | 正常退出再保存一次 | — | false时退出不保存 |
| `map_publish_period` | 2 s | 累计地图云发布周期 | RViz更新慢、CPU低 | RViz更新快、CPU高 |
| `publish_dynamic_points` | false | 是否发布动态候选调试云 | 开启增加调试带宽/CPU | 默认不发布 |
| `max_odom_age` | 0.20 s | 点云/里程计允许时差 | 容忍延迟但位姿误差增大 | 更易跳过扫描 |
| `input_cloud/input_odom` | `/cloud_registered`、`/Odometry` | 输入接口 | 由launch可覆盖 | — |
| `static_scan_topic/static_map_topic` | `/scout/static_scan`、`/scout/static_map_cloud` | 输出接口 | 由YAML定义 | — |
| `dynamic_topic` | `/scout/dynamic_points` | 动态调试输出名 | 仅发布开关打开后有数据 | — |
| `output_path` | `maps/<map_name>/filtered_camera_init.pcd` | PCD保存位置 | 正式launch覆盖YAML中的`/tmp`默认值 | — |

### 6.2 地图生成参数

| 文件/参数 | 当前值 | 输出 |
|---|---:|---|
| `scout_raw.yaml/resolution` | 0.05 m | `map_raw.*` |
| `scout_raw.yaml/padding_m` | 0.50 m | 地图边界留白 |
| `scout_raw.yaml/floor_min_z..floor_max_z` | -0.30～0.05 m | 自由地面高度 |
| `scout_raw.yaml/obstacle_min_z..max_z` | 0.05～1.20 m | 障碍高度 |
| `scout_raw.yaml/free_dilation_m` | 0.10 m | 地面自由区域扩展 |
| `scout_raw.yaml/obstacle_inflation_m` | 0 | 正式navigation基础地图不固化膨胀 |
| `scout_nav.yaml/obstacle_inflation_m` | 0.15 m | `/map_2d`参考图；正式navigation不读取 |

`scout_nav.yaml`其余值与raw配置相同：resolution=0.05、padding=0.50、floor=-0.30～0.05、obstacle=0.05～1.20、free_dilation=0.10。0.15 m是生成参考PGM时写死到像素里的障碍膨胀；它不等于也不叠加到当前导航读取的`map_raw.yaml`。

### 6.3 NDT

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `odom_frame` | `odom` | TF child |
| `tf_postdate_sec` | 0.50 s | 降低future extrapolation |
| `ndt/debug` | true | 输出匹配调试信息 |
| `ndt/num_threads` | 4 | NDT-OMP线程 |
| `maximum_iterations` | 30 | 最大迭代 |
| `voxel_leaf_size` | 0.20 m | 实时扫描降采样 |
| `resolution` | 1.0 m | NDT网格 |
| `transformation_epsilon` | 0.01 | 收敛阈值 |
| `step_size` | 0.10 | 优化步长 |
| `thresh_shift` | 0.50 m | 位移阈值 |
| `thresh_rot` | 0.174533 rad | 约10° |
| `min_scan_range/max_scan_range` | 0.50/50 m | 匹配点范围 |

### 6.4 FAST-LIO当前参数

参数先由`FAST_LIO/config/mid360.yaml`加载，再由`fastlio_mapping_scout.launch`或`fastlio_local_odom.launch`覆盖。两种正式模式参数相同，且都关闭FAST-LIO原生PCD保存。

| 参数组 | 当前值 |
|---|---|
| `common` | `lid_topic=/livox/lidar`；`imu_topic=/livox/imu`；`time_sync_en=false`；`time_offset_lidar_to_imu=0.0` |
| `preprocess` | `lidar_type=1`；`scan_line=4`；`blind=0.5`；未覆盖项沿用源码`timestamp_unit=US`、`scan_rate=10` |
| `mapping`噪声 | `acc_cov=0.1`；`gyr_cov=0.1`；`b_acc_cov=0.0001`；`b_gyr_cov=0.0001` |
| `mapping`范围 | `fov_degree=360`；`det_range=100.0`；`cube_side_length=1000` |
| 雷达/IMU外参 | `extrinsic_est_en=false`；T=`[-0.011,-0.02329,0.04412]`；R=单位阵 |
| 算法覆盖值 | `feature_extract_enable=false`；`point_filter_num=3`；`max_iteration=3`；`filter_size_corner/surf/map=0.5/0.5/0.5`；`map_file_path`为空 |
| 发布 | `path_en=false`；`scan_publish_en=true`；`dense_publish_en=true`；`scan_bodyframe_pub_en=true` |
| 保存/日志 | `pcd_save_en=false`；`interval=-1`但不生效；`runtime_pos_log_enable=false` |

注意这里的`point_filter_num=3`是FAST-LIO内部取点间隔；mapper又对导出地图做0.05 m单帧体素和0.05 m细地图体素。两者作用阶段不同。当前规则是不修改FAST-LIO前端，动态过滤只影响最终保存地图。

### 6.5 TF与适配器参数

| 功能 | 参数 | 当前值 |
|---|---|---|
| `odom → camera_init` | xyz / rpy | `(0.25,0,0.20) m` / `(0,45,0)°` |
| `body → base_link` | 可测正向值 | 配置记录`base_link → body=(0.25,0,0.20) m,(0,45,0)°`，节点以`publish_inverse=true`发布完整逆变换 |
| pose adapter | parent/child/topic/rate | `odom` / `base_link` / `/fastlio_odom` / 20 Hz |
| cloud adapter | input/output/target | `/cloud_registered_body` / `/cloud_registered_base` / `base_link` |
| cloud adapter | queue / TF timeout | 2 / 0.10 s；按消息时间戳查TF |
| D435i安装TF | parent/child/xyz/rpy | `base_link` / `camera_link` / `(0.27,0,0.10) m` / `(0,0,0)` |

### 6.6 Scout底盘与D435i启动参数

| 节点 | 参数 | 当前值 |
|---|---|---|
| `scout_base_node` | port/model | `port_name=can0`；`is_scout_mini=true`；`is_scout_omni=false`；`simulated_robot=false` |
| `scout_base_node` | odom/TF | `odom_topic_name=/scout/odom`（系统总Launch覆盖）；`odom_frame=odom`；`base_frame=base_link`；`pub_tf=false` |
| D435i | streams | color=true；depth=true；align_depth=true；accel=false；gyro=false；pointcloud=false |
| D435i | TF | `publish_tf=true`；`tf_publish_rate=0`（静态发布） |

`scout_base.launch`单独使用时odom默认名是相对名`odom`；只有本项目系统总Launch明确传入`/scout/odom`。因此不要绕过正式入口直接启动底层Launch后再按本表期待`/scout/odom`。

### 6.7 Navigation现状（只记录，不修改）

| 文件/参数 | 当前值 |
|---|---:|
| `costmap_common.yaml/footprint` | 前0.370、后-0.300、左右±0.295 m |
| `footprint_padding` | 0.03 m |
| global/local `inflation_radius` | 0.45 m |
| `cost_scaling_factor` | 4.0 |
| local window | 6×6 m，0.05 m |
| obstacle height | 0.08～1.50 m |
| obstacle/raytrace range | 4.0/5.0 m |
| TEB `max_vel_x` | 0.35 m/s |
| TEB `max_vel_theta` | 1.0 rad/s |
| TEB `min_obstacle_dist` | 0.15 m |
| TEB `inflation_dist` | 0.35 m |
| homotopy | false |

#### move_base、costmap和GlobalPlanner完整值

| 配置 | 当前值 |
|---|---|
| move_base planner/controller | GlobalPlanner；TEB正式/DWA兼容；planner=1 Hz、patience=5 s；controller=10 Hz、patience=5 s |
| move_base recovery/oscillation | `recovery_behavior_enabled=false`；`clearing_rotation_allowed=false`；timeout=20 s；distance=0.05 m；`shutdown_costmaps=false` |
| footprint | `[(0.370,0.295),(0.370,-0.295),(-0.300,-0.295),(-0.300,0.295)]`；padding=0.03 m；transform tolerance=0.5 s |
| global costmap | frame=`map`；base=`base_link`；update/publish=2/1 Hz；rolling=false；static topic=`/nav_static_map`；updates=false |
| global inflation | radius=0.45 m；scaling=4.0 |
| local costmap | frame=`odom`；base=`base_link`；update/publish=8/4 Hz；rolling=true；6×6 m；resolution=0.05 m |
| obstacle source | `/cloud_registered_body`；frame=`body`；PointCloud2；marking/clearing=true；footprint clearing=true |
| obstacle limits | height=0.08～1.50 m；obstacle/raytrace=4/5 m；persistence=0；expected update rate=0 |
| local inflation | radius=0.45 m；scaling=4.0 |
| GlobalPlanner | `allow_unknown=false`；tolerance=0.15；Dijkstra=true；quadratic=true；grid_path=false；old_navfn=false；potential=false |

#### TEB完整值（正式入口）

| 分组 | 当前值 |
|---|---|
| frame/odom | `odom_topic=/scout/odom`；`map_frame=map` |
| trajectory | autosize=true；dt_ref=0.30；hysteresis=0.10；samples=5～50；overwrite orientation=true；backward init=false；lookahead=3.0；prune=1.0；viapoint sep=-1；ordered=false；exact arc=false；feasibility poses=10；feedback=true |
| velocity/acceleration | vx=0.35；backwards=0；vy=0；vtheta=1.0；acc_x=0.5；acc_theta=2.5；turning radius=0 |
| goal | xy/yaw tolerance=0.15/0.15；free goal velocity=false；complete global plan=true |
| obstacles | include costmap=true；min distance=0.15；inflation distance=0.35；behind=1.0；affected poses=20；dynamic=false；converter空；converter thread=true；rate=5 |
| optimization loop | inner/outer=5/4；activate=true；verbose=false；epsilon=0.05 |
| weights | max vx=2；max vtheta=1；acc x/theta=1/1；nonholonomic=1000；forward=1000；turn radius=1；optimal time=2；shortest=0；obstacle=80；inflation=0.5；dynamic=10；viapoint=0；adapt=2 |
| homotopy | enable=false；multithreading=true；simple=false；classes=2；cost hysteresis=1；obstacle scale=1；alternative time=false |
| homotopy graph | samples=10；area width=3；signature prescaler=0.5；threshold=0.1；keypoint offset=0.1；heading threshold=0.45；visualize=true |
| internal recovery | shrink=true；min duration=10；oscillation recovery=true；v/omega eps=0.10/0.10；recovery/filter duration=10/10 |

TEB polygon与costmap裸footprint一致；costmap再加0.03 m padding，TEB再以`min_obstacle_dist=0.15 m`要求车体边缘净空。它们属于不同模块的几何/代价约束，不是简单把0.03、0.15、0.35、0.45四个数相加成一个固定膨胀。

#### DWA完整值（兼容入口）

| 分组 | 当前值 |
|---|---|
| velocity | vx=0～0.35；vy=0；trans=0.05～0.35；vtheta=0.25～1.0；negative x penalized=true |
| acceleration | x=0.50；y=0；theta=2.50；trans=0.50 |
| simulation | time=3.0；linear/angular granularity=0.05/0.05；samples vx/vy/vtheta=12/1/60 |
| scoring | path=32；goal=20；obstacle=0.05；forward=0.30；stop buffer=0.30；scaling speed=0.25；max factor=0.20 |
| goal/other | xy/yaw=0.15/0.15；latch xy=true；prune=true；publish trajectory/cost clouds=true |

## 7. 地图文件与frame

| 文件 | 生成者 | frame | 消费者 |
|---|---|---|---|
| `filtered_camera_init.pcd` | mapper | `camera_init` | finalize脚本 |
| `raw_camera_init.pcd` | finalize归档 | `camera_init` | 重生成资产 |
| `public_map.pcd` | pcd_transform | `map` | NDT map loader |
| `map_raw.pgm/.yaml` | pcd_to_pgm + raw配置 | `map` | navigation map_server |
| `map.pgm/.yaml` | pcd_to_pgm + reference配置 | `map` | localization `/map_2d` |
| `map_metadata.yaml` | finalize | 不适用 | 参数追溯 |

## 8. 常见问题：症状、检查顺序、修复位置

### 8.1 找不到包或launch

症状：`is neither a launch file in package`。

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_fastlio/devel/setup.bash
rospack profile
rospack find scout_system_bringup
```

若刚执行`catkin_make --pkg`，必须重新source。修复位置不是launch内容，而是环境overlay或编译失败。

### 8.2 can0不存在或底盘无数据

```bash
rosrun scout_bringup bringup_can2usb.bash
ip -details link show can0
candump can0
rostopic hz /scout/odom
rostopic echo -n 1 /scout_status
```

依次检查USB-CAN设备名、bitrate、接口UP状态、急停和协议日志`Detected protocol: AGX_V2`。

### 8.3 Livox无点云/无IMU

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
rosnode info /livox_lidar_publisher2
```

检查雷达IP、Jetson网口IP、MID360 JSON、网线和是否重复启动驱动。

### 8.4 FAST-LIO一直不输出Odometry

```bash
rostopic hz /Odometry
rostopic hz /cloud_registered
```

先看是否出现`IMU Initial Done`。检查IMU数据、雷达类型、时间戳、外参和启动日志。初始阶段`No point, skip this scan`偶发一次可继续观察。

### 8.5 mapper一直等待Odometry或跳过扫描

日志：`waiting for FAST-LIO /Odometry`或`odometry differs by ...`。

```bash
rostopic echo -n 1 /Odometry/header
rostopic echo -n 1 /cloud_registered/header
```

检查时间戳和frame。修复位置：FAST-LIO同步配置；不要先盲目扩大`max_odom_age`。

### 8.6 mapper frame mismatch

预期`/Odometry.header.frame_id`和`/cloud_registered.header.frame_id`都为`camera_init`。

```bash
rostopic echo -n 1 /Odometry/header
rostopic echo -n 1 /cloud_registered/header
```

若不同，检查是否订错点云或上游FAST-LIO被修改。

### 8.7 `/scout/static_scan`启动后短时为空

贝叶斯静态条件要求至少8次命中且跨2秒。刚启动短时为空是设计行为。超过5秒仍空时检查输入点数、时间同步、`occupied_probability`和frame。

### 8.8 人或其他机器人留下轨迹

```bash
rostopic hz /scout/static_map_cloud
rosnode info /scout_pointcloud_mapper
```

操作：目标离开后让雷达继续看到原位置，观察日志`bayes_cleared`。完全遮挡位置必须换视角。若重新可见后仍不清，再基于rosbag比较miss证据；不要直接改navigation参数。

### 8.9 地面出现规则孔洞

检查：

```bash
rosparam get /scout_pointcloud_mapper/scan_voxel_size
rosparam get /scout_pointcloud_mapper/map/voxel_size
rosparam get /scout_pointcloud_mapper/dynamic_filter/voxel_size
```

正确值分别0.05、0.05、0.20。粗动态体素绝不能作为最终地图体素。旧PCD必须重新建图，修改参数不会修复既有PCD。

### 8.10 PCD没有自动保存

```bash
rosparam get /scout_pointcloud_mapper/output_path
rosparam get /scout_pointcloud_mapper/map/autosave_period
ls -ld ~/livox_fastlio/maps/<map_name>
```

至少等待30秒并确认日志有静态点。正常Ctrl+C会最终保存；`kill -9`和断电不会执行析构保存。

### 8.11 finalize_map.py失败

```bash
source ~/livox_fastlio/devel/setup.bash
rospack find scout_map_tools
ls -lh ~/livox_fastlio/maps/<map_name>/filtered_camera_init.pcd
rosrun scout_map_tools finalize_map.py <map_name> --replace-raw
```

检查缺失PCD、文件权限、`pcd_transform_node`/`pcd_to_pgm_node`是否已编译、YAML格式和磁盘空间。

### 8.12 public_map方向或高度错误

检查`map_metadata.yaml`保存的geometry快照，并对照：

```bash
rosparam get /scout_geometry/odom_to_camera_init
rosrun tf tf_echo odom camera_init
```

修复唯一位置：`scout_system_bringup/config/scout_geometry.yaml`。修改后重新finalize；不要在命令行另写一套变换。

### 8.13 cloud adapter报TF错误

```bash
rostopic echo -n 1 /cloud_registered_body/header
rosrun tf tf_echo body base_link
rostopic hz /cloud_registered_base
```

启动早期单次错误可能是TF缓存尚未建立；持续错误检查`tf_manager.launch`和`extrinsics.yaml`，不要删除cloud adapter。

### 8.14 pose adapter等待`odom → base_link`

依次检查：

```bash
rosrun tf tf_echo odom camera_init
rosrun tf tf_echo camera_init body
rosrun tf tf_echo body base_link
```

任一边缺失都会导致链断。FAST-LIO未初始化时等待属于正常；初始化后持续等待才修对应发布者。

### 8.15 NDT不收敛

检查顺序：

1. `public_map.pcd`存在且`/map_cloud`有点；
2. `/cloud_registered_base`有点且frame=`base_link`；
3. `/fastlio_odom`时间持续更新；
4. RViz初值位于真实位置附近，朝向合理；
5. 实时点云与地图有足够重叠；
6. 再考虑NDT参数，而不是先乱调分辨率。

### 8.16 `map → odom` future extrapolation

```bash
rosparam get /scout_global_localizer/tf_postdate_sec
rosrun tf tf_echo map odom
```

当前launch为0.50 s。还持续报错时检查系统时间、消息时间戳和是否存在第二个`map → odom`发布者。

### 8.17 TF抖动或树反复跳变

```bash
rosrun tf tf_monitor
rosnode list
rosnode info /scout_global_localizer
rosnode info /scout_base_node
```

确认底盘`pub_tf=false`，只有NDT发`map → odom`，只有FAST-LIO发`camera_init → body`。删除/关闭重复发布者，而不是增加第三条补偿TF。

### 8.18 `/map_2d`和`/nav_static_map`看起来不同

这是当前设计：

- `/map_2d`来自`map.yaml`，参考图固化膨胀0.15 m；
- `/nav_static_map`来自`map_raw.yaml`，固化膨胀0，由现有costmap运行时处理。

导航正常时不要为让两张显示图一样而修改navigation参数。

### 8.19 move_base无全局路径

```bash
rostopic echo -n 1 /nav_static_map/header
rosrun tf tf_echo map base_link
rosservice call /move_base/make_plan "..."
rostopic echo -n 1 /move_base/GlobalPlanner/plan
```

检查目标是否在已知自由区、`allow_unknown=false`、起点/终点是否被footprint或障碍占据。导航当前已验证正常，不因单次目标失败随意改全局参数。

### 8.20 TEB不发速度

```bash
rostopic hz /scout/odom
rostopic hz /cmd_vel
rostopic echo /move_base/status
rostopic echo -n 1 /move_base/local_costmap/costmap
```

确认NDT已收敛、目标有效、局部costmap有数据、底盘急停释放。`/fastlio_odom`不是TEB速度源。

### 8.21 D435i无图像或负载过高

```bash
rosnode list | grep camera
rostopic list | grep '^/camera/'
rostopic hz /camera/color/image_raw
rostopic hz /camera/aligned_depth_to_color/image_raw
```

无图像检查USB3、设备权限和驱动；负载高检查是否误开pointcloud、IMU、高分辨率或第二个相机节点。

### 8.22 两个地图都在，但`/static_map`服务指向不确定

定位和导航分别启动`scout_map_server`与`scout_navigation_map_server`，两者的地图话题不同，但上游map_server服务默认都叫`/static_map`。

```bash
rosservice info /static_map
rostopic echo -n 1 /map_2d/info
rostopic echo -n 1 /nav_static_map/info
```

程序若必须区分两张地图，应订阅明确的话题，不依赖同名服务。不要为了消除服务同名而修改已验证的导航参数。

### 8.23 Action目标无法取消、反复PREEMPTED或ABORTED

```bash
rostopic echo /move_base/status
rostopic echo -n 1 /move_base/current_goal
rostopic info /move_base/goal
rostopic info /move_base/cancel
```

检查是否同时存在RViz、脚本或第二个客户端连续发目标。取消所有目标可向`/move_base/cancel`发布空ID，但车辆运动前先确保现场安全。

### 8.24 局部costmap没有障碍或障碍长期不清

```bash
rostopic hz /cloud_registered_body
rostopic echo -n 1 /cloud_registered_body/header
rosrun tf tf_echo odom body
rostopic echo -n 1 /move_base/local_costmap/costmap
```

依次核对点云频率、时间戳、frame=`body`、TF连续性和0.08～1.50 m高度范围。`observation_persistence=0`且clearing=true；若仍残留，先查射线遮挡和TF，不改导航膨胀。

### 8.25 `/fastlio_odom`有位姿但速度全0

这是当前pose adapter的设计：它从TF生成位姿，twist字段不估计速度。NDT用它同步位姿；TEB的速度源是`/scout/odom`。不要把`/fastlio_odom`改成TEB odom话题。

### 8.26 误启动示例`fast_lio_localization.launch`

症状包括出现`base_body_tf_publisher`、`body → velodyne`、示例`IB-4L.pcd`或第二个NDT节点。立即停止示例launch，重新用：

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a
```

示例文件保留用于上游参考，不属于Scout正式链路。

### 8.27 人工遥控与导航同时争用`/cmd_vel`

```bash
rostopic info /cmd_vel
rosnode list | grep -E 'teleop|move_base'
```

正式导航时`/cmd_vel`应由`/move_base`发布。若`teleop_keybord`也在发布，停止键盘遥控launch；不要用多发布者抢占作为控制切换方案。

### 8.28 只看到话题名却没有数据

ROS节点可以先advertise话题、再因开关或初始化状态不发布消息。例如FAST-LIO的`/path`在`path_en=false`时通常无数据，mapper的动态点在`publish_dynamic_points=false`时无数据。用`rostopic hz`或`rostopic echo -n 1`判断，不能只用`rostopic list`。

### 8.29 D435i安装TF与旧注释不一致

以`D435I.launch`实际`static_transform_publisher`参数为准：`base_link → camera_link=(0.27,0,0.10) m,(0,0,0)`。若文件内旧注释提到roll=π，那是历史说明，不是当前运行参数；改相机安装外参前需实测并同步文档。

### 8.30 mapper地图被意外清空

`/scout_pointcloud_mapper/reset_map`会立即清空内存中的贝叶斯状态和细地图，不能撤销。调用前先确认当前PCD已保存并复制备份。正常建图、自动保存和finalize流程都不需要调用reset服务。

## 9. 一组可复制的完整诊断命令

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_fastlio/devel/setup.bash

rosnode list
rostopic list | sort
rosservice list | sort
rostopic list | sort
rosparam list | sort

rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /Odometry
rostopic hz /cloud_registered
rostopic hz /cloud_registered_body
rostopic hz /scout/static_scan
rostopic hz /scout/static_map_cloud
rostopic hz /scout/odom

rostopic echo -n 1 /cloud_registered/header
rostopic echo -n 1 /cloud_registered_body/header
rostopic echo -n 1 /cloud_registered_base/header
rostopic echo -n 1 /map_cloud/header
rostopic echo -n 1 /map_2d/header
rostopic echo -n 1 /nav_static_map/header

rosrun tf tf_echo map odom
rosrun tf tf_echo odom camera_init
rosrun tf tf_echo camera_init body
rosrun tf tf_echo body base_link
rosrun tf tf_echo map base_link
rosrun tf tf_monitor

rosparam get /scout_pointcloud_mapper
rosparam get /scout_global_localizer
rosparam get /move_base/global_costmap
rosparam get /move_base/local_costmap

rosnode info /laserMapping
rosnode info /scout_pointcloud_mapper
rosnode info /scout_global_localizer
rosnode info /move_base
rosservice info /static_map
```

## 10. 编译和生效规则速查

| 修改对象 | 是否编译 | 何时生效 |
|---|---|---|
| C++源码 | 是，`catkin_make -j1 --pkg <包>` | 重新source并重启节点 |
| Python脚本内容 | 通常否 | 重启节点；新安装规则需编译 |
| launch | 否 | 重启launch |
| mapper YAML | 否 | 重启mapper；旧PCD不追溯修改 |
| map tools YAML | 否 | 重新运行finalize |
| localization参数 | 否 | 重启localization |
| navigation参数 | 否 | 重启move_base；当前导航正常，本轮禁止修改 |

完整编译：

```bash
cd ~/livox_fastlio
source /opt/ros/noetic/setup.bash
catkin_make -j1
source ~/livox_fastlio/devel/setup.bash
```

## 11. 导航日志接口完整表

启动：

```bash
roslaunch scout_navigation nav_logging.launch tag:=nav_test
```

节点`/nav_log_session`执行`nav_log_session.sh`。它不发布控制话题、不设置导航参数，只做快照、rosbag记录和结束分析。输出根目录为`~/livox_fastlio/logs/navigation/`，`LAST_RUN`保存最近一次运行目录。

记录的话题分组：

- 基础：`/tf`、`/tf_static`、`/rosout_agg`、`/cmd_vel`、`/scout/odom`、`/fastlio_odom`；
- 目标与状态：`/move_base_simple/goal`、`/move_base/{status,goal,cancel,feedback,result}`；
- 全局路径：`/move_base/GlobalPlanner/plan`；
- DWA诊断：`global_plan`、`local_plan`、`trajectory_cloud`、`cost_cloud`、`parameter_updates`；
- TEB诊断：`global_plan`、`local_plan`、`teb_poses`、`teb_markers`、`teb_feedback`、`obstacles`、`via_points`、`parameter_updates`；
- costmap：局部/全局costmap、updates、footprint和障碍/膨胀动态参数；
- 环境输入：`/nav_static_map`、`/cloud_registered_body`。

结束后关键文件：

| 文件 | 含义 |
|---|---|
| `navigation*.bag` | LZ4压缩、每2 GiB分包的原始数据 |
| `summary.txt` | planner类型、频率、速度、轨迹、失败和底盘响应摘要 |
| `cmd_vel.csv`、`scout_odom_twist.csv` | 指令与底盘反馈 |
| `local_plan.csv`、`teb_poses.csv` | 局部轨迹统计 |
| `goals.csv`、`move_base_status.csv` | 目标与状态时间线 |
| `planner_fail_logs.csv` | 规划器失败日志 |
| `config_snapshot/`、`launch_snapshot/`、`move_base_params.yaml` | 本次配置证据 |
| `rosbag_info.txt`、`analysis_console.txt` | bag信息和分析错误 |

### 11.1 日志没有生成或无法分析

按顺序检查：

```bash
rosnode list | grep move_base
df -h ~
cat ~/livox_fastlio/logs/navigation/LAST_RUN
RUN_DIR=$(cat ~/livox_fastlio/logs/navigation/LAST_RUN)
ls -lh "$RUN_DIR"
cat "$RUN_DIR/analysis_console.txt"
```

仅有`.bag.active`通常表示未正常停止或rosbag异常退出。正常流程是在日志launch终端按一次`Ctrl+C`并等待`[NAV_LOG] DONE`。已有完整bag时可重跑：

```bash
rosrun scout_navigation analyze_nav_bag.py "$RUN_DIR"
```

## 12. 源码审计范围与复核方法

本表已逐项对照以下当前源码，而不是仅根据一次`rostopic list`抄写：

| 类别 | 真值来源 |
|---|---|
| 系统编排 | `scout_system_bringup/launch/*.launch` |
| 自研Launch | `scout_pointcloud_mapper`、`scout_tf_manager`、`scout_pose_adapter`、`scout_cloud_adapter`、`scout_navigation`各自`launch/` |
| 节点业务接口 | mapper、cloud adapter、NDT、map loader、global plan tester、Scout messenger和FAST-LIO源码 |
| 参数 | mapper、FAST-LIO、map tools、TF、Navigation全部当前YAML/Launch |
| 上游/示例边界 | `scout_ros`、`fast_lio_localization`、`livox_ros_driver2`、`FAST_LIO`仓库内Launch |

“完整”定义为当前Scout业务链路及仓库内会造成误启动/冲突的接口完整；以下内容天然随运行环境变化，因此明确标为运行时发现项：RealSense驱动内部nodelet和image_transport插件、RViz按显示配置产生的订阅、动态重配置自动生成话题/服务、所有节点通用的日志服务。

每次升级ROS包或修改Launch后，用下面命令生成实机证据并与本表对照：

```bash
mkdir -p ~/livox_fastlio/logs/interface_audit
rosnode list | sort | tee ~/livox_fastlio/logs/interface_audit/nodes.txt
rostopic list -v | tee ~/livox_fastlio/logs/interface_audit/topics_verbose.txt
rosservice list | sort | tee ~/livox_fastlio/logs/interface_audit/services.txt
rosparam dump ~/livox_fastlio/logs/interface_audit/params.yaml
rosrun tf2_tools view_frames.py
```

离线核对Launch节点：

```bash
roslaunch --nodes scout_system_bringup scout_mapping.launch
roslaunch --nodes scout_system_bringup scout_localization.launch
roslaunch --nodes scout_navigation navigation_teb.launch
roslaunch --nodes scout_system_bringup D435I.launch
```

## 13. 开发步骤的唯一详细来源

本表只定义运行接口和排错事实。所有必需功能包的来源、依赖、逐文件创建/修改、CMake目标、编译、启动和验收步骤统一见《Scout Mini 自主导航机器人开发文档》第17章，避免三份文档重复粘贴代码后互相失真。
