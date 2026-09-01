# Scout Mini 自主导航机器人开发文档

> 适用平台：AgileX Scout Mini、Jetson、Ubuntu 20.04、ROS Noetic、Livox Mid-360、Intel RealSense D435i。
> 车端工作空间：`~/livox_fastlio`。
> Git仓库克隆：`~/github_upload/AADCL_UAV_UGV`；Scout源码位于仓库的`Scout_mini/`子目录。
> 本手册在仓库中的路径：`Scout_mini/docs/Scout_Mini_开发文档_完整版_V3.0.md`。
> 本文按“从源码修改到实车验证”的顺序记录，每项给出实际文件、写入内容、编译目标和验证命令。
> 本文覆盖底盘、雷达、里程计、TF、静态点云建图、地图转换、重定位、导航、相机、测试和日志全部必需功能包；第17章是逐包从零恢复清单。

## 1. 最终架构与边界

```text
Scout CAN底盘
  ├─ /scout/odom：轮速反馈，供局部规划器读取速度
  └─ /cmd_vel：底盘速度命令

Mid-360 + IMU → FAST-LIO
  ├─ /Odometry：camera_init → body位姿消息
  ├─ TF camera_init → body
  ├─ /cloud_registered：camera_init中的注册扫描
  └─ /cloud_registered_body：body中的当前扫描

/cloud_registered + /Odometry → scout_pointcloud_mapper
  ├─ 几何/离群点预处理
  ├─ 3D贝叶斯占据与动态物体清除
  ├─ 0.05 m细地图累积
  └─ filtered_camera_init.pcd

filtered_camera_init.pcd → finalize_map.py
  ├─ raw_camera_init.pcd
  ├─ public_map.pcd
  ├─ map_raw.pgm / map_raw.yaml
  ├─ map.pgm / map.yaml
  └─ map_metadata.yaml

public_map.pcd + 实时base点云 + /fastlio_odom → NDT-OMP → TF map → odom
map_raw.yaml + TF + /scout/odom + 实时障碍点云 → move_base + TEB → /cmd_vel
```

边界：

- 不修改FAST-LIO的雷达输入，也不把过滤点云回灌FAST-LIO。
- FAST-LIO仍用原始Livox点云计算里程计和内部局部地图。
- mapper处理FAST-LIO去畸变、配准后的输出，负责最终交付PCD。
- FAST-LIO原生PCD保存必须关闭。
- mapper不发布TF；每条TF只能有一个发布者。
- D435i当前未并入激光PCD建图链。
- `Scout_mini`与`WheelTech`是AADCL同一仓库中的并列子项目；本手册只定义Scout Mini的开发与验收。
- 两个子项目可以共享上游算法思路，但底盘驱动、外参、话题、Launch和导航参数必须分别维护，不能跨目录直接覆盖。

## 2. 源码目录与职责

Git仓库与车端运行工作空间是两个不同位置：

```text
~/github_upload/AADCL_UAV_UGV/       # Git提交与推送目录
├─ README.md                         # 整个AADCL仓库入口
├─ Scout_mini/                       # 本手册负责的子项目
│  ├─ AGENTS.md
│  ├─ docs/*_V3.0.md
│  └─ src/
└─ WheelTech/                        # 并列子项目，由其自身文档和开发流程维护

~/livox_fastlio/                     # Scout Mini实车编译、运行和地图目录
├─ AGENTS.md
├─ src/
├─ build/
├─ devel/
├─ maps/
└─ logs/
```

开发时先在`~/livox_fastlio`修改和实车验证，再把应提交的源码与文档同步到Git克隆的`Scout_mini/`。Git操作必须在`~/github_upload/AADCL_UAV_UGV`执行。`WheelTech/`的改动不属于本手册交付范围。

Scout Mini车端工作空间内部结构：

```text
~/livox_fastlio/
├─ AGENTS.md
├─ maps/<map_name>/
└─ src/
   ├─ FAST_LIO/config/mid360.yaml
   ├─ FAST_LIO/src/laserMapping.cpp
   ├─ scout_system_bringup/
   │  ├─ config/scout_geometry.yaml
   │  └─ launch/
   │     ├─ scout_mapping.launch
   │     ├─ scout_localization.launch
   │     ├─ scout_relocalization.launch
   │     ├─ fastlio_mapping_scout.launch
   │     ├─ fastlio_local_odom.launch
   │     ├─ scout_livox_base.launch
   │     ├─ D435I.launch
   │     └─ scout_system.launch
   ├─ scout_pointcloud_mapper/
   │  ├─ CMakeLists.txt
   │  ├─ package.xml
   │  ├─ launch/pointcloud_mapper.launch
   │  ├─ config/mapper.yaml
   │  └─ src/pointcloud_mapper_node.cpp
   ├─ scout_map_tools/
   │  ├─ config/scout_raw.yaml
   │  ├─ config/scout_nav.yaml
   │  ├─ src/pcd_transform.cpp
   │  ├─ src/pcd_to_pgm.cpp
   │  └─ scripts/finalize_map.py
   ├─ scout_tf_manager/
   │  ├─ config/extrinsics.yaml
   │  ├─ launch/tf_manager.launch
   │  └─ scripts/{tf_manager.py,geometry_tf_publisher.py}
   ├─ scout_pose_adapter/
   │  ├─ launch/pose_adapter.launch
   │  └─ scripts/tf_to_odom.py
   ├─ scout_cloud_adapter/
   │  ├─ launch/cloud_adapter.launch
   │  └─ src/cloud_frame_adapter.cpp
   ├─ fast_lio_localization/src/{map_loader.cpp,fast_lio_localization.cpp}
   └─ scout_navigation/
      ├─ launch/{navigation.launch,navigation_teb.launch,
      │          global_planning_test.launch,nav_logging.launch}
      ├─ config/{costmap_common.yaml,global_costmap.yaml,local_costmap.yaml,
      │          global_planner.yaml,move_base.yaml,move_base_teb.yaml,
      │          dwa_local_planner.yaml,teb_local_planner.yaml}
      └─ scripts/{global_plan_tester.py,nav_log_session.sh,analyze_nav_bag.py}
```

`scout_system.launch`是旧入口，直接包含上游`mapping_mid360.launch`，没有贝叶斯mapper。正式建图只使用`scout_mapping.launch`。

## 3. 开发前检查

```bash
source /opt/ros/noetic/setup.bash
cd ~/livox_fastlio
test -d src
rosversion -d
uname -m

rospack find fast_lio
rospack find livox_ros_driver2
rospack find scout_base
rospack find scout_pointcloud_mapper
rospack find scout_map_tools
rospack find scout_tf_manager
rospack find scout_navigation
rospack find teb_local_planner
rospack find realsense2_camera
```

需要：ROS消息、TF/TF2、PCL、map_server、move_base、GlobalPlanner、TEB、RealSense驱动、can-utils，以及仓库内Livox/FAST-LIO/NDT/Scout驱动。

## 4. 配置FAST-LIO

### 4.1 修改`FAST_LIO/config/mid360.yaml`

```yaml
common:
  lid_topic: /livox/lidar
  imu_topic: /livox/imu

preprocess:
  lidar_type: 1
  scan_line: 4
  blind: 0.5

publish:
  path_en: false
  scan_publish_en: true
  dense_publish_en: true
  scan_bodyframe_pub_en: true

pcd_save:
  pcd_save_en: false
  interval: -1
```

不要打开`pcd_save_en`，否则得到未经过动态清除的PCD。

### 4.2 修改`scout_system_bringup/launch/fastlio_mapping_scout.launch`

```xml
<rosparam command="load" file="$(find fast_lio)/config/mid360.yaml" />
<param name="feature_extract_enable" type="bool" value="0" />
<param name="point_filter_num" type="int" value="3" />
<param name="max_iteration" type="int" value="3" />
<param name="filter_size_surf" type="double" value="0.5" />
<param name="filter_size_map" type="double" value="0.5" />
<param name="pcd_save/pcd_save_en" type="bool" value="false" />
<param name="publish/scan_bodyframe_pub_en" type="bool" value="true" />
<node pkg="fast_lio" type="fastlio_mapping" name="laserMapping" output="screen" />
```

定位用`fastlio_local_odom.launch`保持同样的PCD关闭和body点云发布设置。

### 4.3 FAST-LIO输出契约

源码：`FAST_LIO/src/laserMapping.cpp`。

| 输出 | 类型 | frame |
|---|---|---|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | `camera_init` |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | `body` |
| `/cloud_effected` | `sensor_msgs/PointCloud2` | `camera_init` |
| `/Laser_map` | `sensor_msgs/PointCloud2` | `camera_init` |
| `/Odometry` | `nav_msgs/Odometry` | parent=`camera_init`，child=`body` |
| `/path` | `nav_msgs/Path` | `camera_init`；当前`path_en=false` |
| TF `camera_init → body` | TF | FAST-LIO唯一发布 |

## 5. 建立唯一TF系统

### 5.1 TF目标

```text
建图：      odom → camera_init → body → base_link
定位导航：  map → odom → camera_init → body → base_link → camera_link...
```

### 5.2 `odom → camera_init`

唯一参数文件：`scout_system_bringup/config/scout_geometry.yaml`

```yaml
odom_to_camera_init:
  x: 0.25
  y: 0.00
  z: 0.20
  roll_deg: 0.0
  pitch_deg: 45.0
  yaw_deg: 0.0
```

发布代码：`scout_tf_manager/scripts/geometry_tf_publisher.py`

```python
t.header.frame_id = "odom"
t.child_frame_id = "camera_init"
broadcaster = tf2_ros.StaticTransformBroadcaster()
broadcaster.sendTransform(t)
```

### 5.3 `body → base_link`

参数：`scout_tf_manager/config/extrinsics.yaml`

```yaml
transforms:
  - name: body_to_base_link
    parent: base_link
    child: body
    x: 0.25
    y: 0.00
    z: 0.20
    roll_deg: 0.0
    pitch_deg: 45.0
    yaw_deg: 0.0
    publish_inverse: true
```

配置记录易测量的`base_link → body`，`tf_manager.py`对完整4×4刚体变换求逆后实际发布`body → base_link`。存在旋转时不能仅把xyz取负。

### 5.4 禁止底盘重复发布TF

所有正式launch必须写：

```xml
<include file="$(find scout_bringup)/launch/scout_mini_robot_base.launch">
  <arg name="odom_topic_name" value="/scout/odom" />
  <arg name="pub_tf" value="false" />
</include>
```

## 6. 新增点云预处理和贝叶斯地图包

包：`scout_pointcloud_mapper`

### 6.1 `package.xml`

```xml
<buildtool_depend>catkin</buildtool_depend>
<depend>boost</depend>
<depend>nav_msgs</depend>
<depend>pcl_conversions</depend>
<depend>pcl_ros</depend>
<depend>roscpp</depend>
<depend>sensor_msgs</depend>
<depend>std_srvs</depend>
```

### 6.2 `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(scout_pointcloud_mapper)
add_compile_options(-std=c++14 -O2)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_srvs)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud_mapper_node src/pointcloud_mapper_node.cpp)
target_link_libraries(pointcloud_mapper_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS pointcloud_mapper_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

### 6.3 `launch/pointcloud_mapper.launch`

```xml
<launch>
  <arg name="map_name" default="current_mapping" />
  <arg name="input_cloud" default="/cloud_registered" />
  <arg name="input_odom" default="/Odometry" />
  <arg name="output_path"
       default="$(env HOME)/livox_fastlio/maps/$(arg map_name)/filtered_camera_init.pcd" />
  <node pkg="scout_pointcloud_mapper"
        type="pointcloud_mapper_node"
        name="scout_pointcloud_mapper"
        output="screen" required="true">
    <rosparam command="load"
              file="$(find scout_pointcloud_mapper)/config/mapper.yaml" />
    <param name="input_cloud" value="$(arg input_cloud)" />
    <param name="input_odom" value="$(arg input_odom)" />
    <param name="output_path" value="$(arg output_path)" />
  </node>
</launch>
```

### 6.4 `config/mapper.yaml`

```yaml
input_cloud: /cloud_registered
input_odom: /Odometry
static_scan_topic: /scout/static_scan
static_map_topic: /scout/static_map_cloud
dynamic_topic: /scout/dynamic_points
min_range: 0.50
max_range: 50.0
scan_voxel_size: 0.05

radius_filter:
  enable: true
  radius: 0.15
  min_neighbors: 2

self_filter:
  enable: false
  min_x: -0.55
  max_x: 0.55
  min_y: -0.45
  max_y: 0.45
  min_z: -0.40
  max_z: 0.25

dynamic_filter:
  enable: true
  voxel_size: 0.20
  hit_probability: 0.70
  miss_probability: 0.40
  occupied_probability: 0.72
  clearing_probability: 0.35
  min_hit_scans: 8
  min_observation_span: 2.0
  ray_stride: 4
  max_clearing_range: 20.0
  ray_endpoint_margin: 0.30
  candidate_timeout: 5.0
  cleanup_period: 1.0
  max_voxels: 2000000

map:
  voxel_size: 0.05
  max_voxels: 5000000
  autosave_period: 30.0
  save_on_shutdown: true

map_publish_period: 2.0
publish_dynamic_points: false
max_odom_age: 0.20
output_path: /tmp/filtered_camera_init.pcd
```

粗占据体素0.20 m不等于最终地图分辨率；最终地图为0.05 m。禁止再次用粗体素直接导出地图。

### 6.5 `src/pointcloud_mapper_node.cpp`代码路径

完整源码以该文件为唯一真值，必须包含以下实现。

#### 6.5.1 ROS接口

```cpp
static_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(static_scan_topic_, 1);
static_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(static_map_topic_, 1, true);
odom_sub_ = nh_.subscribe(input_odom_, 20, &PointcloudMapper::odomCallback, this);
cloud_sub_ = nh_.subscribe(input_cloud_, 2, &PointcloudMapper::cloudCallback, this);
save_service_ = pnh_.advertiseService("save_map", &PointcloudMapper::saveMap, this);
reset_service_ = pnh_.advertiseService("reset_map", &PointcloudMapper::resetMap, this);
publish_timer_ = nh_.createTimer(ros::Duration(map_publish_period_),
                                 &PointcloudMapper::publishMapTimer, this);
autosave_timer_ = nh_.createTimer(ros::Duration(autosave_period_),
                                  &PointcloudMapper::autosaveTimer, this);
```

#### 6.5.2 预处理

`prefilter()`依次执行：NaN/Inf删除、按`/Odometry`转换到body相对坐标、距离裁剪、可选自车包围盒、PCL 0.05 m VoxelGrid、PCL RadiusOutlierRemoval。

自车包围盒默认关闭，因为最终机械外轮廓尚未实测。

#### 6.5.3 贝叶斯更新

```cpp
static double probabilityToLogOdds(double p) {
  return std::log(p / (1.0 - p));
}
```

命中，每个粗体素每帧最多一次：

```cpp
state.log_odds = std::min(max_log_odds_, state.log_odds + hit_log_odds_);
++state.hit_scans;
if (state.first_hit.isZero()) state.first_hit = stamp;
state.last_hit = stamp;
```

空闲射线穿过：

```cpp
state.log_odds = std::max(min_log_odds_, state.log_odds + miss_log_odds_);
++state.miss_scans;
if (state.log_odds <= clearing_log_odds_) {
  temporal_voxels_.erase(it);
  ++bayesian_cleared_voxels_;
  dirty_ = true;
}
```

静态输出条件：

```cpp
state.hit_scans >= min_hit_scans_
&& state.log_odds >= occupied_log_odds_
&& (state.last_hit - state.first_hit).toSec() >= min_observation_span_
```

射线每4个过滤点取1条，追踪到端点前0.30 m，最远20 m。当前帧所有端点粗体素进入保护集合，避免其他射线误清真实表面。

#### 6.5.4 generation失效机制

每个粗体素创建时获得递增generation。每个0.05 m细体素记录所属粗体素和generation。粗体素清除后，旧generation细点在构图时被删除，避免人的旧点在同一位置重新出现。

#### 6.5.5 保存

`saveMapToDisk()`将当前有效细点保存为binary PCD。dirty时每30秒保存，正常SIGINT退出时再保存。正常流程不调用服务；以下仅用于诊断：

```bash
rosservice call /scout_pointcloud_mapper/save_map
rosservice call /scout_pointcloud_mapper/reset_map
```

旧`scout_map_tools/scripts/finish_mapping.py`已删除，不应恢复。

### 6.6 动态清除边界

- 快速走动目标通常达不到2秒门槛。
- 长时间站立目标可能暂时进入；离开后必须重新看见其原位置。
- 完全遮挡且不再观察的位置无法判断为空闲。
- 多机轨迹重新可见后可清除；当前视野中的其他机器人仍是占据物。
- 真正多机互相屏蔽还需共享机器人位姿并按包围盒删点，当前未实现。

## 7. 将mapper并入一键建图

文件：`scout_system_bringup/launch/scout_mapping.launch`

```xml
<arg name="map_name" default="current_mapping" />
<include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360.launch" />
<include file="$(find scout_system_bringup)/launch/fastlio_mapping_scout.launch">
  <arg name="rviz" value="false" />
</include>
<include file="$(find scout_pointcloud_mapper)/launch/pointcloud_mapper.launch">
  <arg name="map_name" value="$(arg map_name)" />
</include>
<include file="$(find scout_tf_manager)/launch/tf_manager.launch" />
<include file="$(find scout_pose_adapter)/launch/pose_adapter.launch" />
<include file="$(find scout_bringup)/launch/scout_mini_robot_base.launch">
  <arg name="odom_topic_name" value="/scout/odom" />
  <arg name="pub_tf" value="false" />
</include>
```

```bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a
```

节点应包括Livox、`laserMapping`、mapper、两个TF节点、pose adapter和底盘。

## 8. 地图资产生成

### 8.1 文件和编译目标

| 文件 | 目标/用途 |
|---|---|
| `scout_map_tools/src/pcd_transform.cpp` | `pcd_transform_node`，camera_init PCD变换到map |
| `scout_map_tools/src/pcd_to_pgm.cpp` | `pcd_to_pgm_node`，按高度生成PGM/YAML |
| `scout_map_tools/scripts/finalize_map.py` | 串联归档、变换、两套PGM和元数据 |
| `scout_map_tools/config/scout_raw.yaml` | `map_raw.*`，障碍固化膨胀0 |
| `scout_map_tools/config/scout_nav.yaml` | `map.*`，参考图固化膨胀0.15 m |

### 8.2 实际流程

```text
filtered_camera_init.pcd
  ├─复制/覆盖→ raw_camera_init.pcd
  ├─读取scout_geometry.yaml
  ├─pcd_transform_node → public_map.pcd
  ├─scout_raw.yaml → map_raw.pgm + map_raw.yaml
  ├─scout_nav.yaml → map.pgm + map.yaml
  └─map_metadata.yaml
```

```bash
rosrun scout_map_tools finalize_map.py factory_a --replace-raw
ls -lh ~/livox_fastlio/maps/factory_a/
```

| 文件 | frame/用途 |
|---|---|
| `filtered_camera_init.pcd` | mapper实时输出，`camera_init` |
| `raw_camera_init.pcd` | 完成时归档，`camera_init` |
| `public_map.pcd` | NDT地图，`map` |
| `map_raw.pgm/.yaml` | 无固化膨胀，navigation默认读取 |
| `map.pgm/.yaml` | 固化0.15 m，localization的`/map_2d`参考图 |
| `map_metadata.yaml` | 外参与生成参数快照 |

当前navigation读取`map_raw.yaml`，所以参考图0.15 m与costmap 0.45 m在正式导航链中不叠加。本轮文档整改不修改任何navigation参数。

## 9. 重定位开发

### 9.1 点云适配

`scout_cloud_adapter/src/cloud_frame_adapter.cpp`订阅`/cloud_registered_body`，按消息时间查询TF并发布`/cloud_registered_base`，frame设为`base_link`。

`scout_cloud_adapter/launch/cloud_adapter.launch`：

```xml
<param name="input_topic" value="/cloud_registered_body" />
<param name="output_topic" value="/cloud_registered_base" />
<param name="target_frame" value="base_link" />
```

### 9.2 位姿适配

`scout_pose_adapter/scripts/tf_to_odom.py`查询`odom → base_link`，发布`/fastlio_odom`。其twist当前为0，不能给TEB作速度反馈；TEB使用`/scout/odom`。

### 9.3 NDT修改

`fast_lio_localization/src/fast_lio_localization.cpp`必须：

- 订阅`/map_cloud`和`/initialpose`；
- 同步订阅重映射后的`/cloud_registered_base`和`/fastlio_odom`；
- 只发布`map → odom`；
- TF时间戳按`tf_postdate_sec`向未来预发布。

```cpp
tfMsg.header.stamp = ros::Time::now() + ros::Duration(_cfg.tfPostdateSec);
tfMsg.header.frame_id = "map";
tfMsg.child_frame_id = _cfg.odomFrame;
_br.sendTransform(tfMsg);
```

`scout_system_bringup/launch/scout_relocalization.launch`：

```xml
<param name="odom_frame" value="odom" />
<param name="tf_postdate_sec" value="0.50" />
<param name="ndt/num_threads" value="4" />
<param name="ndt/maximum_iterations" value="30" />
<param name="ndt/voxel_leaf_size" value="0.20" />
<param name="ndt/resolution" value="1.0" />
<param name="ndt/transformation_epsilon" value="0.01" />
<param name="ndt/step_size" value="0.10" />
<remap from="/velodyne_points" to="/cloud_registered_base" />
<remap from="/odom_lio" to="/fastlio_odom" />
```

`scout_localization.launch`必须同时包含Mid-360、FAST-LIO local odom、TF manager、pose adapter、cloud adapter、map loader、NDT、map_server和底盘。

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a
```

## 10. 导航开发

### 10.1 footprint

`scout_navigation/config/costmap_common.yaml`：

```yaml
footprint:
  - [ 0.370,  0.295]
  - [ 0.370, -0.295]
  - [-0.300, -0.295]
  - [-0.300,  0.295]
footprint_padding: 0.03
transform_tolerance: 0.5
```

增加机械结构后必须重测外轮廓。

### 10.2 costmap

`global_costmap.yaml`：frame=`map`，读取`/nav_static_map`，膨胀0.45 m。
`local_costmap.yaml`：frame=`odom`，6×6 m、0.05 m分辨率，实时源`/cloud_registered_body`，障碍高度0.08～1.50 m，marking 4 m、clearing 5 m、膨胀0.45 m。

0.45 m是从`map_raw`障碍栅格向外的总渐变代价范围，不是给车体尺寸额外相加0.45 m；footprint负责车体碰撞几何。

### 10.3 TEB

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `odom_topic` | `/scout/odom` | 真实速度反馈 |
| `max_vel_x` | 0.35 m/s | 最大前进速度 |
| `max_vel_x_backwards` | 0 | 禁止主动倒车 |
| `max_vel_theta` | 1.0 rad/s | 最大角速度 |
| `min_obstacle_dist` | 0.15 m | footprint边界净空 |
| `inflation_dist` | 0.35 m | TEB软代价范围 |
| `enable_homotopy_class_planning` | false | 当前关闭多拓扑 |
| `oscillation_timeout` | 20.0 s | move_base 允许原地调整和恢复的时间 |
| `oscillation_distance` | 0.05 m | 平移超过该距离后重置振荡检测 |

#### 10.3.1 有路径但车辆不走，重新下发目标后恢复

若全局路径已经生成、目标仍为 ACTIVE，但一段时间后目标被终止，日志出现 `Robot is oscillating`，而重新下发同一目标后车辆恢复，通常是 move_base 的外层振荡判定过早触发。原地调头主要改变航向，平移量很小；旧参数要求车辆在 `5 s` 内平移 `0.20 m`，会在 TEB 完成原地调整前误判失败。

同时打开以下两个文件：

```bash
nano ~/livox_fastlio/src/scout_navigation/config/move_base_teb.yaml
nano ~/livox_fastlio/src/scout_navigation/config/move_base.yaml
```

两处统一设置：

```yaml
oscillation_timeout: 20.0
oscillation_distance: 0.05
```

TEB 自身的振荡恢复周期约为 10 s，因此外层超时应大于该周期。修改 YAML 不需要重新编译；停止并重新启动导航后验证：

```bash
rosparam get /move_base/oscillation_timeout
rosparam get /move_base/oscillation_distance
rostopic echo /move_base/status
rostopic hz /cmd_vel
```

期望运行值分别为 `20.0` 和 `0.05`。该修改只降低原地调头被误终止的概率，不用于掩盖无有效局部路径、定位跳变、障碍阻塞或底盘急停等故障。

启动顺序：

```bash
# 终端1
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a
# NDT收敛后，终端2
roslaunch scout_navigation navigation_teb.launch map_name:=factory_a
```

## 11. D435i开发

`scout_system_bringup/launch/D435I.launch`当前启用color、depth、align_depth和内部TF；关闭IMU和PointCloud2。

实际安装TF：

```xml
<node pkg="tf2_ros" type="static_transform_publisher"
      name="base_to_d435i"
      args="0.27 0.00 0.10 0 0 0 base_link camera_link"/>
```

launch注释曾写“倒装roll=pi”，但实际参数为0。以代码为准；若确实倒装，必须实测后同时修改代码和注释。

## 12. 修改文件后如何编译

### 12.1 完整编译

```bash
cd ~/livox_fastlio
source /opt/ros/noetic/setup.bash
catkin_make -j1
source ~/livox_fastlio/devel/setup.bash
```

使用`-j1`避免Jetson内存压力和并行依赖竞态。

### 12.2 编译矩阵

| 修改文件 | 命令/动作 |
|---|---|
| `scout_pointcloud_mapper/src/*.cpp`、CMake、package.xml | `catkin_make -j1 --pkg scout_pointcloud_mapper` |
| `scout_cloud_adapter/src/*.cpp` | `catkin_make -j1 --pkg scout_cloud_adapter` |
| `scout_map_tools/src/*.cpp`、CMake、package.xml | `catkin_make -j1 --pkg scout_map_tools` |
| `fast_lio_localization/src/*.cpp` | `catkin_make -j1 --pkg fast_lio_localization` |
| `FAST_LIO/src/*.cpp` | `catkin_make -j1 --pkg fast_lio` |
| 已安装的Python脚本内容 | 通常无需C++编译，重启节点；安装规则/新文件变化时编译该包 |
| `*.launch`、已有`*.yaml` | 无需编译，停止旧节点、重新source、重启launch |
| `scout_map_tools/config/*.yaml` | 无需编译；重新运行`finalize_map.py`才改变已有地图 |
| `scout_navigation/config/*.yaml` | 无需编译；重启move_base |

单包编译后仍须：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_fastlio/devel/setup.bash
```

## 13. 标准修改、部署和验证流程

### 13.1 备份

```bash
stamp=$(date +%Y%m%d_%H%M%S)
mkdir -p ~/livox_fastlio/deploy_backups/$stamp
cp -a ~/livox_fastlio/src/scout_pointcloud_mapper \
      ~/livox_fastlio/deploy_backups/$stamp/
```

按明确包目录备份，不对工作空间根目录递归删除或覆盖。

### 13.2 权限与文本检查

```bash
chmod +x ~/livox_fastlio/src/scout_tf_manager/scripts/*.py
chmod +x ~/livox_fastlio/src/scout_pose_adapter/scripts/*.py
chmod +x ~/livox_fastlio/src/scout_map_tools/scripts/*.py
chmod +x ~/livox_fastlio/src/scout_navigation/scripts/*.py
rg -n "pcd_save_en|pub_tf|dynamic_filter|tf_postdate_sec" ~/livox_fastlio/src
```

### 13.3 静态launch检查

```bash
source ~/livox_fastlio/devel/setup.bash
roslaunch --nodes scout_system_bringup scout_mapping.launch map_name:=test_map
roslaunch --nodes scout_system_bringup scout_localization.launch map_name:=test_map
roslaunch --nodes scout_navigation navigation_teb.launch map_name:=test_map
```

### 13.4 运行验证

```bash
rosnode list
rostopic hz /livox/lidar
rostopic hz /Odometry
rostopic hz /cloud_registered
rostopic hz /scout/static_scan
rostopic echo -n 1 /scout/static_map_cloud/header
rosrun tf tf_echo odom base_link
```

mapper日志应包含：

```text
Mapper: input=... filtered=... static_scan=... map=... occupancy=... bayes_cleared=...
Saved ... fine static-map points to .../filtered_camera_init.pcd
```

### 13.5 动态目标实测

1. 启动正式建图；
2. 让人员横穿；
3. 人员离开后继续观察原位置；
4. 查看`/scout/static_map_cloud`；
5. 观察`bayes_cleared`是否增加；
6. 检查最终PCD；
7. 遮挡区域换视角后再判断。

### 13.6 地图文件检查

```bash
ls -lh ~/livox_fastlio/maps/factory_a/
head -n 11 ~/livox_fastlio/maps/factory_a/filtered_camera_init.pcd
```

二进制PCD头的`POINTS`必须大于0，不能只看文件大小。

## 14. 完整实车流程

```bash
# CAN
rosrun scout_bringup bringup_can2usb.bash
ip -details link show can0

# 建图
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a

# Ctrl+C正常结束后生成地图资产
rosrun scout_map_tools finalize_map.py factory_a --replace-raw

# 定位
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a

# NDT收敛后导航
roslaunch scout_navigation navigation_teb.launch map_name:=factory_a
```

## 15. 已验证结果

- mapper单包编译通过；
- 45秒静态场景自动保存成功；
- PCD约60,645点、948 KiB；
- mapper约21%单核CPU、68 MiB RSS；
- FAST-LIO、底盘和TF节点同时运行正常。

该测试只证明编译、启动、静态保留和保存；人员动态清除必须按13.5节实测，不能用静态测试代替。

## 16. 禁止事项和已知问题

- 禁止FAST-LIO与mapper同时保存交付PCD。
- 禁止重复发布任一TF边。
- 禁止把`/fastlio_odom`作为TEB速度源，其twist为0。
- 禁止用旧`scout_system.launch`正式建图。
- 禁止混淆`map.yaml`与`map_raw.yaml`。
- FAST-LIO完成IMU初始化前，TF树可能短暂未连接；持续报错才是故障。
- D435i倒装注释与实际零旋转参数不一致，待实测。
- 多机器人主动互相屏蔽尚未实现。
- 当前CAN不能确认SOC百分比。

## 17. 全部必需功能包逐包开发与恢复步骤

本章用于在一台新的Jetson上从仓库恢复完整系统，也用于判断改某项功能时究竟要改哪个包。每个包均给出来源、文件、实现契约、编译、启动和验收。第三方包保留上游源码，仅做本项目所需配置；自研包必须随仓库完整保存。

### 17.1 功能包覆盖矩阵

| 层 | 必需包 | 性质 | 本项目修改入口 | 编译/解释执行 |
|---|---|---|---|---|
| CAN与底盘 | `ugv_sdk`、`scout_msgs`、`scout_base`、`scout_bringup` | AgileX上游 | `scout_mini_robot_base.launch`参数 | `scout_base_node` |
| Livox | `Livox-SDK2`、`livox_ros_driver2` | Livox上游 | `MID360_config.json`、ROS1 launch | `livox_ros_driver2_node` |
| 激光里程计 | `fast_lio` | FAST-LIO上游+项目配置 | `mid360.yaml`、两个包装launch | `fastlio_mapping` |
| 总入口 | `scout_system_bringup` | 自研 | `config/`和`launch/`全部文件 | launch-only |
| TF | `scout_tf_manager` | 自研 | 两个Python、YAML、launch | Python |
| 静态地图 | `scout_pointcloud_mapper` | 自研 | C++、YAML、launch | `pointcloud_mapper_node` |
| 地图资产 | `scout_map_tools` | 自研 | 两个C++、Python、两份YAML | 两个节点+Python |
| 点云坐标适配 | `scout_cloud_adapter` | 自研 | C++、launch | `cloud_frame_adapter_node` |
| 位姿适配 | `scout_pose_adapter` | 自研 | Python、launch | Python |
| 重定位 | `fast_lio_localization` | 上游基础+项目修改 | `map_loader.cpp`、`fast_lio_localization.cpp`、系统包装launch | 两个C++节点 |
| 导航/测试/日志 | `scout_navigation` | 自研配置与脚本 | config、launch、3个脚本 | move_base+Python/Bash |
| RGB-D | `realsense2_camera` | Intel上游 | `D435I.launch` | 驱动节点 |

### 17.2 新设备的系统依赖与工作空间

先安装ROS Noetic，再安装本项目运行依赖：

```bash
sudo apt update
sudo apt install -y git can-utils libpcl-dev libeigen3-dev libyaml-cpp-dev
sudo apt install -y ros-noetic-pcl-ros ros-noetic-tf ros-noetic-tf2-ros ros-noetic-tf2-sensor-msgs
sudo apt install -y ros-noetic-navigation ros-noetic-teb-local-planner ros-noetic-map-server
sudo apt install -y ros-noetic-realsense2-camera ros-noetic-rosbag ros-noetic-rqt-tf-tree
```

恢复源码并解析ROS依赖：

```bash
mkdir -p ~/github_upload
cd ~/github_upload

GIT_SSH_COMMAND="ssh -p 443 -i $HOME/.ssh/id_ed25519_github_aadcl -o IdentitiesOnly=yes -o Hostname=ssh.github.com" \
  git clone git@github.com:BAIOLED/AADCL_UAV_UGV.git

mkdir -p ~/livox_fastlio/src
cp -a ~/github_upload/AADCL_UAV_UGV/Scout_mini/src/. ~/livox_fastlio/src/
cp -a ~/github_upload/AADCL_UAV_UGV/Scout_mini/AGENTS.md ~/livox_fastlio/AGENTS.md

cd ~/livox_fastlio
source /opt/ros/noetic/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make -j1
source devel/setup.bash
```

每次新终端都必须`source /opt/ros/noetic/setup.bash`和`source ~/livox_fastlio/devel/setup.bash`。只改launch/YAML通常不必编译；改C++、`CMakeLists.txt`、`package.xml`或新增可执行脚本后必须编译。

### 17.3 AgileX底盘包：`ugv_sdk`、`scout_msgs`、`scout_base`、`scout_bringup`

文件职责：

- `ugv_sdk/`：AGX_V2 CAN协议、报文解析和底盘控制库；不在项目层复制协议实现。
- `scout_ros/scout_msgs/`：底盘状态消息。
- `scout_ros/scout_base/src/scout_base_node.cpp`：ROS节点入口。
- `scout_ros/scout_base/src/scout_messenger.cpp`：`/cmd_vel`到SDK命令、底盘状态与里程计发布。
- `scout_ros/scout_bringup/scripts/bringup_can2usb.bash`：建立`can0`。
- `scout_ros/scout_bringup/launch/scout_mini_robot_base.launch`：正式底盘入口。

本项目集成时必须在所有系统launch中保持：

```xml
<include file="$(find scout_bringup)/launch/scout_mini_robot_base.launch">
  <arg name="odom_topic_name" value="/scout/odom" />
  <arg name="pub_tf" value="false" />
</include>
```

`pub_tf=false`是TF唯一性要求，不能因为看不到`odom→base_link`就打开；该链由FAST-LIO和自研TF包组合得到。编译与验收：

```bash
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
rosrun scout_bringup bringup_can2usb.bash
ip -details link show can0
candump can0
roslaunch scout_bringup scout_mini_robot_base.launch odom_topic_name:=/scout/odom pub_tf:=false
rostopic hz /scout/odom
rostopic echo -n 1 /scout_status
```

若CAN无数据，依次检查接口是否UP、波特率、USB-CAN、底盘急停和协议日志`Detected protocol: AGX_V2`。当前工程没有可靠SOC百分比来源，不要由电压伪造电量百分比。

### 17.4 Livox包：`Livox-SDK2`与`livox_ros_driver2`

保留文件：`Livox-SDK2/`完整SDK、`livox_ros_driver2/config/MID360_config.json`、`livox_ros_driver2/launch_ROS1/msg_MID360.launch`。开发时只配置雷达/主机IP、点云格式和发布频率，不修改底层回调，除非官方驱动存在已定位缺陷。

`MID360_config.json`中的主机IP必须属于Jetson连接雷达的网卡，雷达IP必须与实物一致。ROS1入口必须发布FAST-LIO所需的Livox自定义点云和IMU。编译与验收：

```bash
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
rostopic type /livox/lidar
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic echo -n 1 /livox/imu/header
```

若无数据，先检查`ip addr`与雷达网段，再检查JSON，最后看驱动日志；不要先改FAST-LIO参数。

### 17.5 FAST-LIO包：`fast_lio`

源码入口为`FAST_LIO/src/laserMapping.cpp`与`preprocess.cpp`，本项目原则是不改其原始输入链。项目只维护：

- `FAST_LIO/config/mid360.yaml`：Livox/IMU话题、外参、滤波和`pcd_save_en: false`；
- `scout_system_bringup/launch/fastlio_mapping_scout.launch`：建图模式包装；
- `scout_system_bringup/launch/fastlio_local_odom.launch`：定位模式本地里程计包装。

具体参数和输出契约见第4章。两个包装launch都必须关闭FAST-LIO原生PCD保存，并开启`scan_bodyframe_pub_en`以供重定位/局部避障使用。编译及独立验收：

```bash
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
roslaunch scout_system_bringup fastlio_mapping_scout.launch rviz:=false
rostopic hz /Odometry
rostopic hz /cloud_registered
rostopic hz /cloud_registered_body
rosrun tf tf_echo camera_init body
```

首次启动需等待IMU初始化。`/Odometry`和TF持续缺失时检查IMU与点云时间戳、话题名和外参，不能用静态TF冒充FAST-LIO动态位姿。

### 17.6 总入口包：`scout_system_bringup`

这个包不实现算法，只负责按正确顺序组合节点。必须保存以下文件：

- `config/scout_geometry.yaml`：`odom→camera_init`唯一几何真值；
- `launch/scout_livox_base.launch`：雷达+底盘，不含FAST-LIO；
- `launch/fastlio_mapping_scout.launch`：FAST-LIO建图包装；
- `launch/fastlio_local_odom.launch`：FAST-LIO定位包装；
- `launch/scout_mapping.launch`：正式一键建图；
- `launch/scout_relocalization.launch`：NDT节点和参数；
- `launch/scout_localization.launch`：正式一键定位；
- `launch/D435I.launch`：相机；
- `launch/scout_system.launch`：遗留入口，不能用于正式建图。

`scout_mapping.launch`的固定顺序是Livox→FAST-LIO→mapper→TF→pose adapter→底盘；`scout_localization.launch`是Livox→FAST-LIO→TF→pose adapter→cloud adapter→NDT→`/map_2d`→底盘。新增include时必须先检查节点名、话题和TF是否重复。

这是launch-only包，修改后执行静态检查即可：

```bash
roslaunch --files scout_system_bringup scout_mapping.launch
roslaunch --files scout_system_bringup scout_localization.launch
roslaunch scout_system_bringup scout_mapping.launch map_name:=dev_test
```

### 17.7 TF包：`scout_tf_manager`

逐文件创建：

1. `package.xml`声明`rospy`、`geometry_msgs`、`tf`、`tf2_ros`；
2. `CMakeLists.txt`用`catkin_install_python`安装`tf_manager.py`和`geometry_tf_publisher.py`；
3. `config/extrinsics.yaml`描述外参，并用`publish_inverse: true`发布完整刚体逆变换`body→base_link`；
4. `geometry_tf_publisher.py`读取`/scout_geometry/odom_to_camera_init`，角度转弧度并由`StaticTransformBroadcaster`只发布`odom→camera_init`；
5. `tf_manager.py`用4×4齐次矩阵求逆，禁止只把XYZ取负；
6. `launch/tf_manager.launch`加载`scout_geometry.yaml`和`extrinsics.yaml`，各启动一个脚本。

完整TF约束见第5章。编译与验收：

```bash
chmod +x ~/livox_fastlio/src/scout_tf_manager/scripts/*.py
cd ~/livox_fastlio && catkin_make -j1
source devel/setup.bash
roslaunch scout_tf_manager tf_manager.launch
rosrun tf tf_echo odom camera_init
rosrun tf tf_echo body base_link
rosrun tf tf_monitor
```

验收标准是每条边只有一个publisher，数值与YAML一致，且`camera_init→body`仍由FAST-LIO发布。

### 17.8 静态点云与动态清除包：`scout_pointcloud_mapper`

这是交付PCD的唯一生成包。逐文件开发顺序：

1. 创建`package.xml`，依赖`boost nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_srvs`；
2. 创建`CMakeLists.txt`，启用C++14，生成`pointcloud_mapper_node`并链接PCL/catkin；
3. 创建`config/mapper.yaml`，集中保存输入输出、几何过滤、离群点、贝叶斯、射线清除、细地图和自动保存参数；
4. 创建`launch/pointcloud_mapper.launch`，加载YAML并把`map_name`展开到`~/livox_fastlio/maps/<name>/filtered_camera_init.pcd`；
5. 创建`src/pointcloud_mapper_node.cpp`，实现第6章的同步位姿查找、预处理、log-odds hit/miss、generation失效、细体素候选、周期发布、服务保存、定时保存和正常退出保存；
6. 将launch include到`scout_mapping.launch`，不能要求操作者另开预处理命令。

输入必须是`/cloud_registered`与`/Odometry`，即在FAST-LIO输出端处理，而不是回灌其输入。当前没有对最终地图做规则的全局降采样；0.20 m只用于贝叶斯证据，0.05 m用于交付细地图。具体代码契约和参数见第6章。

```bash
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
roslaunch scout_pointcloud_mapper pointcloud_mapper.launch map_name:=mapper_test
rostopic hz /scout/filtered_scan
rostopic hz /scout/static_map_cloud
rosservice call /scout_pointcloud_mapper/save_map
```

验收需同时覆盖静态物体保留、人员横穿后清除、遮挡解除后更新、PCD非空和Jetson资源占用。多机建图目前只靠观测时序清动态物体，没有机器人身份掩膜，因此不能宣称百分之百消除其他机器人轨迹。

### 17.9 地图资产包：`scout_map_tools`

逐文件开发：

- `src/pcd_transform.cpp`：读取源PCD，应用统一刚体变换，输出`public_map.pcd`；
- `src/pcd_to_pgm.cpp`：按高度切片、分辨率和膨胀生成PGM/YAML；
- `config/scout_raw.yaml`：`obstacle_inflation_m: 0.0`，生成导航实际加载的`map_raw.*`；
- `config/scout_nav.yaml`：`obstacle_inflation_m: 0.15`，生成参考`map.*`；
- `scripts/finalize_map.py`：校验目录和输入PCD，依次调用两个C++工具并写`map_metadata.yaml`。

`CMakeLists.txt`目标名必须是`pcd_transform_node`、`pcd_to_pgm_node`，并安装`finalize_map.py`。编译与执行：

```bash
chmod +x ~/livox_fastlio/src/scout_map_tools/scripts/finalize_map.py
cd ~/livox_fastlio && catkin_make -j1
source devel/setup.bash
rosrun scout_map_tools finalize_map.py factory_a --replace-raw
ls -lh ~/livox_fastlio/maps/factory_a/
```

这里的0.15 m是参考栅格生成膨胀，不是车体footprint，也不会与当前正式导航的0.45 m叠加，因为正式导航读取`map_raw.yaml`。未经用户明确要求，不修改导航costmap参数。

### 17.10 点云坐标适配包：`scout_cloud_adapter`

用途是把`/cloud_registered_body`从消息原frame变换为`base_link`，输出`/cloud_registered_base`供NDT；它不保存地图、不发布TF。开发步骤：

1. `package.xml`声明`roscpp sensor_msgs geometry_msgs tf2_ros tf2_sensor_msgs`；
2. `CMakeLists.txt`生成`cloud_frame_adapter_node`；
3. `src/cloud_frame_adapter.cpp`创建TF buffer/listener，按点云原时间戳查`target_frame←source_frame`，调用`tf2::doTransform`并保留时间戳；
4. `launch/cloud_adapter.launch`设置输入、输出和`target_frame=base_link`；
5. 只在定位链启动，不在建图链增加重复点云转换。

```bash
cd ~/livox_fastlio && catkin_make -j1
source devel/setup.bash
roslaunch scout_cloud_adapter cloud_adapter.launch
rostopic echo -n 1 /cloud_registered_base/header
rosrun tf tf_echo body base_link
```

持续报`Cloud TF failed`时检查TF树和时间戳；不要通过删除坐标转换launch来消除报错。

### 17.11 位姿适配包：`scout_pose_adapter`

用途是把已经存在的TF`odom→base_link`封装成`nav_msgs/Odometry`的`/fastlio_odom`，供NDT和对比分析使用。开发步骤：

1. `package.xml`依赖`geometry_msgs nav_msgs rospy tf2_ros`；
2. `CMakeLists.txt`安装`tf_to_odom.py`；
3. Python以20 Hz查询最新TF，复制pose和frame，发布Odometry；
4. twist明确写0，因为这个节点没有速度估计；
5. `pose_adapter.launch`固定parent=`odom`、child=`base_link`、output=`/fastlio_odom`。

```bash
chmod +x ~/livox_fastlio/src/scout_pose_adapter/scripts/tf_to_odom.py
cd ~/livox_fastlio && catkin_make -j1
source devel/setup.bash
roslaunch scout_pose_adapter pose_adapter.launch
rostopic hz /fastlio_odom
rostopic echo -n 1 /fastlio_odom
```

`/fastlio_odom.twist`为0是设计结果，TEB速度反馈必须继续使用`/scout/odom`。

### 17.12 重定位包：`fast_lio_localization`

所需文件与职责：

- `src/map_loader.cpp`读取`map_path`并发布全局PCD；
- `src/fast_lio_localization.cpp`订阅地图、实时base点云和本地里程计，运行NDT-OMP并发布唯一`map→odom`；
- `src/pclomp/*`提供并行NDT/GICP；
- `scout_system_bringup/launch/scout_relocalization.launch`保存本项目所有NDT参数和remap；
- 上游示例`fast_lio_localization.launch`含示例静态TF和RViz，不作为正式入口。

项目修改必须保留`odom_frame`与`tf_postdate_sec`参数，并确保TF时间戳使用扫描时间加postdate，解决move_base未来查询；同时绝不能创建第二个`map→odom`发布者。编译和验收：

```bash
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a
rostopic echo -n 1 /cloud_registered_base/header
rostopic echo -n 1 /fastlio_odom/header
rosrun tf tf_echo map odom
rosrun tf tf_echo map base_link
```

验收看NDT是否收敛、地图与实时点云是否重合、`map→odom`是否稳定且仅一个authority。不要靠调整导航参数掩盖重定位错误。

### 17.13 导航、全局规划测试与日志包：`scout_navigation`

必须保存：

- `config/costmap_common.yaml`：footprint、障碍源和膨胀公共项；
- `config/global_costmap.yaml`、`local_costmap.yaml`：frame、静态/滚动窗口和插件；
- `config/global_planner.yaml`：GlobalPlanner；
- `config/move_base_teb.yaml`、`teb_local_planner.yaml`：正式TEB；
- `config/move_base.yaml`、`dwa_local_planner.yaml`：遗留/对照测试；
- `launch/navigation_teb.launch`：正式导航，加载`map_raw.yaml`，odom重映射到`/scout/odom`；
- `launch/global_planning_test.launch`与`scripts/global_plan_tester.py`：屏蔽真实`cmd_vel`，用RViz Publish Point调用`/move_base/make_plan`；
- `launch/nav_logging.launch`、`scripts/nav_log_session.sh`、`scripts/analyze_nav_bag.py`：导航记录和离线分析。

脚本安装项必须写入`CMakeLists.txt`：

```cmake
catkin_install_python(PROGRAMS
  scripts/global_plan_tester.py
  scripts/analyze_nav_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/nav_log_session.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

正式启动和验收：

```bash
cd ~/livox_fastlio && catkin_make -j1
source devel/setup.bash
roslaunch scout_navigation navigation_teb.launch map_name:=factory_a
rosparam get /move_base/base_local_planner
rostopic hz /cmd_vel
rostopic hz /scout/odom
rosservice type /move_base/make_plan
```

导航现状已验证，本章只记录恢复方法；除用户明确要求外，不改footprint、costmap、TEB或move_base参数。

#### 17.13.1 导航日志完整开发和使用

`nav_log_session.sh`必须完成以下实现：检查ROS master；创建`~/livox_fastlio/logs/navigation/<时间>_<tag>/`；快照config、launch、`/move_base`参数、节点/话题/服务和磁盘信息；用LZ4记录TF、目标、状态、全局/局部路径、DWA/TEB诊断、costmap、`/scout/odom`、`/cmd_vel`、静态地图和障碍点云；2 GiB分包；收到Ctrl+C时先正常停止rosbag，再调用分析脚本。

使用流程：

```bash
# 先启动定位与导航，再启动日志
roslaunch scout_navigation nav_logging.launch tag:=factory_a_teb_01
# 完成一次或多次目标测试后，在日志终端Ctrl+C

cat ~/livox_fastlio/logs/navigation/LAST_RUN
RUN_DIR=$(cat ~/livox_fastlio/logs/navigation/LAST_RUN)
cat "$RUN_DIR/summary.txt"
rosbag info "$RUN_DIR"/*.bag
```

`analyze_nav_bag.py <运行目录或bag>`输出`summary.txt`以及`cmd_vel.csv`、`scout_odom_twist.csv`、`local_plan.csv`、`teb_poses.csv`、`move_base_status.csv`、`goals.csv`和`planner_fail_logs.csv`。手动重新分析：

```bash
rosrun scout_navigation analyze_nav_bag.py "$RUN_DIR"
```

日志只观察系统，不改变导航参数。缺少`summary.txt`时检查是否用Ctrl+C正常结束、是否存在`.bag.active`、磁盘是否满、`rosbag_info.txt`和`analysis_console.txt`中的错误。

### 17.14 D435i与`realsense2_camera`

驱动使用官方`realsense2_camera`，项目只维护`scout_system_bringup/launch/D435I.launch`。launch必须启用color、depth、camera_info、对齐深度和官方TF，关闭accel、gyro和PointCloud2以降低Jetson负载；另由唯一静态发布者提供`base_link→camera_link`安装外参。

```bash
roslaunch scout_system_bringup D435I.launch
rostopic hz /camera/color/image_raw
rostopic hz /camera/aligned_depth_to_color/image_raw
rostopic echo -n 1 /camera/color/camera_info
rosrun tf tf_echo base_link camera_link
```

当前文件注释写“倒装roll=pi”，实际参数是零旋转。必须实测画面方向和点坐标后统一注释与参数，不能仅凭注释改外参。D435i当前不参与激光PCD建图。

### 17.15 修改任一包后的统一交付验收

```bash
cd ~/livox_fastlio
git diff --check
find src -type f \( -name '*.py' -o -name '*.sh' \) -exec chmod +x {} \;
catkin_make -j1
source devel/setup.bash

roslaunch --files scout_system_bringup scout_mapping.launch
roslaunch --files scout_system_bringup scout_localization.launch
roslaunch --files scout_navigation navigation_teb.launch

roslaunch scout_system_bringup scout_mapping.launch map_name:=acceptance_test
# 正常Ctrl+C并确认filtered_camera_init.pcd
rosrun scout_map_tools finalize_map.py acceptance_test --replace-raw
roslaunch scout_system_bringup scout_localization.launch map_name:=acceptance_test
# NDT稳定后
roslaunch scout_navigation navigation_teb.launch map_name:=acceptance_test
roslaunch scout_navigation nav_logging.launch tag:=acceptance_test
```

最终必须核对：CAN、Livox点云/IMU、FAST-LIO odom、四段TF唯一性、动态清除、PCD与两套栅格、NDT收敛、TEB输入输出、D435i话题和日志总结。任何一项失败都按所属包修复，禁止用改导航参数补偿建图、TF或定位问题。

## 18. 文档与版本管理

每次Scout功能修改同步更新：源码、开发文档、接口排错表、使用手册和`AGENTS.md`。`~/livox_fastlio`是实车运行工作空间，不是Git仓库；提交前把验证通过的Scout文件同步到Git克隆的`Scout_mini/`，不要连同`build`、`devel`、地图或日志一起复制。

建议先检查运行工作空间与Git克隆的差异，再按实际修改文件同步。下面命令仅示范Scout自研包和文档，执行前必须核对目标路径：

```bash
WORKSPACE=~/livox_fastlio
REPOSITORY=~/github_upload/AADCL_UAV_UGV

cp -a "$WORKSPACE/src/scout_system_bringup" "$REPOSITORY/Scout_mini/src/"
cp -a "$WORKSPACE/src/scout_tf_manager" "$REPOSITORY/Scout_mini/src/"
cp -a "$WORKSPACE/src/scout_pointcloud_mapper" "$REPOSITORY/Scout_mini/src/"
cp -a "$WORKSPACE/src/scout_map_tools" "$REPOSITORY/Scout_mini/src/"
cp -a "$WORKSPACE/src/scout_cloud_adapter" "$REPOSITORY/Scout_mini/src/"
cp -a "$WORKSPACE/src/scout_pose_adapter" "$REPOSITORY/Scout_mini/src/"
cp -a "$WORKSPACE/src/scout_navigation" "$REPOSITORY/Scout_mini/src/"
cp -a "$WORKSPACE/AGENTS.md" "$REPOSITORY/Scout_mini/AGENTS.md"
cp -a "$WORKSPACE/docs/." "$REPOSITORY/Scout_mini/docs/"
```

然后只在Git克隆中检查、提交和推送：

```bash
cd ~/github_upload/AADCL_UAV_UGV
git status --short
git diff --check
git diff --stat

git add Scout_mini
git diff --cached --check
git diff --cached --stat
git commit -m "说明Scout Mini本次修改内容"

GIT_SSH_COMMAND="ssh -p 443 -i $HOME/.ssh/id_ed25519_github_aadcl -o IdentitiesOnly=yes -o Hostname=ssh.github.com" \
  git push origin main
```

仓库：`https://github.com/BAIOLED/AADCL_UAV_UGV.git`。主分支为`main`。`Scout_mini/`和`WheelTech/`均属于该仓库，但提交Scout任务时只暂存`Scout_mini`相关路径，保留另一子项目同期提交并使用普通fetch/rebase，禁止强制覆盖远端。

禁止提交PCD、rosbag、build、devel、日志、密码、Token或SSH私钥。完整认证配置见`Scout_mini/docs/GitHub_上传配置与安全说明_V3.0.md`。
