# Scout 相机—Mid-360静态外参标定

本工具适配hku-mars/livox_camera_calib多场景离线标定。不是Kalibr，不需要把车辆抬起来，也不使用AprilGrid黑白纹理作为雷达几何边缘。不修改FAST-LIO、导航参数、原始驱动话题或TF。默认数据目录`~/r3live_ws/calibration`，测试数据另存`calibration_test`/`calibration_synthetic_20260910`，不可当成实车已标定结果。

## 1. 安装

在Scout执行，前提为独立R³LIVE workspace已安装：

```bash
cd ~/github_upload/ugv
bash Scout_mini/optional/r3live/calibration/install_calibration.sh
~/r3live_ws/scout_calibrate.sh --help
```

安装脚本固定上游提交`061fdaa647fc806e59d73a8505a05a10dfcfdaa1`，安装Ceres开发库，在`~/lidar_camera_calib_ws`串行编译，复用R³LIVE overlay的cv_bridge/OpenCV。上游补丁完整保存在`scout-calibration.patch`。保留MIT许可；若HTTPS不可达，可以先用设备已有GitHub SSH连接克隆同一仓库到上述workspace/src，再运行安装脚本。安装脚本不覆盖不同提交上的用户修改。

## 2. 现场采集

选3个有墙角、门框、柜子等真实三维交线的不同位置作为求解数据，另选至少1个不同位置验证。不要只拍平墙、印刷图案、玻璃或重复长廊。主体先选1～5米内，保证相机清晰、雷达有足够回波。

第一个终端启动传感器：

```bash
~/r3live_ws/scout_calibrate.sh sensors
```

已有`/livox/lidar`发布者时复用雷达，不启动第二个驱动；已有同名相机流也复用。发现其他命名的彩色相机流时拒绝打开第二个相机。原有定位节点不会被关闭。该命令不启动底盘或定位；如果两种传感器已存在则直接退出并提示可以采集。任何现存发布者是否健康仍由capture验证。

第二个终端，在每次停车、放开遥控并确认场景静止后执行：

```bash
~/r3live_ws/scout_calibrate.sh capture scene_00
```

输入`YES`，录制默认6秒并自动导出，直到`CAPTURE_OK`后再移动车辆。换位置依次执行：

```bash
~/r3live_ws/scout_calibrate.sh capture scene_01
~/r3live_ws/scout_calibrate.sh capture scene_02
~/r3live_ws/scout_calibrate.sh capture validation_00
```

每条命令都必须在对应位置停车后单独执行，不能把四条一起执行当成四个不同场景。工具不发送停车指令；IMU旋转阈值0.035rad/s、可用轮速线速度阈值0.02m/s只用于拒绝疑似移动数据，不能证明平移静止或检测所有场景动态物体。加速度仍是Livox原始g，本工具不做相机—IMU动态标定，不修改它。

默认不覆盖同名场景。失败时保留`.incomplete`和原始日志/bag，不能用于求解。检查原因后用新场景名重录。可用`--seconds 3`至`--seconds 10`调整录制时长。`--stationary`仅供已确认现场静止的非交互调用，不是绕过运动检查。

## 3. 准备和求解

```bash
~/r3live_ws/scout_calibrate.sh prepare run_01 scene_00 scene_01 scene_02
~/r3live_ws/scout_calibrate.sh solve run_01
```

prepare自动从真实CameraInfo取K/D，通过rig安装初值和相机内部光学TF计算雷达→相机初值，复制编号BMP/PCD并生成多场景和边缘提取配置。仅使用同一分辨率、内参和帧名的数据，拒绝重复点云冒充多个场景。

solve默认使用独立ROS master端口11441，可用`--port 11442`切换空闲端口。无需显示器，不开RViz；初始、粗标定、优化后投影图写入run目录。默认超时1800秒。失败保留`.solving`，重新准备一个新run后再求解；不能把残留矩阵当成成功结果。

检查进度：

```bash
tail -f ~/r3live_ws/calibration/runs/run_01/solver.log
```

程序拒绝几何边缘过少、匹配少于30或Ceres返回不可用解。`CANDIDATE_READY`只说明计算得到候选矩阵，不等于标定精度合格。上游粗搜索只在初值附近搜索旋转，不能修复任意错误的安装方向和平移。

## 4. 独立投影验收

```bash
~/r3live_ws/scout_calibrate.sh project run_01 validation_00
```

查看`runs/run_01/validation/validation_00/`：

- `projection.png`：原图叠加最近深度点，颜色表示深度。
- `edges.png`：绿色是图像边缘，红色是近似深度突变与局部法向变化边缘。
- `report.yaml`：覆盖像素、近似边缘距离、是否独立场景、矩阵和点云哈希。

这些边缘距离受纹理、遮挡和点云密度影响，不是定位精度，也不是官方优化器的平面交线残差。必须人工检查不同距离、不同方向和画面区域。独立场景既不能是训练名字，也不能与训练点云文件内容相同。

## 5. 明确接受后用于R³LIVE

仅在检查独立投影后执行：

```bash
~/r3live_ws/scout_calibrate.sh accept run_01 --confirm-validation
```

生成`runs/run_01/accepted_calibration.yaml`，不修改任何运行中的定位或默认外参。停止原定位入口后显式启动：

```bash
~/r3live_ws/start_scout_r3live.sh \
  calibration_file:=/home/nvidia/r3live_ws/calibration/runs/run_01/accepted_calibration.yaml
```

R³LIVE验证接受状态、矩阵、相机分辨率/K/D/光学帧、雷达帧和内部雷达IMU平移；改变相机模式或安装后不能继续沿用旧外参。`calibration_io.py`执行`T_imu_camera = T_imu_lidar * inverse(T_camera_lidar)`，不增加TF。源相机帧是右下前光学坐标，所有平移单位米。未传calibration_file仍使用原安装初值。接受只代表人工空间标定验收，时间偏移未估计，不能填入lidar_time_delay；导航未接入此结果。

## 6. 旧bag导出与文件说明

```bash
~/r3live_ws/scout_calibrate.sh export imported_00 /absolute/path/static.bag
```

同一时段分别录制的雷达和相机bag也可作为两个位置参数传入。默认取首个雷达时间起6秒，允许`--seconds 3..10`。必须确认车和场景静止；雷达必须是driver2 CustomMsg，图像、CameraInfo、IMU、相机内部tf_static必须齐全且源时间重叠。导出不会使用SLAM点云。

每场景保存`raw.bag`（capture时）、record.log、capture.yaml、scene.yaml、image.bmp和cloud.pcd。导出过滤非有限点、非法tag/line、0.5m以内和20m以外点；按初始相机视野向四周扩展半幅裁剪，去掉相机后方点，再以2cm体素保留代表点。最多250万输入有效点；初值严重错误会导致裁剪错误，应先修安装初值。PCD仍是源雷达坐标。

每run保存训练数据副本、manifest.yaml、edges.yaml、multi_calib.yaml、solver/master日志、阶段投影图、extrinsic.txt、result.yaml、validation/和可选accepted_calibration.yaml。默认不删除或覆盖旧结果，不向Git提交bag、地图或运行图像。

## 7. 开发和检查

| 文件 | 责任 |
|---|---|
| install_calibration.sh | 固定版本、应用补丁、Ceres依赖、两workspace构建和部署 |
| scout_calibrate.sh | 加载匹配OpenCV的ROS环境，统一CLI |
| calibrate.py | sensors/capture/export/prepare/solve/project/accept；无速度/TF发布 |
| scout-calibration.patch | 上游CMake仅编译多场景工具；无GUI批处理；匹配/Ceres失败检查；输出精度；修复多场景粗匹配计分和k3畸变遗漏、K近邻不足检查 |
| ../scout_r3live_bringup/scripts/calibration_io.py | 刚体矩阵、相机一致性、接受文件校验及坐标转换 |
| ../scout_r3live_bringup/scripts/session.py、launch/scout_r3live.launch | 新增显式calibration_file，保留旧默认行为和冲突检查 |
| test_geometry.py | 逆变换/杆臂、坏矩阵、相机变更、未接受结果、PCD和路径约束测试 |
| test_synthetic_scene.py | 生成已知单位外参的房间多平面合成数据；绝不是本车实际外参 |
| test_bag_rejection.py | 用短时实录bag的临时副本验证运动、缺少内参和点数损坏拒绝；不发布数据 |

若求解因场景不足失败，先查看原图、初始投影和边缘提取日志，再补采有不同方向立体边缘的场景，不降低检查门槛来强行得到结果。程序没有车辆控制权；任何异常都需要现场人员自行停车。
