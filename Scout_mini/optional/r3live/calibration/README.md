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

## 8. 基于实测安装参数的约束求解与对照（2026-09-11）

本车实测对照与未通过验收的原因见[2026-09-11结果报告](RESULTS_20260911.md)。程序通过测试不等于本车外参通过验收。

当不同场景产生不同外参时，先用已有数据运行独立研究入口，不继续盲目增加场景，也不把原版project的深度/法向边缘指标当作精度。先更新仓库并重新运行本文件安装命令。此入口使用SciPy（端侧1.3兼容）、NumPy、Python OpenCV及上述C++几何提取器，不修改原无约束solve，不发布TF或速度。

```bash
~/r3live_ws/scout_calibrate.sh refine prior_center_20260911 \
  --training scene_00 scene_01 scene_02 validation_02 \
  --validation validation_00 validation_03 \
  --mount-reference lidar-center-hypothesis
```

该名字为本次真实研究目录；复跑必须改名。`--root`放在refine之后，默认`~/r3live_ws/calibration`；`--port`默认11443。至少3组训练、1组独立验证，名字和点云内容必须互异。名称是历史记录，`validation_02`在该研究中明确成为训练，不能再作为独立验收；validation_00和validation_03不进入拟合。

可加`--compare-runs run_20260911_01 run_20260911_02`，在同一几何样本上比较旧两次求解。对照文件必须完整、hash未变、相机及坐标一致，且旧训练不能包含本次独立验证场景或其点云内容。最终完整研究名为`prior_robust_final_20260911`。固定全部几何样本，直接最小化其到可靠图像边缘的鲁棒Chamfer距离；最多500次求值，ftol/xtol/gtol均为1e-6，不收敛拒绝，不把达到迭代上限当成稳定。

测量原点问题：用户确认位置量到雷达外壳/几何中心，并非内部IMU。现有rig.yaml把`[0.25,0,0.20]`命名为base_to_imu，因此需要对照，不直接重写运行配置。`stored-imu`保留旧解释；`lidar-center-hypothesis`暂把测量中心近似为雷达原点。若旧矩阵为`T_camera_imu*T_imu_lidar`，新解释从其平移中减去`R_camera_lidar*t_imu_lidar`，旋转不变。外壳中心不保证等于厂家雷达坐标原点，实际偏差仍未知；两种解释均保留在研究报告。

`prior_refine.py`执行完整步骤：校验和快照→在独立master仅提取每场景真实平面交线及图像边缘→保留实测基线与旧IMU解释→三档小范围先验拟合→固定中档先验逐一移除训练场景重算→同一组独立几何样本对照。相机内参、坐标变换方向不变。求解采用固定样本到可靠图像边缘的最近欧氏距离，即鲁棒Chamfer目标；不在外循环丢弃或重新筛选点，也不采用上游vpnp中的方向残差。斜线垂距测试保留为上游方向残差审计依据，实际拟合使用边缘距离。

三档假设尺度为旋转0.5°/1°/2°，平移0.005/0.01/0.02m，分别作为各轴先验尺度，不是测量误差证书。旋转以相机系左乘增量表示，平移以相机系直接偏移表示。每轴限制±3倍尺度，最终旋转或平移模长达到约3倍尺度时拒绝；优化不收敛或任一训练场景匹配少于30也拒绝。拟合距离超过20px后保留常数代价以拒绝粗大误匹配，样本不删除；图像匹配门限20px、局部5邻点直线性比>4、像素尺度2px、Huber阈值2。每场景等权、全部数据总计200个等效观测权重，避免点数多的场景压倒先验。这些是公开可复现的调参约定，不是推导出的真实传感器协方差。

对比使用2cm采样、每场景最多2000个真实交线点；固定基线视野内样本，新候选移出视野的样本赋50px惩罚。输出中位数、P90、3/5px比例和可见数量，避免只看留下的少量好匹配。图中绿色为提取的图像边缘，红色为真实交线投影。此处仍存在遮挡、伪交线和错误对应风险，指标不能换算成本车定位精度，也不能直接与旧project近似边缘指标比较。

研究目录包含`manifest.yaml`、提取日志、`image/*_edges.csv`及`*_image_edges.csv`、`measured.yaml`、`legacy_imu_interpretation.yaml`（选择该假设时）、tight/medium/loose.yaml、comparison/下全场景图表和`study.yaml`，失败保留`.study_incomplete`。新模块不生成extrinsic.txt或accepted_calibration.yaml，原accept入口不能采用研究文件；保留人工审查和明确部署步骤。先看未参与拟合的场景是否一致改善，再看逐一移除训练场景的稳定性；先验约束本身限制了变化，因此“变化小”也不能单独证明准确。

新增开发文件：`scout-extraction.patch`仅为旧补丁后的C++程序增加环境变量`SCOUT_CALIB_EXTRACT_ONLY=1`提取退出分支；install依次幂等应用两份补丁；calibrate.py的solve支持内部extract_only分支；scout_calibrate.sh路由refine；`prior_refine.py`是完整研究实现；`test_prior_refine.py`验证斜线残差、坐标/杆臂、已知真值及坏匹配拒绝。修改C++后重新运行install串行编译`lidar_camera_multi_calib`；只改Python则部署完整脚本。测试方法：

```bash
source ~/lidar_camera_calib_ws/devel/setup.bash --extend
export PYTHONPATH=~/r3live_ws/src/scout_r3live_bringup/scripts:$PYTHONPATH
python3 ~/r3live_ws/calibration_tools/test_prior_refine.py
```
