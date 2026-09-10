# R³LIVE Scout 编译与功能测试，2026-09-10

设备：192.168.50.120，Jetson Orin NX ARM64，Ubuntu20.04/ROS Noetic。软件版本与补丁见开发文档及安装脚本。运行测试使用私有ROS master `http://127.0.0.1:11431`；原11311的FAST-LIO、底盘与旁路融合继续运行，没有改导航参数、发车辆运动指令或关闭原测试。

| 检查 | 结果/证据 |
|---|---|
| 串行编译 | `catkin_make -j1 -DR3LIVE_BUILD_MESHING=OFF`通过，含r3live_mapping、前端、cv_bridge/image_geometry与启动包 |
| 运行库 | source独立overlay后，cv_bridge来自r3live_ws/devel/lib；OpenCV4.5，无4.2混载或缺失库 |
| Mid-360合成转换 | 120个后向点保留；NaN、球形盲区、非法line/tag剔除；末点11.9ms；源stamp一致；point_num不匹配拒绝 |
| D435i | 实机rgb8，640×480；fx603.01416、fy603.14185、cx320.65094、cy235.57579；读取到彩色光学TF；外参仍是安装初值 |
| 原master冲突检查 | 检出laserMapping、Livox、fusion_guard、shadow_ekf，拒绝启动，无接管 |
| 私有master预检查 | PREFLIGHT_OK，不启动传感器或估计器 |
| 旧走廊LIO回放 | `/home/nvidia/2026-09-10-14-36-01.bag`前45秒，只播放雷达和IMU，422帧有效里程计、源时间跨度42.10s |
| 最终独立入口联合回放 | 同时段采集的20秒雷达/IMU和彩色图像；178帧里程计、264帧相机位姿、264帧跟踪图像；输出跨度17.70s，启动等待消耗部分输入 |
| 输出合同 | 有限位姿、单位四元数、时间单调、独立帧和路径头`r3live_world`通过 |
| 视觉分支 | 日志`R3LIVE visual: tracked=105…130 geometric=1 photometric=1`；返回状态证明上游更新函数运行，不能代替真实轨迹误差测量 |

修复了编译/测试发现的ARM SSE和CPUID依赖、driver消息版本、OpenCV ABI环境、相机包及nodelet插件查找、路径头残留旧world帧问题。上游广告的track_points/render_points默认没有调用发布函数，不以其无消息判定VIO失效；新增低频文字日志观察视觉更新，不新增诊断话题。

最终运行日志：`~/r3live_ws/logs/20260910_195307_ai5jz5f8/`，含实际runtime.yaml、rig/sensor快照、estimator.log。汇总、构建与检查日志、两份约333MB原始测试bag保留于`~/r3live_ws/logs/installation_20260910/`；不纳入Git。相机bag记录CameraInfo和相机内部tf_static，雷达bag只记录CustomMsg/IMU，不含速度指令。

重放本次入口测试时，先确认11431空闲，然后在独立终端启动私有master：

```bash
source ~/r3live_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11431
roscore -p 11431
```

另一个终端设置同一master并启动入口：

```bash
source ~/r3live_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11431
rosparam set /use_sim_time true
~/r3live_ws/start_scout_r3live.sh start_lidar:=false start_camera:=false
```

在同一私有master的另外两个终端先启动观察器再回放（不要漏掉ROS_MASTER_URI）：

```bash
python3 ~/github_upload/ugv/Scout_mini/optional/r3live/test_observe.py \
  --duration 40 --require-vision --output /tmp/r3live_retest_result.json
```

```bash
rosbag play \
  ~/r3live_ws/logs/installation_20260910/scout_r3live_live_lidar.bag \
  ~/r3live_ws/logs/installation_20260910/scout_r3live_live_camera.bag \
  --clock --delay=3 --topics /livox/lidar /livox/imu \
  /r3live_camera/color/image_raw /r3live_camera/color/camera_info /tf_static
```

测试结束Ctrl+C退出入口和私有master。不要对着原11311回放这批输入，也不要同时启动两个定位入口。最初手工后台回放因终端stdin挂起，重启为`</dev/null`后正常；最终表格取重新运行的完整结果。原走廊bag没有相机，不能用它验证视觉。

未验证：精准空间/时间标定、动态车辆与长走廊精度、暗光扬尘鲁棒性、长期实时性/内存、相机失效时的安全降级、NDT/导航集成、现有Bayesian地图保存链路兼容性。当前不会据此替换正式定位导航。
