# Scout R³LIVE 独立试验

2026-09-11：120物理工作空间已迁入 `~/livox_fastlio/optional/r3live_ws`，旧 `~/r3live_ws` 为兼容链接。试用外参保存在 `config/accepted_20260911/`，运行 `start_scout_r3live_test.sh`；新测试数据在 `logs/indoor_tests/`。历史bag和标定导出已清理，配置备份保留。参见[目录指南](../../tools/workspace/WORKSPACE_GUIDE.md)。

官方R³LIVE（不是R³LIVE++），Mid-360＋D435i彩色流，Ubuntu20.04/Noetic/Jetson ARM64。

工程接口默认开启（`project_interface:=true`）：保留全部`/r3live/*`原始输出，并提供`/Odometry`、`/fastlio_odom`、三种`/cloud_registered*`点云和现有odom→camera_init→body→base_link链。原始动态TF隔离至`/r3live/tf_raw`，相机安装TF使用本次实际外参。首帧车体在odom下归零；适配不增加点云降采样。切回纯独立观察用`project_interface:=false`。

原独立test入口不启动NDT、底盘、轮速融合、Bayesian mapper或导航。新增四个持续运行入口，直接在原Scout环境使用（四选一）：

```bash
roslaunch scout_system_bringup scout_r3live_local.launch
roslaunch --sigint-timeout=90 scout_system_bringup scout_r3live_mapping.launch map_name:=r3live_map_01
roslaunch scout_system_bringup scout_r3live_localization.launch map_name:=indor
roslaunch scout_system_bringup scout_r3live_navigation.launch map_name:=indor
```

四入口自动加载R³LIVE环境和已选外参，默认带底盘、不录bag，持续到人工退出。mapping加Bayesian mapper；localization加NDT及地图；navigation包括完整定位和原TEB导航，不再另开第二套。建图退出保存后执行`rosrun scout_map_tools finalize_map.py r3live_map_01`。`check_only:=true`可先做无硬件预检。导航参数沿用原工程。

公共里程计仍为pose-only；`/fastlio_odom`是历史兼容名称，当前数据来自R³LIVE。完整话题/TF表、增量编译与转换公式见三大手册；[合成验证报告](TEST_REPORT_PROJECT_INTERFACE_20260911.md)不代表实车精度验收。

```bash
cd ~/github_upload/ugv
bash Scout_mini/optional/r3live/install_scout_r3live.sh
~/r3live_ws/start_scout_r3live.sh
```

运行前停车并退出原FAST-LIO/融合/导航入口；新入口检查冲突，不强制接管。只做安装检查用`check_only:=true`。本入口不启动底盘，不发送速度，不启动NDT或导航。空间/时间标定尚未完成，不以启动成功代替精度验收。

- [操作流程](../../docs/Scout_Mini_使用文档_V5.1.md)：独立入口、日志、退出、RViz。
- [开发实施](../../docs/Scout_Mini_开发实施文档_V5.1.md)：逐文件修改、版本、依赖、编译和测试。
- [详细信息](../../docs/Scout_Mini_详细信息表_V5.1.md)：launch、节点、话题、TF、参数与排错。

`scout-r3live.patch`只用于固定上游提交，保留原上游许可。`install_scout_r3live.sh`不覆盖已有不同版本的脏源码，不替换系统OpenCV/cv_bridge；默认关闭可选CGAL网格重建。新工作空间不放入主工程src，避免catkin包重名和覆盖原cv_bridge环境。

2026-09-11起，相机驱动及compressed/compressedDepth/theora插件也在本overlay按固定版本重编；`check_opencv_runtime.py ~/r3live_ws`必须全部通过。旧相机库混用OpenCV4.2/4.5会在图像转换时崩溃，仅验证mapping链接不足以证明传感器链路正常。六场景loose候选室内试用入口、备份和回退见使用文档“已选外参的室内试用”。

`config/rig.yaml`的外参是安装尺寸初值。相机内参/内部光学TF启动时从设备读取。`/r3live/odometry`是原始世界系IMU位姿；通过工程适配节点转换后才输出`/fastlio_odom`与工程TF，不能只重命名原始话题。

新增[Scout端相机—雷达标定工具](calibration/README.md)：静态采集、driver2导出、多场景无GUI求解、独立投影验证和显式接受。设备入口`~/r3live_ws/scout_calibrate.sh`。得到接受文件后，通过`calibration_file:=...`给独立R³LIVE使用；不会自动替换安装初值，不估计时间偏移。

测试使用`~/r3live_ws/start_scout_r3live_test.sh`：加载已选六场景外参，自动启动驱动、时间检查、估计器、接口适配和bag记录；看到`READY`再人工低速运动。READY后默认10分钟自动结束，`test_duration:=60`可改测试时长（30～1800秒）。停车Ctrl+C可提前结束，终端异常时人工停车；程序没有底盘控制权。日志与bag统一在logs/indoor_tests本次会话目录。发现时钟/断流/非法位姿后锁定失效、退出自己的进程，不自动复位重启。旧trial目录已经清理。
