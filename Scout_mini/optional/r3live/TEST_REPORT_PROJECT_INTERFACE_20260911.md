# Scout R³LIVE 工程接口验证（2026-09-11）

设备：120，Jetson ARM64、Ubuntu20.04、ROS Noetic。范围：只增加独立R³LIVE输出适配，未修改FAST-LIO输入、导航参数、车体几何数值或NDT算法。

## 结果

`catkin_make -j1 --pkg scout_r3live_bringup -DCMAKE_BUILD_TYPE=Release -DR3LIVE_BUILD_MESHING=OFF`成功，生成`project_interface_node`。session.py语法检查通过。`test_project_interface.py`在独占11329端口的私有ROS master执行成功（1项综合集成测试，4.148秒），仅启动适配与两份原静态TF节点，不启动传感器或底盘、不录bag。

| 验证项 | 输入与检查 | 结果 |
|---|---|---|
| 初始原点 | R³LIVE世界系平移[12,-7,2]m、roll0.05/pitch-0.12/yaw0.73rad | 首个base_link位姿在odom下为单位变换 |
| 坐标轴与杆臂 | 随后车体沿odom Y移动3m并yaw转90°，保留原45°安装倾角和[0.25,0,0.20]m杆臂 | 公共车体位姿和TF对应[0,3,0]，不是错误写入世界X |
| 配准扫描 | 同一组几何点分别转camera_init、body、base_link | 三路XYZ与独立构造预期一致，容差3e-6m仅为数值计算误差 |
| 数据保持 | 2行组织点云、intensity41/42、每行8字节padding | 点数/排列/字段/padding/原stamp全部保持，无适配层降采样 |
| 相机TF | 使用设备已选T_camera_lidar及非零合成camera_link→optical变换 | T_imu_camera推导和TF一致；合成工厂变换不写回设备标定 |
| TF所有权 | 监听主/tf与/tf_static，检查body、base_link、terrain_sensor、livox和r3live_imu父节点 | 每个child只有一个parent，无map→odom伪发布 |
| 原始TF隔离 | 解析estimator.launch两种project_interface参数 | true时mapping /tf→/r3live/tf_raw；false保持独立/tf |
| 时间同步 | 点云0.20s、位姿0.25s，故意不匹配 | 没有公共输出，不用最近位姿凑配 |
| 非法输入 | 同步帧但四元数全零 | 不发布新公共消息，适配节点required退出 |
| 未提供的观测 | 公共Odometry twist | 保持pose-only约定，方差对角1e6标为未知占位，非高置信零速度 |

本次测试不能证明外参准确、R³LIVE无漂移、动态地图已过滤、NDT重定位正常或实车导航安全。旧bag已按用户要求清理，未做真实bag回放。接口本身不修复连续漂移，也不估计轮速融合。真实测试仍须停车启动、等待READY，然后人工低速验证。

复现命令：加载`~/r3live_ws/devel/setup.bash`后运行仓库`Scout_mini/optional/r3live/test_project_interface.py`。脚本自行关闭私有master和测试launch；11329占用时拒绝启动，绝不向11311发布合成数据。
