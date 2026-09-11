# R³LIVE 配套操作入口测试（2026-09-11）

120设备，Ubuntu20.04/ROS Noetic/ARM64。`scout_system_bringup`和`scout_r3live_bringup`均以`catkin_make -j1 --pkg <包名>`完成配置/构建，shell和Python语法检查通过。没有修改FAST-LIO、车体或导航算法参数。

`test_operation_launches.py`三项综合测试通过：

| 测试 | 覆盖内容 | 结果 |
|---|---|---|
| 功能组合 | local/mapping/localization/navigation四种pipeline的节点唯一；不包含FAST-LIO、第二套TF或pose adapter；底盘pub_tf=false、/scout/odom | 通过 |
| 导航一致性 | R³LIVE导航pipeline的全部/move_base参数与同地图的原navigation_teb.launch逐项比较 | 完全一致 |
| 数据保护及模式 | mapping拒绝非空目录；已有.finalization_incomplete拒绝定位；缺失地形层拒绝导航；test不能duration=0，操作模式不能关闭工程接口 | 通过 |
| 四个公开入口 | 在11339私有ROS master逐个执行scout_system_bringup中的四个launch，check_only=true | 全部PREFLIGHT_OK，自动加载R³LIVE环境，没有启动雷达、底盘或move_base |

测试使用indor地图的临时副本，未修改正式地图。测试无bag录制、无传感器/底盘/NDT/导航实跑。由此可确认入口、配置和预检逻辑，不能据此宣称全链路实车导航或大地图关机保存已验收。

实现中session在局部估计通过健康检查后才启动operation_pipeline，退出顺序相反；pipeline的roslaunch退出宽限为60秒，session等待80秒，超时标记失败。建图时建议外层`roslaunch --sigint-timeout=90 ...`，停车退出并等待保存结束，再单独finalize。没有在清理/预检时执行真实硬件启动。

原始测试及构建日志保存在120的`~/livox_fastlio/maintenance/`，对应`r3live_operation_launch_*_20260911.log`。复现脚本默认读取indor，其他设备可用SCOUT_TEST_MAP_DIRECTORY指定完整地图包。
