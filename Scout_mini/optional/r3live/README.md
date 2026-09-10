# Scout R³LIVE 独立试验

官方R³LIVE（不是R³LIVE++），Mid-360＋D435i彩色流，Ubuntu20.04/Noetic/Jetson ARM64。

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

`config/rig.yaml`的外参是安装尺寸初值。相机内参/内部光学TF启动时从设备读取。`/r3live/odometry`表示独立局部原点下的IMU位姿，不是车体位姿，不能直接替换原`/fastlio_odom`或导航TF。
