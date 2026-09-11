# 轮趣四轮差速机器人 V5.1

> 当前为 `experiment/wheeltech-light-exploration-v5.1` 试验分支，完整实车探索仍待验收。改动范围、录包验证结果和恢复方法见 [试验分支说明](docs/轻量探索试验分支说明_V5.1.md)。

本项目在 Jetson NX、ROS Noetic 上实现 Livox Mid-360 / FAST-LIO2 建图、NDT 重定位、GlobalPlanner + TEB 导航及前沿自主探索。静态点云经贝叶斯积累与动态滤除后，由原地图工具生成 PCD、PGM 和 2.5D 地形文件；导航实时障碍由当前帧 Patchwork++ 与 Terrain Guard 提供。

当前整理及部署对象为 .123 轮趣，工作空间 `~/livox_fastlio`。探索复用原建图和规划组件，独立的 `wheeltec_explore_lite` 只选择目标，不使用 FLARE。普通导航参数与探索覆盖参数分开管理。

## 功能入口

每个终端先执行 `source ~/livox_fastlio/devel/setup.bash`。

| 功能 | 命令 |
| --- | --- |
| 相机（可选） | `roslaunch wheeltec_system_bringup wheeltec_orbbec336l.launch` |
| 遥控建图 | `roslaunch wheeltec_system_bringup wheeltec_mapping.launch map_name:=factory_a` |
| 已有地图重定位 | `roslaunch wheeltec_system_bringup wheeltec_localization.launch map_name:=factory_a` |
| 导航（先运行重定位） | `roslaunch wheeltec_navigation navigation_teb.launch map_name:=factory_a` |
| 导航/探索日志 | `roslaunch wheeltec_navigation nav_logging.launch tag:=factory_a` |
| 自主探索建图 | `roslaunch wheeltec_exploration wheeltec_exploration.launch map_name:=room_explore_01` |

探索入口包含建图、导航、探索目标选择及安全门，不能与独立建图、重定位或导航入口重复启动。启动 launch 后等待地图和传感器就绪，自动开始探索，无需手动服务调用。Ctrl+C 或持续无有效前沿确认完成后停车并退出，再人工运行原转换命令。启动前告知现场人员；只观察不运动时加 `auto_start:=false`。详细流程见使用文档。

结束建图或探索并正常退出后，沿用 `rosrun wheeltec_map_tools finalize_map.py NAME` 生成最终地图。项目子模块归入 `launch/include/*.launch.xml`，不作为日常功能入口；供应商驱动示例保留。

NX 上的六个入口集中在 `~/livox_fastlio/launch/`，均为指向上述正式文件的相对符号链接，不保存第二份参数或实现。例如也可以运行 `roslaunch ~/livox_fastlio/launch/wheeltec_exploration.launch map_name:=room_explore_01`。原有按包名启动的命令不变。

## 工作空间目录

```text
~/livox_fastlio/
├── README.md       项目与入口索引
├── launch/         6个正式入口的快捷链接
├── src/            项目包与传感器、底盘驱动源码
├── docs/           当前V5.1使用、开发、详细信息文档
├── maps/           地图与原始采集成果
├── logs/           测试和运行记录
├── vendor/         Livox SDK等编译依赖
├── build/          编译中间文件
└── devel/          可运行程序与setup.bash
```

本地项目源码为 `WheelTech/wheeltec_stack/`，车端对应 `src/` 内的项目包。探索在线只维护局部观测记忆图；正常结束保存PCD后再人工转图。旧在线高程建图、周期整图转换器、重复探索配置及对应旧程序已清理。地图、录包、驱动依赖和当前编译环境保留。2026-09-11本次清理前的源码、旧文档及被移除编译产物保存在开发电脑 `diagnostics/nx123_workspace_cleanup/`，NX不存备份。

## 文档与代码

- [使用文档 V5.1](docs/轮趣四轮差速机器人_使用文档_V5.1.md)：按功能启动、探索、停止和保存地图。
- [开发实施文档 V5.1](docs/轮趣四轮差速机器人_开发实施文档_V5.1.md)：依赖、源码复制、编译、修改和测试流程。
- [详细信息表 V5.1](docs/轮趣四轮差速机器人_详细信息表_V5.1.md)：节点、话题、TF、服务、参数与文件索引。

`wheeltec_stack/` 保存项目 ROS 包，依赖和车体外参见三大文档。探索已有室内短距离实车测试，完整房间覆盖与自动结束仍需验收。本次入口整理沿用 V5.1，不代表已同步到其他车辆或 GitHub。
