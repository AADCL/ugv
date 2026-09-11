# WheelTech 前沿探索

本包复用原点云积累、实时局部感知和 GlobalPlanner + TEB。独立 `wheeltec_explore_lite` 负责前沿目标选择；不使用 FLARE，只有一个 move_base。

探索时不再周期调用 `finalize_map.py`。局部 ObstacleLayer 的未膨胀观测以约 2 Hz 写入轻量记忆图 `/nav_static_map`，只保留未知、空闲和障碍，用于选前沿与全局连接；TEB 仍使用实时局部 costmap。未观测格不会因窗口移走而清除，也不把代价膨胀存成障碍。在线不生成 PGM/2.5D；坡面实时识别仍由原 Patchwork++ / Terrain Guard 完成，在线不叠加离线坡度代价。

原贝叶斯静态 PCD 在内存继续积累，探索关闭周期存盘，正常结束时保存 PCD/轨迹。随后人工执行原转换命令，生成原格式的 PCD、PGM 和 2.5D 成果。普通手动建图仍每 30 s 自动存盘，普通导航仍加载原 PGM/坡度层。

唯一公开入口：

```bash
source ~/livox_fastlio/devel/setup.bash
roslaunch wheeltec_exploration wheeltec_exploration.launch map_name:=room_explore_01
```

默认 `auto_explore=true`、`auto_start=true`。启动前告知现场人员；地图 ready、传感器及导航就绪后自动申请一次安全许可并开始探索，无需手动服务调用。Ctrl+C 或确认完成后停车退出整套 launch，最后人工 `rosrun wheeltec_map_tools finalize_map.py NAME`。需要仅观察时显式添加 `auto_start:=false`。

完成要求：有效地图/TF 持续可用、无活动目标、连续 30 s 无符合 0.50 m 阈值的可达前沿，期间至少两次新的观测图成功更新。数据失效、故障及仍有前沿却无路径不能记为完成。安全锁停后保留诊断与点云积累，等待现场处理，不自动 reset/rearm。内部 `launch/include/frontier.launch.xml` 不作为单独入口。

日常操作、开发和接口参数统一维护于三大文档：NX 上为 `~/livox_fastlio/docs/`，源码仓库为 `WheelTech/docs/`，均为 V5.1。地图保存仍用 `rosrun wheeltec_map_tools finalize_map.py NAME`。不要同时启动独立建图、重定位或导航入口。

探索专用参数在 `config/exploration_overrides.yaml`，不修改普通 TEB 参数。包内只保留当前前沿探索、观测记忆和TEB适配，旧在线高程建图及整图刷新分支已移除。最近的编译与录包验证范围见三大文档；完整房间实车覆盖与自动结束仍需验收。
