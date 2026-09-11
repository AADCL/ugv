# 轮趣 V5.1 文档索引

更新：2026-09-11。对应 `.123` 当前代码及本次功能入口整理。

本分支的发布状态、改动范围和已完成验证见 [轻量探索试验分支说明](轻量探索试验分支说明_V5.1.md)。

| 文档 | 用途 |
| --- | --- |
| [使用文档](轮趣四轮差速机器人_使用文档_V5.1.md) | 相机、建图、重定位、导航、日志、探索的启动和停止，地图保存 |
| [开发实施文档](轮趣四轮差速机器人_开发实施文档_V5.1.md) | 依赖安装、复制源码、修改、编译及测试 |
| [详细信息表](轮趣四轮差速机器人_详细信息表_V5.1.md) | 节点、话题、TF、服务、安全限制和配置文件 |

三大文档已集成探索，日常只使用其中列出的 6 个功能 launch。内部 `launch/include/*.launch.xml` 由公开入口调用。探索 launch 默认在地图和传感器就绪后自动开始，Ctrl+C 或确认完成后停车退出，最终仍人工运行 `rosrun wheeltec_map_tools finalize_map.py NAME`。只观察不运动时显式加 `auto_start:=false`；普通导航与重定位继续分开启动。

NX 文档目录为 `~/livox_fastlio/docs/`；源码目录为 `~/livox_fastlio/src/`。Windows 对应 `WheelTech/docs/` 和 `WheelTech/wheeltec_stack/`。V4.3 车端旧文档已归档到开发电脑的本地备份，不再作为操作说明。

NX 工作空间根目录 `README.md` 提供目录索引，`launch/` 集中六个正式入口的相对符号链接。原包名启动命令保留。历史V4.5文档、旧上传README和旧文档生成器已从本地项目主目录移至 `diagnostics/nx123_workspace_cleanup/archive/local/`，供追溯使用。
