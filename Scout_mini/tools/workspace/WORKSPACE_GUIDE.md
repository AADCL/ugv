# Scout 120 工作空间整理（2026-09-11）

统一物理目录为 `/home/nvidia/livox_fastlio`。FAST-LIO 导航和 R³LIVE 仍独立启动、独立编译。此次仅整理路径和测试数据，导航参数、车身参数、TF 和已选外参保持原值。

| 目录 | 用途 |
|---|---|
| `livox_fastlio/src、build、devel` | FAST-LIO、底盘、建图、NDT、导航、影子融合 |
| `livox_fastlio/optional/r3live_ws` | R³LIVE，及配套 OpenCV 4.5 相机、cv_bridge、图像插件 |
| `livox_fastlio/optional/realsense_ws` | 原导航相机环境 |
| `livox_fastlio/optional/lidar_camera_calib_ws` | 雷达—相机标定 C++ 工具 |
| `livox_fastlio/optional/r3live_ws/calibration_tools` | 采集、提取、优化、投影及验收脚本 |
| `livox_fastlio/optional/r3live_ws/config/accepted_20260911` | 当前外参、来源候选、原配置备份和预期运行外参 |
| `livox_fastlio/maps` | 正式导航地图，保留 |
| `livox_fastlio/maintenance` | 清理清单、迁移日志、配置哈希 |
| `github_upload/ugv/Scout_mini` | Git 源码及三大手册维护入口 |
| `ccs_edge_ws` | 用户要求完全不动 |

旧 `~/r3live_ws`、`~/realsense_ws`、`~/lidar_camera_calib_ws` 是兼容软链接，不是额外副本。已有 catkin 环境、CMake 缓存、动态库及脚本引用旧绝对路径，必须保留链接。`optional` 在导航 `src` 外，导航编译不会合并这些独立环境；禁止混用两套 OpenCV 相机组件。

## 当前入口

```bash
~/livox_fastlio/optional/r3live_ws/start_scout_r3live_test.sh
~/livox_fastlio/optional/r3live_ws/view_scout_r3live.sh ~/Desktop/Scout_R3LIVE.rviz
~/livox_fastlio/optional/r3live_ws/scout_calibrate.sh --help
```

试用入口读取 `config/accepted_20260911/trial_calibration.yaml`，新数据写入 `logs/indoor_tests/<会话>/`。自动健康检查、录包和600秒上限保留。旧 `calibration/trials/indoor_20260911_01/start.sh` 已删除。六场景外参仍仅获室内试用授权，时间偏移未标定，独立精度验收未通过。

已删除旧 bag、`.bag.active`、对应回收站文件、标定场景/优化输出/画廊、室内试用会话、导航/融合测试日志、合成数据、FAST-LIO 旧 `PCD/scans.pcd`。正式地图、选定外参及配置备份保留。设备旧数据不能再回放，本次未删除 Windows 本地副本。重新标定需采集新场景。现有功能包仍被启动链引用，予以保留；没有凭 test/fusion 名称删除运行依赖。

## 修改、编译与验证

修改文件 `optional/r3live/start_scout_r3live_test.sh`：更换配置/输出目录；`install_scout_r3live.sh` 在目标配置目录不存在时复制仓库保留的试用外参，不覆盖设备既有选择；`calibration/prepare_indoor_trial.py` 对已删除的历史研究给出明确提示。外参文件保存在 `optional/r3live/config/accepted_20260911/`，恢复用完整旧配置备份另保留在设备同名目录和本地 `scout_cleanup_retained_config.tar.gz`。这些 shell/Python/配置改动无需 catkin 编译。后续 C++ 修改从兼容旧路径编译，避免与 CMake 缓存冲突：

```bash
# R³LIVE：独立新终端
cd ~/r3live_ws
source /opt/ros/noetic/setup.bash
source ~/realsense_ws/devel/setup.bash
source ~/livox_fastlio/devel/setup.bash
source ./devel/setup.bash
catkin_make -j1 -DCMAKE_BUILD_TYPE=Release -DR3LIVE_BUILD_MESHING=OFF
python3 ./check_opencv_runtime.py "$PWD"
# 标定 C++：另开终端 source ~/r3live_ws/devel/setup.bash，
# 再 cd ~/lidar_camera_calib_ws，执行 catkin_make -j1 -DCMAKE_BUILD_TYPE=Release
```

`audit_workspaces.py --output <新文件.json>` 只读核对包、源码和上游修改。`organize_workspaces.py` 是本设备此次专用迁移工具：无参数预览，`--apply` 检查ROS及文件占用，复制并哈希核验外参备份后永久清理，最后同盘迁移并建兼容链接。迁移后重复执行会拒绝已有链接，不是日常清理命令。切勿重复运行来清除新采集数据。

`maintenance/cleanup_20260911_175821/plan.json` 与 `journal.jsonl` 保存具体删除目标及迁移记录。删除不提供撤销，只能从其他既有副本恢复数据。源码和配置可从 Git/保留备份恢复。迁回目录需先停ROS，核对后只移除旧目录的软链接，再将对应空间迁回；不得递归删除工作空间。

本次验证范围：入口shell语法、分环境包发现、OpenCV依赖、已选外参哈希；不启动ROS硬件。后续实车运行效果另行测试。
