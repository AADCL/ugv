# WheelTech 专用 explore_lite 分支

来源：<https://github.com/hrnr/m-explore/tree/melodic-devel/explore>，包内版本 2.1.4。2026-09-11 获取上游源码，保留原版权和 BSD 许可证。

本地上游原始归档：`diagnostics/nx123_exploration/upstream_m_explore.tar.gz`；SHA256：`8f5e015c8746b5611ca0d1e073497a9078e8288cd34e99ac76117d83df5fc416`。该分支下载未固定 Git 提交，因此以归档校验值记录本次源码基线。

使用独立包 `wheeltec_explore_lite`、程序 `wheeltec_explore`。不替换 `/opt/ros/noetic` 安装的 `explore_lite`，不修改 GlobalPlanner 或 TEB 源码。

## 修正内容

项目统一入口为 `roslaunch wheeltec_exploration wheeltec_exploration.launch map_name:=NAME`。上游示例 `explore.launch` / `explore_costmap.launch` 已移除，`doc/wiki_doc.txt` 仅保留为上游历史参考，不作为本车启动说明。

1. 保留上游 BFS 前沿搜索、前沿长度/距离评分。补齐前沿种子点：上游把它计入点数，却没有计入点集合和质心总和。
2. 目标从真实前沿附近0.75 m内的已知区域选择，要求整车和前向净空均有通行空间。前沿之间仍按原评分排序，同一前沿内按距质心距离加 `0.15 × 转向角绝对值` 排序，优先较直接的接近方向。
3. 目标距车至少0.40 m，且不小于当前 TEB 的 `xy_goal_tolerance + 地图分辨率`，不修改 TEB 容差。每轮最多查询12个分离候选点；通过原 `/move_base/make_plan` 后，继续使用原始全局图检查整车路径、原始局部图检查近处0.75 m的路径。全局不穿越未知。
4. 活跃目标保持到完成、失败或无进展超时，不因地图细微变化连续抢占。进展按到真实目标距离每减少 2 cm 计量；失败目标仍使用上游黑名单。
5. 目标朝向使用全局路径末段的方向。`~preview_only:=true` 只发布 `/explore/selected_goal` 和调用路径查询，不向 move_base 发送或取消行动目标。
6. 只取消本节点拥有的目标，不使用上游 `cancelAllGoals()`。

前沿用于评价信息收益，车辆目标落在已观测一侧；路径可行不等于实物安全确认。程序不把未知地面伪造为自由区域，不绕过实时障碍检查、速度安全门和底盘超时停车。地图太小、候选点不满足条件或无有效路径时继续定时等待地图更新，不将此情况当作全屋探索完成。
