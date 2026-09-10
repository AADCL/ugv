# BATF-Nav V5.1 统一技术说明

完整技术名称：`BATF-NavBayesianAccumulationTerrainFusionNavigation`。

简称：**BATF-Nav**。英文展开：**Bayesian Accumulation Terrain Fusion Navigation**。中文释义：贝叶斯累积与地形融合导航。

2026-09-10起，本仓库Scout Mini与WheelTech的V5.1技术方案统一使用此名称。名称指现有系统集成方案，不表示本次新增算法、提高定位精度或改变上游组件名称。两车继续使用独立的硬件适配、参数和验证记录。

## 技术组成

| 名称组成 | V5.1对应功能 |
|---|---|
| Bayesian Accumulation | 配准点云的可逆贝叶斯静态证据累积，结合命中和自由射线证据抑制动态残影，保留细尺度地图几何 |
| Terrain Fusion | 离线鲁棒地面重建、PGM占据地图与2.5D高程/坡度表达；导航使用保存坡度代价及当前帧地面/障碍分类 |
| Navigation | FAST-LIO局部里程计、已有地图NDT重定位、GlobalPlanner/TEB及车型独立的底盘控制和安全配置 |

```text
建图：Livox → FAST-LIO配准 → 贝叶斯静态累积＋车体轨迹 → 离线地面重建 → PCD / PGM / 2.5D
定位：FAST-LIO局部运动估计＋已有地图NDT修正 → 全局位姿
导航：PGM静态占据＋保存坡度代价＋当前帧地形障碍 → GlobalPlanner / TEB → 底盘
```

这里的Terrain Fusion是地图及地形信息在系统中的组合使用，不意味着所有信息进入一个概率滤波器。Scout新增的轮速/FAST-LIO旁路EKF属于Scout扩展，不能由统一命名推断WheelTech已部署同一功能。当前已知的重复走廊定位退化、NDT错误修正及动态测试限制也不会因为命名而消失。

## 平台与详细文档

| 平台 | 入口 | 详细文档 |
|---|---|---|
| Scout Mini | [项目README](../Scout_mini/README.md) | [使用](../Scout_mini/docs/Scout_Mini_使用文档_V5.1.md) · [开发实施](../Scout_mini/docs/Scout_Mini_开发实施文档_V5.1.md) · [详细信息表](../Scout_mini/docs/Scout_Mini_详细信息表_V5.1.md) |
| WheelTech | [项目README](../WheelTech/README.md) | [使用](../WheelTech/docs/轮趣四轮差速机器人_使用文档_V5.1.md) · [开发实施](../WheelTech/docs/轮趣四轮差速机器人_开发实施文档_V5.1.md) · [详细信息表](../WheelTech/docs/轮趣四轮差速机器人_详细信息表_V5.1.md) · [技术链路](../WheelTech/docs/轮趣四轮差速机器人_点云处理与导航链路_V5.1.md) |

## 使用名称的约定

- 对外展示使用“BATF-Nav V5.1”，需要区分车型时使用“BATF-Nav V5.1 · Scout Mini”或“BATF-Nav V5.1 · WheelTech”。
- 完整名称按上述连续字符串书写，英文说明使用带空格的展开名称。
- ROS功能包、节点、话题、TF帧、launch命令和已有文件名保持不变，不因技术命名而改动接口。
- 原有V5.1文档保留文件名及完整开发、编译和操作步骤。上游组件的名称、版权和许可证保持各自含义。

[返回仓库总说明](../README.md)
