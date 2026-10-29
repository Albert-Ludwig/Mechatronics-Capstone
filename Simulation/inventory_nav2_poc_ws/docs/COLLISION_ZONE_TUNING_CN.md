# Nav2 减速和停车区域调整（0.6.1）

2026-10-28，本机 ROS 2 Lyrical / Gazebo Sim 10。

## 哪些属于 Nav2

减速圈和停车圈由官方 `nav2_collision_monitor` 实现。两个圆形区域是本项目
在 `config/nav2_overrides.yaml` 中追加的配置，不是官方默认区域。
本机官方配置默认只有 `FootprintApproach`，时间阈值为 1.2 秒。
本项目此前为独立行人设置了更保守的区域与 2 秒预判。

机器人运动仍由官方 MPPI、速度平滑器、碰撞监测器控制；没有增加自行控制
机器人速度的避障节点，行人没有机器人位置输入，也不主动停让。
参数含义见 [Nav2 Collision Monitor 官方说明](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/)。

## 当前调整

| 参数 | 0.6.0 | 当前 |
|---|---:|---:|
| 减速圈半径 | 1.20 米 | 1.15 米 |
| 减速输出比例 | 50% | 50% |
| 停车圈半径 | 0.65 米 | 0.60 米 |
| FootprintApproach 预判时间 | 2.0 秒 | 2.0 秒 |

半径从 `base_footprint` 参考点测量，不是从车身表面测量。
0.5×0.5 米车身加 0.025 米 padding 后的外接半径约 0.389 米。
实际减速/停车由圈内有效雷达点触发，保持 `min_points=2`。
机器人前进上限仍为 1 m/s，行人仍为 0.6 m/s。

较大幅度收小保护区域的试验中，小车在横穿路线时进入行人前方，然后急停，
行人继续运动导致接触。因此最终仅小幅缩小两个圆，保持原有 MPPI 参数、
减速比例和 2 秒方向相关预判。额外的控制器权重试验也已撤回。

## 未通过的调整

以下运行确实失败，不能当作通过记录；接触后任务立即取消，未发生顶飞。

| 方案 | 减速半径 / 比例 | 停车半径 | 预判时间 | 结果 |
|---|---|---|---|---|
| 第一组 | 0.90 米 / 70% | 0.50 米 | 1.2 秒 | A→B 交会接触 |
| 第二组 | 1.05 米 / 60% | 0.60 米 | 2.0 秒 | A→B 交会接触 |
| 第三组 | 1.05 米 / 60% | 0.60 米 | 2.0 秒 | 加强 MPPI 近碰惩罚、减弱路径对齐后仍接触 |

前两组未调整 MPPI 近碰阈值和路径对齐权重。
证据分别保存在 [第一组失败摘要](runtime/nav2-v0.6.1-aggressive-zones-failure.json)
与 [第二组失败摘要](runtime/nav2-v0.6.1-zones-only-failure.json)。
对应原始日志目录为 `/tmp/nav2-mission-kbcnrjgr` 和 `/tmp/nav2-mission-0dwl5yas`。
第三组曾将 MPPI `CostCritic.near_collision_cost` 从 253 改为 150、
`critical_cost` 从 300 改为 1000、`PathAlignCritic.cost_weight` 从 14 改为 7，
仍未通过，见 [第三组失败摘要](runtime/nav2-v0.6.1-mppi-tuning-failure.json)。
原始目录 `/tmp/nav2-mission-8agmv2v0`。这些值均不在最终配置中。

## 最终运行验证

第一轮新场景已完成完整任务，见
[隔离运行摘要](runtime/nav2-v0.6.1-smaller-zones-mission.json)。
原始日志目录 `/tmp/nav2-mission-uihy_0df`。

| 观测项 | 第一轮结果 |
|---|---|
| Home → A → B → C → Home | 完成，`acceptance_passed=true` |
| 接触事件 | 0；8/8 个接触发布端已发现 |
| 任务仿真时间 | 111.556 秒 |
| A/B/C 连续静止停留 | 5.406 / 5.372 / 5.202 秒 |
| 机器人实际最大速度 | 1.000 m/s |
| 行人实际最大速度 | 0.600 m/s |
| 机器人参考坐标最高 z | 0.01000714 米 |
| 最大倾斜 | 0.00004034 弧度 |
| 最近人车参考点距离 | 1.17037 米，不是表面间隙 |
| Nav2 地图动态障碍观测 | 50 次 |
| 回 Home 定位误差 | 0.04575 米 / 0.16366 弧度 |

0.6.0 基线单轮任务为 182.862 仿真秒；此轮为 111.556 秒。
控制器采样、启动时序和行人交会相位会改变路线与等待时间，不能据此宣称固定提速比例。
31 项 unittest、colcon 构建、Python 编译和 Git 差异检查通过。

第二轮从新启动的桌面场景开始，同样通过完整验收，见
[桌面复测摘要](runtime/nav2-v0.6.1-desktop-repeat.json)。
原始观测目录 `/tmp/nav2-mission-p78fajmo`；任务报告与轨迹在
`~/inventory_poc_results/20261029T000001_094990Z/`（目录使用 UTC 时间）。
复测窗口与服务器一起启动，观测工具以 `--attach` 只读采集，
结束后保留桌面场景；未临时修改参数，也不发布速度。

| 观测项 | 第二轮结果 |
|---|---|
| 完整任务与总验收 | 完成，`acceptance_passed=true` |
| 接触事件 | 0；8/8 接触发布端已发现 |
| 任务仿真时间 | 121.860 秒 |
| A/B/C 连续静止停留 | 5.202 / 5.406 / 5.406 秒 |
| 机器人 / 行人最高速度 | 1.000 / 0.600 m/s |
| 机器人最高 z / 最大倾斜 | 0.01000714 米 / 0.00004034 弧度 |
| 最近人车参考点距离 | 1.06794 米，不是表面间隙 |
| Nav2 地图动态障碍观测 | 43 次 |
| 回 Home 定位误差 | 0.06340 米 / 0.13934 弧度 |

直接读取运行中的 Nav2 参数，确认 `StopCircle.radius=0.60`、
`SlowCircle.radius=1.15`、`slowdown_ratio=0.5`、`time_before_collision=2.0`，
`FollowPath.vx_max=1.0`，与最终文件一致。

复现命令及接触、物理稳定、停留和返航验收方法见
[完整测试说明](SMOOTH_PEDESTRIAN_TEST_CN.md)。最终配置已通过两次独立新场景，
尚未完成三次重复验收；这两次通过也不表示覆盖所有初始相位和交会方式。
