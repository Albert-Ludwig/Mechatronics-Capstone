# 可选行人场景验证（2026-10-28）

后续实际巡检已发现 0.3.0 会发生行人碰撞，原来的运动/雷达检查不足以证明安全。
该版本已由 0.3.1 修复，见 [碰撞修复验证](PEDESTRIAN_FIX_VERIFICATION.md)。
以下保留原始检查范围和限制，供追溯。

环境：本机 ROS 2 Lyrical / Gazebo Sim 10。行人功能标签为 `nav2-v0.3.0`。
本记录验证限界运动和雷达观测，不代表完整导航任务已通过。

## 已通过

- `colcon build --symlink-install --packages-select inventory_inspection_poc`。
- `check_environment` 检查。
- 25 项 unittest，包括端点掉头、长时间步长不越界、路线中段穿货架被拒绝、
  仓库边界检查、可选世界生成、接触话题数量及运动状态过期检查。
- 实际调用 launch 的两个分支：`pedestrian:=false` 使用原世界、不创建行人节点；
  `pedestrian:=true` 创建带行人的临时世界、服务桥及运动节点。
- 启动 Gazebo 和 Nav2，关闭自动任务与图形窗口，使用独立 ROS domain/Gazebo partition
  观测运动，测试后停止本次启动的进程。

联调启动参数：

```bash
ros2 launch inventory_inspection_poc inspection.launch.py \
  pedestrian:=true headless:=true use_rviz:=false run_mission:=false
```

独立位姿来自 Gazebo `/world/default/pose/info` 中名称为 `inspection_pedestrian` 的模型，
通过 `gz topic -e -t /world/default/pose/info --json-output` 读取。
没有用运动节点发布的目标位置代替 Gazebo 实际位置。

| 观测项 | 结果 |
|---|---|
| Gazebo 模型位姿样本 | 1012 |
| 实际 x 最小/最大值 | -4.0 / 2.8 米（浮点输出为 2.799999999999999） |
| 实际 y 最大绝对值 | 0 米 |
| 远端掉头 | 已观测到，回程离开最大 x 超过 0.4 米 |
| 雷达样本 | 91 |
| 机器人正前方 ±0.15 弧度的最近距离变化 | 约 1.48～8.29 米 |
| 成功移动服务状态样本 | 361 |

本次临时原始日志位于 `/tmp/nav2-pedestrian-integration/`，包含 `launch.log`、
`gazebo_poses.jsonl`、`report.json`。`/tmp` 可能被系统清理，以上摘要已纳入 Git。
首次观测尝试使用 TFMessage 桥接位姿，但转换后模型名称为空，无法按名称识别；
改用 Gazebo 原始消息后完成上述独立位姿验证。

## 尚未验证及观察到的问题

- 未完成带行人的 Home → A → B → C → Home 自动巡检及三次重复验收。
- 未注入机器人与行人的碰撞来验证接触事件的完整传递链。
- 本次运行未验证暂停/重置按钮；运动实现使用仿真时间，端点约束有单元测试。
- 测试停止时，官方机器人话题桥和接触话题桥出现退出码 -11，部分 Nav2 节点需要
  SIGTERM 退出。运行期间的运动、位姿和雷达检查已通过；退出问题未在本次功能中修复。
- Jazzy 未做本次运行验证。

行人是按预设路线移动的简化实体人形，无骨骼步行动画，也不会主动避让机器人。
