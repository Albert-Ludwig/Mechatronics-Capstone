# 行人平滑运动与 Nav2 避障测试（0.6.0）

此页记录 0.6.0 的基线测试。当前 0.6.1 的区域调整及验证见
[减速和停车区域调整](COLLISION_ZONE_TUNING_CN.md)；下面的测试命令仍适用。

环境：本机 ROS 2 Lyrical / Gazebo Sim 10。行人巡航 0.6 m/s，机器人前进上限
1.0 m/s，转弯及避障时由 Nav2 决定实际速度。Jazzy 尚未运行此测试工具。

## 如何复现自动测试

在本机已登录图形桌面的终端运行：

```bash
cd /home/qhl/inventory_nav2_poc_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to inventory_inspection_poc
source install/local_setup.bash
python3 src/inventory_inspection_poc/tools/verify_pedestrian_runtime.py --mode mission --timeout 360
```

工具会启动全新的机器人、行人、Gazebo、AMCL 和 Nav2，自动跑一轮
Home → A → B → C → Home。它使用独立 ROS domain 与 Gazebo partition，
避免与桌面正在运行的场景混在一起；结束后关闭自己启动的进程。
默认不打开 Gazebo/RViz 窗口，GPU 雷达仍需要本地可用的渲染环境。
`360` 是墙钟超时秒数，较慢机器可增加；超时且未完成任务会失败。

开头打印 `Runtime evidence: /tmp/nav2-mission-...`，该目录包含：

- `verification.json`：最终测量摘要。
- `results/<时间>/mission_report.json`：任务逐项验收和事件。
- `results/<时间>/trajectory.csv`：机器人 TF 位姿及里程计轨迹。
- `launch.log`、`ros/`：启动与运行日志。
- `poses.jsonl`：Gazebo 实体真实位姿。

成功时进程退出码为 0；碰撞、机器人抬升/倾倒、越界、任务失败或缺少关键观测时
退出码非零。失败时查看终端的断言/异常和 `launch.log`，提前失败可能没有最终摘要。
`/tmp` 可能被清理，要保留结果请复制整个输出目录。

## 想看画面时怎么跑

关闭当前桌面演示，再启动可见场景：

```bash
systemctl --user stop inventory-nav2-gui inventory-nav2-sim
./run_poc.sh pedestrian:=true
```

这会编译并打开 Gazebo、RViz，自动运行同一条巡检路线。
在另一个终端，可以只观测这次场景，不再启动第二个世界：

```bash
cd /home/qhl/inventory_nav2_poc_ws
source /opt/ros/lyrical/setup.bash
source install/local_setup.bash
python3 src/inventory_inspection_poc/tools/verify_pedestrian_runtime.py --mode mission --attach --timeout 360
```

尽早启动观测工具，建议在场景启动后的数秒内运行。
`--attach` 需要任务尚未结束，否则会等待新的报告直到超时；它不会重置或重跑旧任务。
默认报告在 `~/inventory_poc_results`；如果 launch 使用其他 `results_dir`，观测工具要加
`--results-dir /实际目录`。该终端需与场景使用相同 ROS domain 和 Gazebo partition。
上面的手动 launch 默认使用当前终端环境；本次自动打开的桌面服务使用
`GZ_PARTITION=inventory_nav2_visible`，附加到该服务时也要在观测终端导出同一值。
附加模式退出时保留原场景，启动场景的终端按 Ctrl+C 才会关闭它。

## 测试实际做了什么

1. **真实导航任务**：将 A/B/C 交给 Nav2 `FollowWaypoints`，各设置停留 5 秒，
   然后用 `NavigateToPose` 返回 Home。检查动作成功、没有漏掉目标，且成功返航。
   mission 模式的观测工具不发布速度，行人也不读取机器人位置或主动让路。
2. **实测停留**：读取 TF 位姿和里程计，检查机器人在当前检查点 0.23 米范围内、
   朝向误差 ≤0.25 弧度、线速度 ≤0.02 m/s、角速度 ≤0.04 rad/s，
   连续保持至少 4.5 秒；数据间断不能算作静止停留。
3. **接触监测**：墙、货架与行人共 8 个 Gazebo 接触传感器需全部发现发布端；
   任何上报接触都导致失败。发生接触时任务会取消导航并发送紧急零速度。
   传感器是事件触发，不能用“没收到消息”直接推断链路正常。
4. **物理稳定与行人边界**：读取 Gazebo 真实位姿，检查行人 x 在 [-4, 2.8]、
   y 在 0 附近；机器人参考坐标 z 不超过 0.15 米、倾斜不超过 0.35 弧度。
   这些是异常保护阈值，摘要还会给出实际最大值。
5. **Nav2 确实在工作**：观测 `/cmd_vel_nav` 的控制器命令，并在 Nav2 全局代价地图
   的原本空闲通道中检查行人腿部附近的高代价值。须既有控制命令，也有动态障碍观测。
6. **返航与总验收**：回 Home 定位误差 ≤0.25 米、朝向误差 ≤0.30 弧度，
   行人反馈正常，上述任务、停留和接触检查全部通过才有 `acceptance_passed=true`。
7. **额外记录**：行人实际关节速度、机器人实际里程计速度、Nav2 最大前进命令、
   人物视觉朝向变化速率、最近人车参考点距离。后者是参考点距离，不是表面间隙。

另有 `--mode contact`，它会以 0.12 m/s **主动碰撞行人**来检查接触链路与物理稳定，
不代表避障测试。本次要求的一轮使用 `mission`，没有运行这个主动碰撞模式。

## 平滑运动改动如何检查

行人以 2 秒余弦速度曲线起步、制动，端点停稳后留出 3.2 秒转身。
视觉位置每个物理步直接同步自身物理模型，转身角速度上限 1.4 rad/s，
角加速度上限 2.5 rad/s²。步行动画按实际位移推进，转身时保留小幅迈步。
底层仍是有限推力物理关节；碰撞代理为简化人体几何，并非逐骨骼碰撞。

代码检查命令：

```bash
python3 -m unittest discover -s src/inventory_inspection_poc/tests -v
```

31 项检查包括速度曲线连续性、目标速度/加速度、停稳换向、范围、反馈过期、
模型及贴图完整性、机器人三处限速配置、任务动作失败处理和停留计时。
它们与完整仿真测试互相补充，不能替代真实运行。

## 本次运行结果

2026-10-28，新启动的一轮完整仿真通过，测试工具退出码为 0。
测试过程中另开 Gazebo GUI 连接同一隔离场景供观察，未修改运行参数。
原始日志目录为 `/tmp/nav2-mission-8pg4ffk5`，完整摘要已保存到
[本轮测量报告](runtime/nav2-v0.6.0-smooth-pedestrian.json)。

| 检查项 | 本轮实测 |
|---|---|
| 任务与总验收 | Home → A → B → C → Home 完成，`acceptance_passed=true` |
| 接触事件 | 0；8/8 接触发布端已发现 |
| 任务耗时 | 仿真时间 182.862 秒；任务节点墙钟约 190.36 秒 |
| A/B/C 连续静止停留 | 5.406 / 5.406 / 5.202 秒 |
| 行人实际最大速度 | 0.600000000001 m/s，浮点误差约 1e-12 |
| 机器人实际最大线速度 | 1.000000000000 m/s |
| 人物实际最大转身速率 | 1.400000064 rad/s，8772 个视觉位姿样本，含完整 180° 转身 |
| 行人物理 x 范围 | -4.000～2.78765 米，y=0 |
| 机器人参考坐标最高 z | 0.01000714 米，维持初始约 0.01 米 |
| 机器人最大倾斜 | 0.00004034 弧度 |
| 最近人车参考点距离 | 1.21246 米，不能当作表面间隙 |
| Nav2 控制命令 / 地图动态障碍观测 | 3266 条 / 59 次 |
| Nav2 碰撞监测器减速动作 | 11 次 SlowCircle 状态切换 |
| 回 Home 定位误差 | 距离 0.02851 米；朝向 0.15332 弧度 |

31 项 unittest、原生插件与 Python 包构建、Python 编译和 shell 语法检查通过。
此外对摘要检查了实际速度上限、转身速率上限及完整 180° 转身。
本次没有重新触发主动碰撞，接触检测链路的主动接触试验保留在
[历史碰撞修复记录](PEDESTRIAN_FIX_VERIFICATION.md)。

单次通过只说明这一组路线、初始位置和参数在本次运行通过。
尚未完成三次独立重启的重复验收，也未覆盖随机初始相位、更多行人和所有交会方式。
