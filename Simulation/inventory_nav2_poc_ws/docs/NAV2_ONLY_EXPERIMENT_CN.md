# Nav2 独立避障实验

日期：2026-10-28（America/Toronto）。本机 ROS 2 Lyrical / Gazebo Sim 10。

## 稳定版快照

用户指定的英文名称为 `stable-simulation-with-extra-protection`，已作为带说明的 Git 标签
直接指向提交 `002773a6327e734691bca4c503e38d4fe1ba65cd`，与 `nav2-v0.6.1` 相同。
该版本保留额外保护，并有两次新场景完整验收通过记录。
名称表示保存的版本，不表示覆盖所有动态交会情况。

实验在独立分支 `experiment/nav2-only-navigation` 中进行，稳定标签及 `main` 不随实验改动。

## 实验改变了什么

新增启动选项 `extra_protection:=false`，关闭 Collision Monitor 的全部预防性干预：

- `StopCircle.enabled=false`：无固定停车圈干预。
- `SlowCircle.enabled=false`：无固定减速圈干预。
- `FootprintApproach.enabled=false`：无额外碰撞时间减速。
- `collision_monitor.enabled=false`，且关闭其观测源，避免源超时仍触发停车。

机器人仍使用雷达、AMCL、代价地图、NavFn、MPPI、Nav2 速度平滑器和物理底盘。
MPPI、代价地图、机器人速度/加速度参数、行人模型、路线、速度和起停曲线与稳定版一致。
行人独立运动，不读取机器人位姿，不停让。

沿用官方启动文件的速度话题链路，Collision Monitor 进程保留为无干预的速度转发节点。
实验额外读取运行中的五个开关，并按消息时间戳配对 `/cmd_vel_smoothed` 和 `/cmd_vel`，
检查实际发送给底盘的前进、侧向和转向速度是否原样透传。不能仅凭配置文件声称没有干预。
此关闭方式根据 [Lyrical 官方实现](https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_collision_monitor/src/collision_monitor_node.cpp) 检查；
关闭观测源只针对该保护组件，不关闭 Nav2 代价地图的雷达输入。

接触传感器继续用于验收。发生真实接触就记失败并取消任务、发布零速度结束试验；
这属于失败后的终止处理，不能算作成功避障。速度透传比较排除接触前最后 0.1 秒及之后的数据，
避免把任务终止零速度混入正常控制链路的比较。

## 运行命令

```bash
cd /home/qhl/inventory_nav2_poc_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to inventory_inspection_poc
source install/local_setup.bash
python3 src/inventory_inspection_poc/tools/verify_pedestrian_runtime.py \
  --mode mission --extra-protection off --timeout 360
```

每次从新世界的 Home 出发，完成 Home → A → B → C → Home；A/B/C 各停 5 秒。
与稳定版使用同样的接触、物理稳定、实际停留、返航和行人范围检查。
工具使用独立 ROS domain 和 Gazebo partition，结束后清理自己的场景。

手动打开实验场景：

```bash
./run_poc.sh pedestrian:=true extra_protection:=false
```

实验分支的默认值仍为 `extra_protection:=true`。显式设为 `false` 才进入无额外保护的实验。
恢复同一分支的保护模式可运行 `./run_poc.sh pedestrian:=true extra_protection:=true`。
先退出当前场景再启动另一场景，避免默认 ROS domain 上出现重复控制节点。

需要单独取出原稳定版本时：

```bash
git worktree add --detach /tmp/nav2-stable-protected stable-simulation-with-extra-protection
```

在该目录独立编译；不要复用实验分支的安装产物。

## 验证记录

32 项 unittest、colcon 构建、Python 编译通过。
新增测试确认关闭保护只改变 Collision Monitor 配置，规划器、控制器、代价地图及平滑器参数一致。

最初一次启动发生 `follow_path` 动作服务器发现超时，导航尚未开始，主动结束后重试。
它属于启动失败，不能作为避障成功或失败的证据。摘录见
[启动失败日志](runtime/nav2-only-startup-failure.log)，原始目录 `/tmp/nav2-mission-sw_mpb3m`。

首次桌面复测也出现 `global_costmap/clear_entirely_global_costmap` 服务发现超时，
未进入导航，主动结束后以独立 ROS domain 重启。见
[桌面启动失败日志](runtime/nav2-only-desktop-startup-failure.log) 和
[失败摘要](runtime/nav2-only-desktop-startup-failure.json)。原始目录 `/tmp/nav2-mission-gk4icwf1`。
报告中的 `Interrupted by operator` 是结束该启动失败场景的结果；
其 `nav2_command_samples=0`、任务阶段为 `startup`，没有把它记作导航成功。

最后一次桌面场景使用 `ROS_DOMAIN_ID=44`、`GZ_PARTITION=inventory_nav2_visible`。
要在第二终端观测该服务，加载 ROS/工作区后设置相同环境：

```bash
export ROS_DOMAIN_ID=44
export GZ_PARTITION=inventory_nav2_visible
```

这只是通信隔离设置，导航控制参数没有变化。

三次新世界完整巡检结果如下；前两次无界面运行，第三次开启桌面 Gazebo / RViz，
通过附加观测器采样。以下是自动验收结果，未录制三次视频，也不声称完成三次人工逐帧检查。

| 运行 | 任务仿真耗时 | 接触事件 | 最小中心距离 | 最终 Home 距离误差 | 速度配对数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| [第 1 次](runtime/nav2-only-run-1.json) | 73.628 s | 0 | 0.956 m | 0.033 m | 1148 |
| [第 2 次](runtime/nav2-only-run-2.json) | 73.212 s | 0 | 1.120 m | 0.084 m | 1139 |
| [第 3 次](runtime/nav2-only-run-3.json) | 74.702 s | 0 | 0.544 m | 0.076 m | 1168 |

中心距离是采样得到的机器人与行人中心距离，不是外形表面的净空距离。
原始目录依次为 `/tmp/nav2-mission-6i7p9gps`、`/tmp/nav2-mission-afzrffeq`、
`/tmp/nav2-mission-c_c_z1xl`，上述 JSON 已复制进仓库，避免依赖临时目录。

三次均完成 Home → A → B → C → Home，A/B/C 实际静止均超过 5 秒，
8/8 接触传感器发布端注册、无接触事件、机器人无飞起或倾倒，`acceptance_passed=true`。
实际机器人速度上限约 1 m/s，行人为 0.6 m/s（浮点误差量级约 1e-12）。
动态代价地图观测数分别为 36、33、33；关闭保护没有切断 Nav2 的障碍物感知。

三次运行的五个保护开关实测均为 false，Collision Monitor 动作记录为空；
配对速度中的运动样本分别为 1085、1077、1106，三次速度差最大值均为 0。
这验证了底盘使用平滑后的 Nav2 指令，没有额外保护圈限速或停车。
另用实际安装的 Nav2 默认参数合并稳定版与实验版配置，排除 Collision Monitor 后逐项比较，
其余参数及场景布局一致；本实验没有调整 MPPI 或代价地图参数来获得通过结果。

**结论：当前场景和当前参数下，Nav2 在没有额外保护干预时完成了三次巡检，均未检测到碰撞。**
这支持在该场景下由 Nav2 自己决定避障速度；三轮测试尚不能覆盖任意行人相位、
突然切入、多行人等情况。两次服务发现导致的启动失败已单独记录，未算入三次导航通过。
测试结束后保留第三次桌面场景，机器人已返回 Home，额外保护仍关闭。
