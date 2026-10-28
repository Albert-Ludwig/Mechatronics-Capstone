# 仓库自动巡检 POC：先读这里

实现目标来自 `Concept_design_doc.md`：**Home → A → B → C → Home**。
用官方 TurtleBot3 Waffle、AMCL、Nav2 和 Waypoint Follower；A/B/C 各停5秒。
这是完整目标导航集成代码，不再使用之前“遇障碍固定转1.5秒”的节点。

## 本版修改：0.5米小车与C点背面检查

以先前Gazebo截图从Home看向仓库的视角：

| 点 | 对应货架 | 坐标 (x,y) | 面向 |
|---|---|---|---|
| Home | 靠近画面下方的起点 | (-5.5, 0) | +x / 0度 |
| A | 画面左侧长货架 | (-1.5, 1.5) | +y / 90度 |
| B | 画面右侧长货架 | (-1.5, -1.5) | -y / -90度 |
| C | 最远处横着的货架，改到其后侧 | (5.5, 0) | -x / 180度 |

C原来是(2.5,0,0)，现在是(5.5,0,pi)。Home/A/B没有移动。
Nav2自行规划绕过C货架的路线；不增加强制经过的中间目标。
`docs/warehouse_layout.png` 展示与截图一致的观察方向。

小车模型改为0.50×0.50米的蓝色方形底盘，车身高度保持0.089米。
Gazebo物理碰撞体、RViz的URDF外观与碰撞体同时改变。左右轮距改为0.47米，
保持车轮在0.5米总宽范围内，同时更新DiffDrive参数和URDF轮关节。
模型在启动时从本机官方TurtleBot3生成，不修改 `/opt/ros` 的安装文件。

Nav2局部costmap使用方形footprint：

```yaml
footprint: '[[0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25]]'
footprint_padding: 0.025
```

MPPI启用 `CostCritic.consider_footprint: true`。全局NavFn按圆形机器人检查栅格，
因此全局costmap使用半径0.39米的保守外接圆，覆盖方形四角和安全余量，不能用0.25米半径。
碰撞监测器读取局部costmap发布的footprint。禁行区关闭方式保留上次修复，不使用 `filters: []`。

## 1. 解压到主目录，保留独立工作区

压缩包内已有顶层 `inventory_nav2_poc_ws/` 文件夹。下载后：

```bash
unzip -n "/home/qhl/下载/inventory_nav2_poc.zip" -d /home/qhl
cd /home/qhl/inventory_nav2_poc_ws
```

`-n` 不覆盖同名文件。以后更新时先备份自己的修改，再将新包解压到另一个目录比较。
这个工作区不会覆盖 `~/inventory-management-robot-simulation`。

## 2. 安装依赖：按你正在使用的系统选择一行 source

Ubuntu 26.04 / Lyrical：

```bash
source /opt/ros/lyrical/setup.bash
```

Ubuntu 24.04 / Jazzy：

```bash
source /opt/ros/jazzy/setup.bash
```

同一终端只加载一种 ROS 发行版。打开新终端时，先检查 `.bashrc` 有没有加载别的项目/版本。
下面的 `$ROS_DISTRO` 会使用刚加载的发行版：

```bash
echo "$ROS_DISTRO"
sudo apt update
sudo apt install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-nav2-minimal-tb3-sim ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-rviz2 python3-colcon-common-extensions python3-yaml python3-setuptools
```

如果某个安装包找不到，保留完整输出，不要混用其他发行版的软件源。
Humble/Fortress 不在这个 POC 包的目标范围内。

## 3. 编译、检查、启动

先关闭昨天的 Gazebo 和避障节点。它们也使用 `/cmd_vel`，不能与 Nav2 同时控制机器人。

```bash
cd /home/qhl/inventory_nav2_poc_ws
colcon build --symlink-install --packages-select inventory_inspection_poc
source install/local_setup.bash
ros2 run inventory_inspection_poc check_environment
ros2 launch inventory_inspection_poc inspection.launch.py
```

正常过程：Gazebo出现15米见方仓库、3个货架、TurtleBot3；RViz显示地图；程序等待
AMCL/导航节点、雷达、里程计和TF就绪，然后自动运行任务。无需点击 RViz 的 Initial Pose。
Home附近的绿色方块是起点，另外3个绿色方块是检查点。

终端关键事件：

```text
waiting_for_navigation
localization_ready
mission_started
navigating_to A
navigating_to B
navigating_to C
returning_home
mission_complete
```

Waypoint Follower 自己也会打印 `sleeping for 5000 milliseconds`。
RViz里可以看到 Nav2 的规划路径。完整任务耗时取决于仿真速度，通常应预留几分钟，
这里没有测量过你电脑上的实际耗时。

## 4. 结果在哪里

每次任务单独保存在：

```text
/home/qhl/inventory_poc_results/<运行时间>/
├── mission_report.json
└── trajectory.csv
```

- `navigation_success`：A/B/C动作和返回Home动作都成功，无已报告碰撞。
- `dwell_measurements`：用TF位姿和轮式里程计测量每个目标附近的连续静止时间；
  目标设置5秒，观测允许采样误差，至少4.5秒。
- `contact_monitor_coverage`：Gazebo中发现7个墙/货架接触传感器话题的比例。
  接触传感器只在碰撞时发布消息，不能要求没有碰撞时也收到“空消息”。
- `acceptance_passed`：导航、观测停留、回Home位置/朝向、接触监测覆盖和无已报告碰撞都满足。
- `error`：失败原因，例如定位超时、目标被拒绝、导航动作失败或漏掉检查点。

话题发现只能证明监测端已注册，不能独立证明检测链路一定可靠；覆盖不足会让验收标志
为 false。最终3次试验还要配合Gazebo画面检查，保留展示录像。
即使某次 `acceptance_passed=true`，仍需完整重启仿真再连续成功3次，才能满足文档的重复性要求。

退出：启动终端按 `Ctrl+C`。重复试验时重新运行整个 launch，机器人会重新生成在Home。
不要只在旧场景中重跑任务节点，因为它会按Home重新初始化定位。

## 5. 先做单目标测试（文档推荐的第一阶段）

```bash
ros2 launch inventory_inspection_poc inspection.launch.py run_mission:=false
```

AMCL初始位姿由参数自动设置为Home；在RViz里点“Nav2 Goal”，在空旷处拖出朝向。
确认单目标导航成功后，关闭整个仿真，按默认命令启动自动任务。

## 6. 笔记本卡顿时

保留Gazebo，关闭RViz：

```bash
ros2 launch inventory_inspection_poc inspection.launch.py use_rviz:=false
```

无图形窗口运行：

```bash
ros2 launch inventory_inspection_poc inspection.launch.py headless:=true use_rviz:=false
```

`headless`只隐藏窗口，GPU LiDAR仍需要可用的渲染环境；普通笔记本本地登录运行即可，
没有显示/渲染环境的服务器不能只靠这个选项保证成功。

## 7. 调试用第二个终端

加载与启动终端相同的ROS版本，并加载这个工作区。然后：

```bash
source /opt/ros/lyrical/setup.bash
source /home/qhl/inventory_nav2_poc_ws/install/local_setup.bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /inspection/status --once --qos-durability transient_local
ros2 run tf2_ros tf2_echo map base_footprint
```

Jazzy电脑把第一行改为 `/opt/ros/jazzy/setup.bash`。
`ros2 topic hz` 和 `tf2_echo` 会持续输出，按Ctrl+C退出后再执行下一条。

常见问题：

| 现象 | 检查 |
|---|---|
| Package not found | 编译是否成功、是否 source 当前工作区 |
| 模型找不到 | `check_environment`、官方 minimal TB3 包是否安装完整 |
| 没有 `/clock` | Gazebo是否启动、是否暂停、官方桥接是否运行 |
| 没有 `/scan` | Gazebo传感器/渲染错误，先查看启动终端 |
| TF/AMCL超时 | `/odom`、`/scan`、`map→odom→base_footprint→base_link→base_scan` |
| 小车打转或两个控制器抢占 | 确认旧 `obstacle_avoider` 已关闭 |
| 导航成功但停留验收失败 | 查看轨迹、odom速度、仿真实时因子，确认TF观测没有中断 |
| 目标不可达 | 改过布局时是否重新生成地图与世界；不要只移动Gazebo里的货架 |

## 8. 改代码看哪些文件

```text
src/inventory_inspection_poc/
├── config/layout.yaml                 # 唯一布局/检查点/起点定义
├── config/nav2_overrides.yaml         # 本项目Nav2参数修改
├── launch/inspection.launch.py        # 全部组件启动入口
├── inventory_inspection_poc/mission.py# 等待就绪、任务动作、报告
├── inventory_inspection_poc/acceptance.py # 停留/动作结果验收
├── inventory_inspection_poc/robot_model.py # SDF/URDF底盘尺寸、轮距同步
├── worlds/warehouse.sdf               # 自动生成的Gazebo世界
├── maps/warehouse.pgm                 # 自动生成的2D地图
├── maps/warehouse.yaml
├── tools/generate_assets.py           # 同一坐标生成世界与地图
└── tests/test_poc.py                  # 不需要ROS的自动检查
```

改 `layout.yaml` 后运行：

```bash
cd /home/qhl/inventory_nav2_poc_ws/src/inventory_inspection_poc
python3 tools/generate_assets.py
python3 -m unittest discover -s tests -v
cd /home/qhl/inventory_nav2_poc_ws
colcon build --symlink-install --packages-select inventory_inspection_poc
```

Nav2参数在启动时以安装版本自带的默认配置为基础合并。速度消息类型从该版本官方
TurtleBot3桥接配置中检测，避免把 `Twist` 和 `TwistStamped` 混用。
实际生效的临时Nav2参数路径会打印在启动终端里，运行期间可打开查看。

## 9. 已验证与尚未验证

已提供可执行的资产生成、几何可达性、ROS参数合并、动作结果处理、停留观测、
任务顺序与失败分支测试。参考了官方TurtleBot3与Nav2源码。

**当前交付环境未安装ROS/Gazebo，ROS软件源请求失败，所以没有在这里完成Gazebo动态任务。**
不能把静态测试当作“三次完整任务成功”的证据。第一次请从 `check_environment` 开始，
保存实际启动输出；最终展示视频和3次验收结果需在你们电脑运行后产生。

这个POC模拟拍照停留；不保存摄像头图片，不做商品识别、SLAM或学习控制。
