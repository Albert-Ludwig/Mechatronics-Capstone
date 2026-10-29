# Nav2 仿真 Git 管理

日常开发使用 `/home/qhl/inventory_nav2_poc_ws`，主分支为 `main`。
带额外保护的稳定版英文标签为 `stable-simulation-with-extra-protection`，对应
`002773a` / `nav2-v0.6.1`。无额外保护的实验使用独立分支
`experiment/nav2-only-navigation`，命令及结果见 [实验记录](NAV2_ONLY_EXPERIMENT_CN.md)。
2026-10-28 根据现存备份补录前三个版本，0.3.0 起记录后续功能开发。
前三次提交时间是补录时间，不代表原始开发时间；
旧版本只恢复当时保存的 ROS 包，顶层启动脚本和说明文档从当前版开始纳入历史。

| 标签 | 来源 | 变化 |
|---|---|---|
| `nav2-v0.1.0` | `/home/qhl/inventory_inspection_backup_20261016_151409` | 官方 TurtleBot3 原模型；代价地图半径参数 0.22 米；C=(2.5,0,0)；保留 `filters: []` |
| `nav2-v0.1.1` | `/home/qhl/inventory_inspection_poc_before_050m` | 删除局部和全局代价地图中的 `filters: []`，修复已记录的 Lyrical 启动问题 |
| `nav2-v0.2.0` | 当前工作区 | 0.50×0.50 米底盘、0.47 米轮距；局部方形轮廓、全局半径 0.39 米；膨胀半径 0.65 米；C=(5.5,0,pi) |
| `nav2-v0.3.0` | 可选行人初版（有碰撞缺陷） | 增加 `pedestrian:=true`；实际巡检发现行人实体强制移动会撞击小车，已由 0.3.1 修复 |
| `nav2-v0.3.1` | 行人碰撞修复 | 去除静态实体瞬移；采用有限推力物理运动、接近小车停让、碰撞几何匹配及近距离停车；接触立即取消任务 |
| `nav2-v0.4.0` | Nav2 独立避障场景 | 保留物理修复，删除行人停让及其机器人/TF 输入；行人保持独立往返，由 Nav2 控制小车避障 |
| `nav2-v0.5.0` | 1 米/秒与 HuNav 外观 | 行人巡航和机器人前进上限均为 1 米/秒；底盘限速同步；复用旧研究项目的 HuNav 工装人物和步行动画 |
| `nav2-v0.6.0` | 行人降速与平滑运动 | 行人 0.6 米/秒、平滑起停、停稳转身和按位移播放动画；机器人上限保持 1 米/秒 |
| `nav2-v0.6.1` | 减速和停车区域微调 | 减速半径 1.20→1.15 米、停车半径 0.65→0.60 米；保留减速比例及碰撞预判，并记录未通过的收圈试验 |

各版巡检顺序均为 Home → A → B → C → Home，A/B/C 各停 5 秒。
这些标签表示代码快照，不表示完成了 Gazebo 动态验收。
原始备份目录保留，后续修改使用 Git 提交管理，无需再复制版本目录。

`/home/qhl/Mechatronics-Capstone-SIMULATION_LYRICAL` 是另一个已有 Git 的项目仓库，
其中 `Simulation/inventory_nav2_poc_ws` 的 ROS 包在本次整理时与当前版源码一致。
本仓库不自动同步那个副本；其已有提交和远程设置保持独立。
旧的 `inventory-management-robot-simulation` 是避障演示，未纳入这条 Nav2 历史。

## 查看版本和差异

```bash
cd /home/qhl/inventory_nav2_poc_ws
git status
git log --oneline --decorate --graph --all
git tag -n
git diff nav2-v0.1.0 nav2-v0.1.1 -- src/
git diff nav2-v0.1.1 nav2-v0.2.0 -- src/
```

## 保存后续修改

```bash
git diff
git add src/ README_FIRST_CN.md docs/ run_poc.sh .gitignore
git diff --cached
git commit -m "说明本次修改的目的"
```

有需要时，为确认过的里程碑创建新标签，例如：

```bash
git tag -a nav2-v0.2.1 -m "说明本次里程碑"
```

编译产物 `build/`、`install/`、日志 `log/`、Python 缓存、编辑器配置、
助手元数据、凭据目录、嵌套项目仓库和 `*.before_filters_fix` 不纳入版本管理。
地图、Gazebo 世界和布局图属于运行及说明资产，保留在 Git 中。

## 单独检查旧版

使用独立 worktree，不影响当前工作区的文件：

```bash
git worktree add --detach /tmp/inventory_nav2_v011 nav2-v0.1.1
git -C /tmp/inventory_nav2_v011 status
```

旧版只有 ROS 包，运行前需加载相同的 ROS 发行版，在该 worktree 中重新编译。
不要复用当前版的 `build/` 和 `install/`。
检查完且无待保留修改后清理：

```bash
git worktree remove /tmp/inventory_nav2_v011
```

## 撤销某次已提交的修改

先确认 `git status` 没有待保存的修改，用 `git log --oneline` 找到提交号，
然后执行 `git revert <提交号>`，按提示保存提交说明。
这会增加一条反向修改提交，保留完整记录。

本次仅整理本地 Git，没有配置新远程或推送到 GitHub。
