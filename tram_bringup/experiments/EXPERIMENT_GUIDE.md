# Tram 场景实验快速操作手册

> 本文档说明如何一键重跑全部/单个场景实验，以及如何查看可视化结果。

---

## 1. 环境准备

确保已 source 工作空间（**任意终端首次执行**）：

```bash
cd /home/cyun/Documents/project/tram_ws
source devel/setup.bash
```

---

## 2. 一键重跑全部实验

执行自动化脚本，它会自动：启动 roscore → 依次运行 5 个场景 → 自动检测完成 → 保存 CSV → 生成报告。

```bash
cd /home/cyun/Documents/project/tram_ws

# 1) 运行全部实验
python3 src/tram_bringup/scripts/run_experiments.py

# 2) 生成报告（含 SCI 风格图表）
python3 src/tram_bringup/scripts/generate_report.py
```

**总耗时**：约 8–10 分钟（5 个场景连续运行）。

---

## 3. 单独重跑某个场景

如果只想测试/重跑单个场景，手动组合 **数据记录节点 + roslaunch**：

```bash
cd /home/cyun/Documents/project/tram_ws
source devel/setup.bash

# 先清理旧数据（可选）
rm src/tram_bringup/experiments/standard_stop.csv

# 终端 1：启动数据记录
python3 src/tram_bringup/scripts/data_logger.py \
  src/tram_bringup/experiments/standard_stop.csv

# 终端 2：启动场景（新开终端，同样 source 后执行）
roslaunch tram_bringup sim_pi_lqr.launch scenario:=standard_stop rviz:=false
```

`data_logger.py` 会自动检测场景完成（progress ≥ 0.99 且速度 < 0.1 m/s 持续 3 秒）并退出。

**各场景启动命令对照表**：

| 场景 | 启动命令 |
|------|----------|
| 标准站点停靠 | `roslaunch tram_bringup sim_pi_lqr.launch scenario:=standard_stop rviz:=false` |
| 高速站点停靠 | `roslaunch tram_bringup sim_pi_lqr.launch scenario:=highspeed_stop rviz:=false` |
| 短站台停靠 | `roslaunch tram_bringup sim_pi_lqr.launch scenario:=short_platform rviz:=false` |
| 道岔换道 | `roslaunch tram_bringup sim_pi_lqr.launch scenario:=switch_lane rviz:=false` |
| 180° 弯道掉头 | `roslaunch tram_bringup sim_mpc.launch scenario:=uturn_180 rviz:=false` |

> **注意**：U-turn 使用 MPC 控制器，其余场景使用 PI+LQR。

---

## 4. 打开 RViz 实时可视化（调试用）

如果需要在运行时观察 RViz，去掉 `rviz:=false` 即可：

```bash
roslaunch tram_bringup sim_pi_lqr.launch scenario:=uturn_180
```

或使用 GUI 启动器：

```bash
python3 src/tram_bringup/scripts/scenario_gui.py
```

---

## 5. 查看实验结果

### 5.1 原始数据（CSV）

```bash
ls src/tram_bringup/experiments/*.csv
```

每行记录包含：时间、场景名、车辆状态 `[x,y,ψ,vx,vy,r]`、速度跟踪、横向误差、航向误差、转向角、加速度指令、路径完成比例。

### 5.2 可视化图表

每个场景有独立文件夹，含 6 张 SCI 风格图：

```
src/tram_bringup/experiments/report/
├── report.md                    ← 完整 Markdown 报告
├── standard_stop/
│   ├── speed_tracking.png       ← 速度跟踪
│   ├── lateral_error.png        ← 横向误差
│   ├── heading_error.png        ← 航向误差
│   ├── steering.png             ← 前后轮转角
│   ├── trajectory.png           ← XY 轨迹
│   └── accel.png                ← 纵向加速度指令
├── highspeed_stop/  (同上 6 张)
├── short_platform/  (同上 6 张)
├── switch_lane/     (同上 6 张)
└── uturn_180/       (同上 6 张)
```

### 5.3 查看 Markdown 报告

```bash
cat src/tram_bringup/experiments/report/report.md
```

或在 VS Code 中直接预览：打开 `report.md` → `Ctrl+Shift+V`（或右键 → Preview）。

---

## 6. 修改场景后重新实验

如果修改了路径规划（`tram_planning/scenarios/`）或控制器参数（`tram_control/config/`），需要确保代码生效：

```bash
cd /home/cyun/Documents/project/tram_ws
# Python 脚本无需 catkin_make，直接重跑即可
python3 src/tram_bringup/scripts/run_experiments.py
python3 src/tram_bringup/scripts/generate_report.py
```

> Python 节点在运行时通过 `rosrun` 或 `roslaunch` 直接调用源码，因此修改 `.py` 文件后**无需编译**，重跑即生效。

---

## 7. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `CSV missing or empty` | logger 未正常启动或 launch 提前退出 | 检查 roscore 是否在运行，手动执行单场景测试 |
| 实验卡住不结束 | 车辆未完成路径或停止条件未触发 | 检查 `run_experiments.py` 中的 `timeout`，默认 120–150s |
| trajectory.png y 轴不可见 | 已修复，使用最新 `generate_report.py` | 重新执行报告生成脚本 |
| 图表中文字符显示为方块 | 系统缺少中文字体 | 不影响数据可读性；如需修复，安装 `fonts-wqy-zenhei` 并删除 matplotlib 缓存 |

---

## 8. 核心脚本清单

| 脚本 | 路径 | 用途 |
|------|------|------|
| `run_experiments.py` | `src/tram_bringup/scripts/` | 一键批量运行全部 5 个场景 |
| `data_logger.py` | `src/tram_bringup/scripts/` | ROS 数据记录节点（20 Hz，自动检测完成） |
| `generate_report.py` | `src/tram_bringup/scripts/` | 读取 CSV 生成 SCI 风格图表 + Markdown 报告 |
| `scenario_gui.py` | `src/tram_bringup/scripts/` | 可视化 GUI 启动器（带按钮） |

---

*文档生成时间：自动生成*
