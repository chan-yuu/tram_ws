# Tram Simulation Workspace — 设计文档

## 1. 架构总览

```
┌─────────────────────────────────────────────────────────────────┐
│                        tram_bringup                             │
│                    (一键 Launch 入口)                            │
└────────────────────────────┬────────────────────────────────────┘
                             │
          ┌──────────────────┼──────────────────┐
          ▼                  ▼                  ▼
  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐
  │ tram_planning │  │ tram_control  │  │tram_visualiz- │
  │  (路径发布)    │  │  (控制器)     │  │  ation(可视化) │
  └───────┬───────┘  └───────┬───────┘  └───────┬───────┘
          │                  │                  │
          │ /planning/path   │ /control/*        │
          │ /planning/path_  │ /control/heart-   │
          │  viz(nav Path)   │  beat             │
          └─────────┐        │         ┌─────────┘
                    ▼        ▼         ▼
              ┌─────────────────────────────┐
              │       tram_simulator         │
              │   (6-state 动力学仿真节点)    │
              └─────────────────────────────┘
                    │
                    │ /sim/vehicle_state
                    │ /sim/odom (nav_msgs/Odometry)
                    │ /sim/pose (PoseStamped)
                    │ /sim/path_trace (nav_msgs/Path)
                    │ TF: map → base_link
                    ▼
              ┌─────────────────────────────┐
              │      tram_description        │
              │  (vehicle.json + 动力学模型) │
              └─────────────────────────────┘
```

## 2. 包结构

```
tram_ws/src/
├── DESIGN.md
├── tram_msgs/               # 自定义 ROS 消息
├── tram_description/        # 车辆参数 + 动力学模型 + URDF 描述
├── tram_simulator/          # 仿真节点
├── tram_planning/           # 路径规划 + 场景生成
├── tram_control/            # 控制器节点（PI+LQR / MPC）
├── tram_visualization/      # 标记可视化节点 + RViz 配置
└── tram_bringup/            # 顶层 Launch 文件
```

## 3. 话题接口总表

### 3.1 控制话题（tram_control → tram_simulator）

| 话题 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `/control/torque_front` | `std_msgs/Float64` | N·m | 前轴电机扭矩，正=驱动，负=制动 |
| `/control/torque_rear` | `std_msgs/Float64` | N·m | 后轴电机扭矩 |
| `/control/brake_decel` | `std_msgs/Float64` | m/s² | 附加摩擦制动减速度（纯正值） |
| `/control/steering_cmd` | `std_msgs/Float64` | rad | 前轮路面转角 |
| `/control/steering_rear_cmd` | `std_msgs/Float64` | rad | 后轮路面转角 |
| `/control/heartbeat` | `std_msgs/Header` | - | 控制器心跳，>500ms 未收到则停车 |

### 3.2 仿真状态话题（tram_simulator → 其他）

| 话题 | 类型 | frame_id | 说明 |
|------|------|----------|------|
| `/sim/vehicle_state` | `tram_msgs/VehicleState` | map | 6 状态 [x,y,ψ,vx,vy,r] |
| `/sim/odom` | `nav_msgs/Odometry` | map→base_link | 标准里程计 |
| `/sim/pose` | `geometry_msgs/PoseStamped` | map | 当前位姿 |
| `/sim/twist` | `geometry_msgs/TwistStamped` | base_link | 当前速度 |
| `/sim/path_trace` | `nav_msgs/Path` | map | 实际行驶轨迹（历史） |
| `/sim/control_echo` | `tram_msgs/ControlCmd` | - | 控制器输出回显 |
| TF `map→base_link` | tf | - | 实时位姿变换；`base_link` 为车辆 CG 的地面投影 |
| TF `base_link→front_axle/rear_axle` | tf_static | - | URDF 固定坐标系，由 robot_state_publisher 发布 |

### 3.2.1 坐标系约定

- 6 状态模型中的 `x, y, psi` 是车辆质心 CG 在地面平面的投影，不是后轴中心
- `/sim/vehicle_state`、`/sim/odom`、`map→base_link` 都使用这个 CG 投影
- URDF 中 `front_axle` 相对 `base_link` 为 `+3.638m`
- URDF 中 `rear_axle` 相对 `base_link` 为 `-3.962m`
- 如果后续控制或定位需要以后轴中心为参考，可使用模型里的 `cg_to_rear()` 或 TF `base_link→rear_axle` 转换

### 3.3 规划话题（tram_planning → 其他）

| 话题 | 类型 | frame_id | 说明 |
|------|------|----------|------|
| `/planning/path` | `tram_msgs/TramPath` | map | 含速度的自定义路径（Latched） |
| `/planning/path_viz` | `nav_msgs/Path` | map | 标准 Path，供 RViz 显示（Latched） |
| `/planning/scenario` | `std_msgs/String` | - | 当前场景名 |
| `/planning/target_speed` | `std_msgs/Float64` | - | 路径代表速度标量（rqt_plot 用） |

### 3.4 调试话题（tram_control → rqt_plot）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/debug/speed_actual` | `std_msgs/Float64` | 实际速度 m/s |
| `/debug/speed_ref` | `std_msgs/Float64` | 参考速度 m/s |
| `/debug/lateral_error` | `std_msgs/Float64` | 横向误差 m |
| `/debug/heading_error` | `std_msgs/Float64` | 航向误差 rad |
| `/debug/steering_front` | `std_msgs/Float64` | 前轮指令 rad |
| `/debug/steering_rear` | `std_msgs/Float64` | 后轮指令 rad |
| `/debug/accel_cmd` | `std_msgs/Float64` | 纵向加速度指令 m/s² |
| `/debug/progress` | `std_msgs/Float64` | 路径完成比例 [0,1] |
| `/debug/stop_distance` | `std_msgs/Float64` | 到当前停车点的带符号距离 m |
| `/debug/stop_abs_error` | `std_msgs/Float64` | 当前停车点绝对误差 m |
| `/debug/stop_state` | `std_msgs/String` | 停车状态：cruise/brake/crawl/settle/hold/terminal_hold |
| `/debug/stop_dwell_elapsed` | `std_msgs/Float64` | 当前停车保持已持续时间 s |
| `/debug/stop_dwell_remaining` | `std_msgs/Float64` | 当前中途站 dwell 剩余时间 s，终点停车为 0 |

控制节点还会按 `debug_control_detail_period` 低频打印 `[control_node][detail]` 日志，用于诊断掉头和道岔等横向控制问题。日志包含路径进度、当前位置、最近点距离、曲率、参考/实际速度、曲率限速状态、横向误差、航向误差、前后轮转角、转角饱和标志、侧偏角、横摆角速度和停车状态。

### 3.5 可视化话题（tram_visualization）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/viz/vehicle_marker` | `visualization_msgs/MarkerArray` | 车辆包围盒 + 坐标轴 + 随车状态文字 |
| `/viz/path_markers` | `visualization_msgs/MarkerArray` | 路径点 + 曲率指示 |
| `/viz/key_points` | `visualization_msgs/MarkerArray` | 起点、站点停车点、终点停车点、减速/加速入口、曲线入口/出口 |
| `/viz/control_markers` | `visualization_msgs/MarkerArray` | 最近点、横向误差箭头 |
| `/viz/lookahead_marker` | `visualization_msgs/Marker` | 前视目标点 |

### 3.6 服务接口

| 服务 | 类型 | 节点 | 说明 |
|------|------|------|------|
| `/sim/reset` | `std_srvs/Empty` | tram_simulator | 重置车辆到初始状态 |
| `/planning/set_scenario` | 待定 | tram_planning | 后续如需在线切换场景再补充 |

## 4. 自定义消息定义

### VehicleState.msg
```
Header header
float64 x        # 全局 x (m)
float64 y        # 全局 y (m)
float64 psi      # 偏航角 (rad)
float64 v_x      # 纵向速度 (m/s, 车体系)
float64 v_y      # 横向速度 (m/s, 车体系)
float64 r        # 横摆角速度 (rad/s)
float64 speed    # 合速度 sqrt(vx²+vy²) (m/s)
```

### PathPoint.msg
```
float64 x        # 路径点全局 x (m)
float64 y        # 路径点全局 y (m)
float64 yaw      # 路径切线方向 (rad)
float64 v        # 期望速度 (m/s)
float64 s        # 累计弧长 (m)
float64 kappa    # 路径曲率 1/R (1/m)，直线=0
bool stop_point  # 是否为运营停车点
float64 dwell_time # 停车保持时间 (s)，仅 stop_point 有效
```

### TramPath.msg
```
Header header
string scenario_id
tram_msgs/PathPoint[] points
```

### ControlCmd.msg（内部日志 / 回显）
```
Header header
float64 torque_front    # N·m
float64 torque_rear     # N·m
float64 brake_decel     # m/s²
float64 steering_front  # rad
float64 steering_rear   # rad
string  controller_mode # "pi_lqr" | "mpc"
```

## 5. 控制算法说明

### 5.1 PI + LQR 策略

**纵向（PI）**
- 误差：e_v = v_ref - v_actual
- 输出：a_cmd = Kp·e_v + Ki·∫e_v dt（含 anti-windup）
- 转矩分配：vehicle.torques_from_accel_cmd(a_cmd, ...)
- 制动分配：前 45% / 后 55%

**横向（LQR）**
- 状态向量：x = [e_y, e_ψ, v_y, r]
  - e_y：横向位置误差（正 = 路径左侧）
  - e_ψ：航向误差（正 = 车头偏左）
- 线性化自行车模型（以当前 v_x 为调度变量）
- 离散化（ZOH，dt=0.02s）→ 求解离散 DARE → 反馈增益 K
- 输出：[δ_f, δ_r] = -K · x + 曲率前馈
- 增益每步在线重算（v_x 变化时跟踪更新）

**弯道速度保护**
- 控制器在曲率段按 `v <= sqrt(curve_lateral_accel_limit / |kappa|)` 对参考速度二次限幅
- 默认 `curve_lateral_accel_limit=1.2m/s²`，R=15m 掉头弯道会被限制到约 4.2m/s
- 该保护用于避免场景速度配置过高时横向控制器物理上跟不过弯

### 5.2 MPC 策略

- 当前实现：纵向仍使用 PI + 精准停车状态机，横向使用线性 MPC
- 状态：z = [e_y, e_ψ, v_y, r]（4维）
- 控制：u = [δ_f, δ_r]（2维）
- 预测步长：N=15，dt=0.05s（750ms 预测域）
- 目标函数：min Σ z_k.T Q z_k + u_k.T R u_k
- 约束：转角限幅、首步转角变化率限幅
- 求解器：线性二次型预测矩阵 + 解析解后限幅
- 后续如需“完整 MPC”，再扩展为 [e_y, e_ψ, v_y, r, v_x] 和 [δ_f, δ_r, a_x]

### 5.3 精准停车逻辑

- 规划路径不再用“站台段 0 速度路径”表达停靠，而是在减速末端标记 `stop_point`
- 控制器搜索前方未完成 `stop_point`，在 `stop_control_distance` 内按剩余距离限制参考速度
- 远距离按舒适减速度包络减速，最后 `stop_crawl_distance` 内进入低速 crawl
- 如果实际速度已经低于停车速度包络，控制器会抑制继续制动并清除 PI 负积分，避免在 crawl 入口附近提前刹停再重新启动
- 到达 `stop_tolerance` 且车速小于 `stop_speed_tolerance` 后，发布保持制动并等待 `dwell_time`
- 中途站点等待结束后将该 stop point 标记为完成，从下一路径点继续加速，不会卡在零速段
- 最后一个路径点也标记为 `stop_point`，作为任务终点；到达后进入 `terminal_hold`，持续保持制动，不再恢复启动

## 6. 场景路径说明

所有场景均以 `ds=0.5m` 采样，自动计算 s（弧长）和 κ（曲率）。所有测试场景都在任务终点设置最终停车点，终点停车不恢复启动。

| 场景 ID | 最高速度 | 总长度 | 关键特征 |
|---------|---------|--------|---------|
| `standard_stop` | 60 km/h | ≈420m | 150m进站 → 60m减速停车 dwell → 60m加速 → 90m巡航 → 60m终点停车 |
| `highspeed_stop` | 80 km/h | ≈350m | 80m进站 → 60m紧急减速停车 dwell → 60m加速 → 90m巡航 → 60m终点停车 |
| `short_platform` | 40 km/h | ≈260m | 60m进站 → 60m减速停车 dwell → 60m加速 → 20m巡航 → 60m终点停车 |
| `switch_lane` | 30 km/h | ≈360m | 150m主线 → 60m道岔过渡 → 90m侧线巡航 → 60m终点停车 |
| `uturn_180` | 8 m/s 直线 / 4 m/s 弯中 | ≈207m | 50m直线 → 30m弯前减速 → R=15m半圆 → 20m出弯加速 → 20m反向巡航 → 40m终点停车；控制器也会按曲率限速 |

## 7. 节点运行频率

| 节点 | 频率 | 定时器类型 |
|------|------|-----------|
| `sim_node` | 100 Hz | rospy.Timer |
| `control_node` (PI+LQR) | 50 Hz | rospy.Timer |
| `control_node` (MPC) | 20 Hz | rospy.Timer |
| `planning_node` | Latched + 1 Hz | 发布后保持 |
| `viz_node` | 20 Hz | rospy.Timer |

## 8. 一键启动说明

```bash
# PI+LQR 控制 + 标准站台场景
roslaunch tram_bringup sim_pi_lqr.launch scenario:=standard_stop

# MPC 控制 + 掉头场景
roslaunch tram_bringup sim_mpc.launch scenario:=uturn_180

# 各场景快速启动（默认 PI+LQR）
roslaunch tram_bringup scenarios/switch_lane.launch
```

## 9. 依赖项

| 依赖 | 用途 |
|------|------|
| `numpy` | 矩阵运算、数值积分 |
| `scipy` | DARE 求解（LQR）；MPC 当前使用 numpy 解析解 |
| `tf` | TF 广播 |
| `rospkg` | 跨包路径查找 |

ROS 标准包：`std_msgs` `nav_msgs` `geometry_msgs` `visualization_msgs` `std_srvs` `diagnostic_msgs`
