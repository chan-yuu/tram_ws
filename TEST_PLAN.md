# Tram Simulation 测试计划

本文档用于在 ROS 环境中验证 `tram_ws` 的仿真、规划、控制和精准停车逻辑。

## 1. 测试前准备

### 1.1 环境要求

- ROS Noetic
- Python 3
- `numpy`
- `scipy`
- `rospkg`
- `rviz`
- `rqt_plot`（可选；部分 pyqtgraph/numpy 版本组合不兼容）

建议先安装依赖：

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full python3-numpy python3-scipy python3-rospkg
```

### 1.2 构建工作区

因为 `PathPoint.msg` 已增加 `stop_point` 和 `dwell_time` 字段，必须重新构建消息。

```bash
cd ~/vscode_ws/tram_ws
catkin_make
source devel/setup.bash
```

检查消息字段：

```bash
rosmsg show tram_msgs/PathPoint
```

期望看到：

```text
float64 x
float64 y
float64 yaw
float64 v
float64 s
float64 kappa
bool stop_point
float64 dwell_time
```

## 2. 静态接口检查

### 2.1 包依赖检查

```bash
rospack find tram_msgs
rospack find tram_planning
rospack find tram_control
rospack find tram_simulator
rospack find tram_bringup
```

全部应返回对应包路径。

## 3. 单节点测试

建议开 4 个终端，均先执行：

```bash
cd ~/vscode_ws/tram_ws
source devel/setup.bash
```

### 3.1 启动 roscore

终端 1：

```bash
roscore
```

### 3.2 测试规划节点

终端 2：

```bash
roslaunch tram_planning planning.launch scenario:=standard_stop ds:=0.5
```

检查路径话题：

```bash
rostopic echo -n 1 /planning/scenario
rostopic echo -n 1 /planning/path | head -80
rostopic hz /planning/target_speed
```

重点检查：

- `/planning/path` 能收到 `TramPath`
- `scenario_id` 为 `standard_stop`
- 至少有一个点 `stop_point: True`
- 站点停车点 `dwell_time` 应为 `5.0`
- 路径最后一个点也应为 `stop_point: True`，`dwell_time` 应为 `0.0`
- 非停车点不应出现连续 0 速度站台段

停车点检查命令：

```bash
rostopic echo -n 1 /planning/path | grep -n "stop_point: True" -A 2 -B 6
```

### 3.3 测试仿真节点

终端 3：

```bash
roslaunch tram_simulator simulator.launch
```

检查输出：

```bash
rostopic hz /sim/vehicle_state
rostopic echo -n 1 /sim/vehicle_state
rostopic hz /sim/odom
rosparam get /robot_description | head
rosrun tf tf_echo base_link rear_axle
```

期望：

- `/sim/vehicle_state` 约 100 Hz
- 初始 `x/y/psi/v_x/v_y/r` 接近 0
- `/robot_description` 已加载车辆 URDF
- `base_link -> rear_axle` 的 x 平移约为 `-3.962`

当前定位约定：

- `/sim/vehicle_state.x/y` 和 `/sim/odom` 的 `base_link` 是车辆质心 CG 的地面投影
- 不是后轴中心
- 后轴中心使用 TF `rear_axle`，相对 `base_link` 为 `x=-3.962m`

### 3.4 测试控制节点

终端 4：

```bash
roslaunch tram_control control_pi_lqr.launch
```

检查控制话题：

```bash
rostopic hz /control/heartbeat
rostopic echo -n 1 /control/torque_front
rostopic echo -n 1 /control/torque_rear
rostopic echo -n 1 /control/brake_decel
rostopic echo -n 1 /control/steering_cmd
rostopic echo -n 1 /control/steering_rear_cmd
```

期望：

- `/control/heartbeat` 约 10 Hz
- 控制话题持续发布
- 没有 Python exception

## 4. 一键闭环测试

### 4.1 标准停车 PI + LQR

```bash
roslaunch tram_bringup sim_pi_lqr.launch scenario:=standard_stop rviz:=true
```

观察：

- RViz 中车辆沿直线路径行驶
- RViz 中 `/viz/key_points` 能看到 `START`、`STATION STOP`、`FINAL STOP`、`DECEL IN`、`ACCEL IN`
- RViz 中车辆旁边应有随车文字状态牌，包含场景名、当前阶段、实际速度、目标速度、停车距离、dwell 计时和路径进度
- 车辆在停车点附近减速
- 最后低速 crawl
- 停稳约 5 秒
- 之后继续加速出站
- 不应卡死在停车点

### 4.2 关键话题监控

另开终端：

```bash
rostopic echo /debug/speed_actual
rostopic echo /debug/speed_ref
rostopic echo /debug/progress
rostopic echo /debug/stop_distance
rostopic echo /debug/stop_state
rostopic echo /debug/stop_dwell_elapsed
rostopic echo /debug/stop_dwell_remaining
```

建议优先使用 `rostopic echo` 观察关键调试话题。`rqt_plot` 是可选项，部分环境中 `pyqtgraph` 会因为 `np.float` 与新版 `numpy` 不兼容而报错。

如确认本机 `rqt_plot` 可用，可手动启动：

```bash
rqt_plot /debug/speed_actual/data /debug/speed_ref/data /debug/accel_cmd/data
```

或在 bringup 时显式开启：

```bash
roslaunch tram_bringup sim_pi_lqr.launch scenario:=standard_stop rviz:=true rqt_plot:=true
```

如果出现：

```text
AttributeError: module 'numpy' has no attribute 'float'
```

这是系统 Python 包版本问题，不影响仿真节点。可选处理方式：

```bash
python3 -m pip install --user --upgrade pyqtgraph
```

如果 ROS 自带工具被用户目录下的 Python 包污染，也可以临时禁用用户 site packages 后再启动：

```bash
PYTHONNOUSERSITE=1 rqt_plot /debug/speed_actual/data /debug/speed_ref/data
```

期望曲线：

- 进站巡航阶段：实际速度接近参考速度
- 停车控制区：参考速度逐渐降低
- 最后 crawl：参考速度约从 `0.6 m/s` 降到 0
- crawl 入口附近不应出现“先完全刹停，再重新启动”的现象
- `/debug/stop_distance` 为带符号距离，正数表示还没到站，负数表示过站
- `/debug/stop_state` 中途站应依次出现 `cruise/brake/crawl/settle/hold`
- 任务终点应进入 `terminal_hold`，速度保持 0，不再恢复启动
- dwell 期间：实际速度接近 0
- dwell 期间：`/debug/stop_dwell_elapsed` 增加，`/debug/stop_dwell_remaining` 递减
- 出站阶段：参考速度重新升高，车辆继续行驶

## 5. 精准停车专项测试

### 5.1 评价指标

建议记录以下指标：

| 指标 | 推荐目标 |
|------|----------|
| 停车纵向误差 `abs(stop_s - current_s)` | ≤ 0.15 m |
| 停稳速度 | ≤ 0.05 m/s |
| 停车过程最大减速度 | ≤ 4.5 m/s² |
| 舒适停车阶段减速度 | 约 1.2 m/s² |
| 停车后保持时间 | 约 5 s |
| 中途停车后是否继续出站 | 是 |
| 终点停车后是否保持不动 | 是 |

### 5.2 查看停车日志

控制节点应输出类似：

```text
[control_node] stop reached idx=..., error_s=..., dwell=5.0 s
```

如果过站，应输出：

```text
[control_node] stop point overshot idx=..., error_s=...
```

出现 overshot 说明停车精度未达标，需要调参。

### 5.3 调参顺序

优先调整：

1. `stop_control_distance`
2. `stop_comfort_decel`
3. `stop_crawl_distance`
4. `stop_crawl_speed`
5. `pi_kp`
6. `pi_ki`

配置文件：

```text
tram_control/config/pi_lqr_params.yaml
tram_control/config/mpc_params.yaml
```

调参建议：

- 停不住或过站：增大 `stop_control_distance` 或 `stop_comfort_decel`
- 最后抖动：减小 `pi_kp` 或增大 `stop_crawl_distance`
- 停得太慢：增大 `stop_crawl_speed`
- 在 `stop_crawl_distance` 附近提前刹停再启动：增大 `stop_crawl_entry_margin`，或适当减小 `pi_ki`
- 停车误差偏大：减小 `stop_crawl_speed`，增大 `stop_crawl_distance`

## 6. 场景测试清单

### 6.1 标准站点停靠

```bash
roslaunch tram_bringup sim_pi_lqr.launch scenario:=standard_stop rviz:=true
```

检查：

- 60 km/h 进站
- 停车点约 `s=210 m`
- dwell 后继续出站
- 终点约 `s=420 m`，最终进入 `terminal_hold`

### 6.2 高速站点停靠

```bash
roslaunch tram_bringup sim_pi_lqr.launch scenario:=highspeed_stop rviz:=true
```

检查：

- 80 km/h 进站
- 停车点约 `s=140 m`
- 终点约 `s=350 m`，最终进入 `terminal_hold`
- 不应明显过站
- `/debug/accel_cmd` 不应长期超过 `-4.5 m/s²`

如果过站，优先提高：

```yaml
stop_control_distance: 110.0
stop_comfort_decel: 1.5
```

### 6.3 短站台停靠

```bash
roslaunch tram_bringup sim_pi_lqr.launch scenario:=short_platform rviz:=true
```

检查：

- 40 km/h 进站
- 停车点约 `s=120 m`
- 终点约 `s=260 m`，最终进入 `terminal_hold`
- 最终误差应小于 0.15 m

### 6.4 道岔换道

```bash
roslaunch tram_bringup sim_pi_lqr.launch scenario:=switch_lane rviz:=true
```

检查：

- 横向误差 `/debug/lateral_error` 不应持续发散
- 前后轮转角方向应体现 4WS 协同
- 车辆最终进入 8° 侧线
- 侧线末端最终进入 `terminal_hold`

### 6.5 180° 掉头

推荐使用 MPC：

```bash
roslaunch tram_bringup sim_mpc.launch scenario:=uturn_180 rviz:=true
```

检查：

- 弯前 30m 内应从 `8 m/s` 降到约 `4 m/s`
- 半圆弯道内速度应保持约 `4 m/s`
- `/debug/speed_ref` 在弯道中不应明显高于 `4.2 m/s`
- 半径 15 m 弯道中横向误差可控
- 航向角能恢复到反向直线
- 前后轮转角不应长期饱和
- 反向直线末端最终进入 `terminal_hold`

如果跟踪不住：

- 降低 `V_TURN`
- 降低 `curve_lateral_accel_limit`
- 增大 `delta_max`
- 增大 MPC 的 `mpc_horizon`
- 调大 `mpc_Q` 中 `e_y` 和 `e_psi` 权重

掉头调试时，控制节点会每 `debug_control_detail_period` 秒打印一条详细日志：

```text
[control_node][detail] scenario=uturn_180 ...
```

重点看这些字段：

- `nearest_dist`：车辆到最近路径点的距离，持续变大说明路径跟踪失败或最近点跳变
- `kappa`：弯道曲率，R=15m 半圆应约为 `0.0667`
- `v_ref / curve_limit / limited`：确认弯道速度是否被限制到合理范围
- `e_y / e_psi`：横向误差和航向误差，判断是否发散
- `delta_f / delta_r / sat_f / sat_r`：转角是否长期饱和
- `beta / vy / r`：侧偏、横向速度和横摆角速度，判断是否明显侧滑
- `ay_ref / yaw_rate_ref`：参考横向加速度和参考横摆角速度

如果日志太多，可以在控制参数中改为只打印弯道或大误差段：

```yaml
debug_control_detail_curve_only: true
debug_control_detail_period: 1.0
```

## 7. 控制话题接口测试

导师要求的话题必须都存在：

```bash
rostopic list | grep /control
```

必须包含：

```text
/control/torque_front
/control/torque_rear
/control/brake_decel
/control/steering_cmd
/control/steering_rear_cmd
/control/heartbeat
```

检查频率：

```bash
rostopic hz /control/torque_front
rostopic hz /control/torque_rear
rostopic hz /control/brake_decel
rostopic hz /control/steering_cmd
rostopic hz /control/steering_rear_cmd
rostopic hz /control/heartbeat
```

期望：

- PI + LQR 控制话题约 50 Hz
- MPC 控制话题约 20 Hz
- heartbeat 约 10 Hz

## 8. 安全逻辑测试

### 8.1 心跳丢失停车

启动 simulator 后，不启动 control_node，车辆应不动。

闭环运行中手动停止 control_node：

```bash
rosnode kill /control_node
```

期望：

- simulator 在 `0.5 s` 后清零转矩和转角
- `/control/heartbeat` 消失
- 仿真模型施加 `heartbeat_brake_decel`
- 车辆逐渐停车

## 9. 推荐记录数据

运行每个场景时建议录包：

```bash
mkdir -p ~/tram_test_bags
rosbag record -O ~/tram_test_bags/standard_stop_pi_lqr.bag \
  /planning/path \
  /sim/vehicle_state \
  /sim/odom \
  /control/torque_front \
  /control/torque_rear \
  /control/brake_decel \
  /control/steering_cmd \
  /control/steering_rear_cmd \
  /debug/speed_actual \
  /debug/speed_ref \
  /debug/lateral_error \
  /debug/heading_error \
  /debug/accel_cmd \
  /debug/progress \
  /debug/stop_distance \
  /debug/stop_state \
  /debug/stop_dwell_elapsed \
  /debug/stop_dwell_remaining \
  /viz/key_points
```

后续可用 Python 或 `rqt_bag` 分析：

- 停车误差
- 停车时间
- 最大减速度
- 横向误差峰值
- 横摆角速度峰值
- 转角是否饱和

## 10. 通过标准

一个场景认为通过，需要满足：

- 所有必需 ROS 话题存在
- 节点无异常退出
- 仿真车辆能沿路径行驶
- 停车场景不会在 stop point 卡死
- 中途停车误差满足设定容差
- 中途 dwell 结束后能继续出站
- 所有场景最终都在终点停车，进入 `terminal_hold` 后不再恢复启动
- RViz 能看到 `/viz/key_points`：START、STATION STOP、FINAL STOP、DECEL/ACCEL IN、CURVE IN/OUT
- RViz 车辆旁的状态文字能正确显示当前阶段和关键数值
- 横向误差不持续发散
- 控制输出不长期饱和
- 心跳丢失后车辆能安全停车

## 11. 常见问题定位

### 11.1 找不到消息字段

现象：

```text
AttributeError: 'PathPoint' object has no attribute 'stop_point'
```

处理：

```bash
cd ~/vscode_ws/tram_ws
catkin_make
source devel/setup.bash
rosmsg show tram_msgs/PathPoint
```

### 11.2 车辆停住后不再启动

检查：

```bash
rostopic echo -n 1 /planning/path | grep -n "stop_point"
```

如果没有 `stop_point: True`，说明规划消息不是最新版本。

### 11.3 车辆过站

查看 control_node 日志是否有：

```text
stop point overshot
```

处理：

- 增大 `stop_control_distance`
- 增大 `stop_comfort_decel`
- 降低进站速度
- 降低 `stop_crawl_speed`

### 11.4 掉头跟踪失败

处理：

- 用 `sim_mpc.launch`
- 增大 `delta_max`
- 降低掉头速度
- 调高横向误差权重

### 11.5 RViz 无路径或无车辆

检查：

```bash
rostopic echo -n 1 /planning/path_viz
rostopic echo -n 1 /sim/pose
rosrun tf tf_echo map base_link
```

如果 TF 不存在，检查 `sim_node` 是否运行。
