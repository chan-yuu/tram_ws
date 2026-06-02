# Tram 自动驾驶场景测试报告

**生成时间**: 自动生成  
**测试场景数**: 5  

## 1. 实验配置

| 配置项 | 值 |
|--------|-----|
| 控制器 (标准/高速/短站台/道岔) | PI + LQR |
| 控制器 (U-turn) | MPC |
| 仿真频率 | 100 Hz |
| 控制频率 (PI+LQR) | 50 Hz |
| 控制频率 (MPC) | 20 Hz |
| 数据记录频率 | 20 Hz |

## 2. 综合指标汇总

| 场景 | 时长(s) | 距离(m) | 均速(m/s) | 横向误差 RMSE(m) | 横向误差 Max(m) | 航向误差 RMSE(°) | 航向误差 Max(°) | 速度跟踪 RMSE(m/s) |
|------|---------|---------|-----------|------------------|-----------------|------------------|-----------------|--------------------|
| 标准站点停靠 (60 km/h) | 75.7 | 402.0 | 5.31 | 0.000 | 0.000 | 0.00 | 0.00 | 0.850 |
| 高速站点停靠 (80 km/h) | 55.7 | 482.2 | 8.65 | 0.000 | 0.000 | 0.00 | 0.00 | 2.720 |
| 短站台停靠 (40 km/h) | 61.1 | 173.3 | 2.83 | 0.000 | 0.000 | 0.00 | 0.00 | 0.732 |
| 道岔换道 (8°) | 77.1 | 360.5 | 4.67 | 0.003 | 0.031 | 0.12 | 1.17 | 1.211 |
| 180° 弯道掉头 (R=15m) | 78.1 | 217.3 | 2.78 | 0.107 | 0.524 | 1.39 | 7.38 | 1.382 |

## 3. 各场景详细指标

### 标准站点停靠 (60 km/h)

| 指标 | 值 |
|------|-----|
| 运行时长 | 75.75 s |
| 行驶距离 | 402.04 m |
| 采样点数 | 1516 |
| 平均车速 | 5.31 m/s |
| 速度跟踪 RMSE | 0.850 m/s |
| 速度跟踪 MAE | 0.505 m/s |
| 速度跟踪 Max Error | 3.342 m/s |
| 横向误差 RMSE | 0.000 m |
| 横向误差 MAE | 0.000 m |
| 横向误差 Max | 0.000 m |
| 弯道段横向误差 Max | 0.000 m |
| 航向误差 RMSE | 0.00 ° |
| 航向误差 MAE | 0.00 ° |
| 航向误差 Max | 0.00 ° |
| 最大前轮转角 | 0.0 ° |
| 最大后轮转角 | 0.0 ° |
| 最大加速度指令 | 3.50 m/s² |
| 最大减速度指令 | -4.50 m/s² |

![speed](standard_stop/speed_tracking.png)
![lateral](standard_stop/lateral_error.png)
![heading](standard_stop/heading_error.png)
![steering](standard_stop/steering.png)
![trajectory](standard_stop/trajectory.png)
![accel](standard_stop/accel.png)

### 高速站点停靠 (80 km/h)

| 指标 | 值 |
|------|-----|
| 运行时长 | 55.75 s |
| 行驶距离 | 482.17 m |
| 采样点数 | 1116 |
| 平均车速 | 8.65 m/s |
| 速度跟踪 RMSE | 2.720 m/s |
| 速度跟踪 MAE | 1.870 m/s |
| 速度跟踪 Max Error | 8.416 m/s |
| 横向误差 RMSE | 0.000 m |
| 横向误差 MAE | 0.000 m |
| 横向误差 Max | 0.000 m |
| 弯道段横向误差 Max | 0.000 m |
| 航向误差 RMSE | 0.00 ° |
| 航向误差 MAE | 0.00 ° |
| 航向误差 Max | 0.00 ° |
| 最大前轮转角 | 0.0 ° |
| 最大后轮转角 | 0.0 ° |
| 最大加速度指令 | 3.50 m/s² |
| 最大减速度指令 | -4.50 m/s² |

![speed](highspeed_stop/speed_tracking.png)
![lateral](highspeed_stop/lateral_error.png)
![heading](highspeed_stop/heading_error.png)
![steering](highspeed_stop/steering.png)
![trajectory](highspeed_stop/trajectory.png)
![accel](highspeed_stop/accel.png)

### 短站台停靠 (40 km/h)

| 指标 | 值 |
|------|-----|
| 运行时长 | 61.15 s |
| 行驶距离 | 173.28 m |
| 采样点数 | 1224 |
| 平均车速 | 2.83 m/s |
| 速度跟踪 RMSE | 0.732 m/s |
| 速度跟踪 MAE | 0.414 m/s |
| 速度跟踪 Max Error | 3.155 m/s |
| 横向误差 RMSE | 0.000 m |
| 横向误差 MAE | 0.000 m |
| 横向误差 Max | 0.000 m |
| 弯道段横向误差 Max | 0.000 m |
| 航向误差 RMSE | 0.00 ° |
| 航向误差 MAE | 0.00 ° |
| 航向误差 Max | 0.00 ° |
| 最大前轮转角 | 0.0 ° |
| 最大后轮转角 | 0.0 ° |
| 最大加速度指令 | 3.50 m/s² |
| 最大减速度指令 | -1.56 m/s² |

![speed](short_platform/speed_tracking.png)
![lateral](short_platform/lateral_error.png)
![heading](short_platform/heading_error.png)
![steering](short_platform/steering.png)
![trajectory](short_platform/trajectory.png)
![accel](short_platform/accel.png)

### 道岔换道 (8°)

| 指标 | 值 |
|------|-----|
| 运行时长 | 77.15 s |
| 行驶距离 | 360.45 m |
| 采样点数 | 1544 |
| 平均车速 | 4.67 m/s |
| 速度跟踪 RMSE | 1.211 m/s |
| 速度跟踪 MAE | 0.516 m/s |
| 速度跟踪 Max Error | 6.839 m/s |
| 横向误差 RMSE | 0.003 m |
| 横向误差 MAE | 0.001 m |
| 横向误差 Max | 0.031 m |
| 弯道段横向误差 Max | 0.000 m |
| 航向误差 RMSE | 0.12 ° |
| 航向误差 MAE | 0.03 ° |
| 航向误差 Max | 1.17 ° |
| 最大前轮转角 | 8.1 ° |
| 最大后轮转角 | 2.0 ° |
| 最大加速度指令 | 3.50 m/s² |
| 最大减速度指令 | -1.10 m/s² |

![speed](switch_lane/speed_tracking.png)
![lateral](switch_lane/lateral_error.png)
![heading](switch_lane/heading_error.png)
![steering](switch_lane/steering.png)
![trajectory](switch_lane/trajectory.png)
![accel](switch_lane/accel.png)

### 180° 弯道掉头 (R=15m)

| 指标 | 值 |
|------|-----|
| 运行时长 | 78.15 s |
| 行驶距离 | 217.31 m |
| 采样点数 | 1564 |
| 平均车速 | 2.78 m/s |
| 速度跟踪 RMSE | 1.382 m/s |
| 速度跟踪 MAE | 0.589 m/s |
| 速度跟踪 Max Error | 8.000 m/s |
| 横向误差 RMSE | 0.107 m |
| 横向误差 MAE | 0.038 m |
| 横向误差 Max | 0.524 m |
| 弯道段横向误差 Max | 0.430 m |
| 航向误差 RMSE | 1.39 ° |
| 航向误差 MAE | 0.48 ° |
| 航向误差 Max | 7.38 ° |
| 最大前轮转角 | 20.0 ° |
| 最大后轮转角 | 20.0 ° |
| 最大加速度指令 | 3.50 m/s² |
| 最大减速度指令 | -1.19 m/s² |

![speed](uturn_180/speed_tracking.png)
![lateral](uturn_180/lateral_error.png)
![heading](uturn_180/heading_error.png)
![steering](uturn_180/steering.png)
![trajectory](uturn_180/trajectory.png)
![accel](uturn_180/accel.png)

## 4. 结论与建议

- **标准站点停靠**、**高速站点停靠**、**短站台停靠**：主要考核纵向速度跟踪与精准停车能力。
- **道岔换道**：考核横向跟踪与前后轮协同，关注横向误差峰值是否收敛。
- **180° 弯道掉头**：考核大曲率横向跟踪与姿态恢复，MPC 相比 LQR 在曲率阶跃处具有更好的预测能力。
- 若某场景横向误差 RMSE > 0.15 m 或出现持续发散，建议进一步调大 LQR/MPC 的 e_y 权重，或降低该场景参考速度。
