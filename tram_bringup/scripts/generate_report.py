#!/usr/bin/env python3
"""Experiment report generator — reads CSV logs and produces Markdown + per-scenario plots.

Each scenario gets its own folder under report/ with 6 separate figures (no titles).
Usage:
  python3 generate_report.py
"""
import csv
import math
from pathlib import Path
from collections import defaultdict

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import seaborn as sns
    HAS_MPL = True
except Exception:
    HAS_MPL = False

WS_PATH = Path("/home/cyun/Documents/project/tram_ws")
EXP_DIR = WS_PATH / "src" / "tram_bringup" / "experiments"
REPORT_DIR = EXP_DIR / "report"

SCENARIO_LABELS = {
    "standard_stop": "标准站点停靠 (60 km/h)",
    "highspeed_stop": "高速站点停靠 (80 km/h)",
    "short_platform": "短站台停靠 (40 km/h)",
    "switch_lane": "道岔换道 (8°)",
    "uturn_180": "180° 弯道掉头 (R=15m)",
}


def rmse(vals):
    if not vals:
        return 0.0
    return math.sqrt(sum(v**2 for v in vals) / len(vals))


def mae(vals):
    if not vals:
        return 0.0
    return sum(abs(v) for v in vals) / len(vals)


def max_abs(vals):
    if not vals:
        return 0.0
    return max(abs(v) for v in vals)


def load_csv(path: Path):
    rows = []
    with open(path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                rows.append({k: float(v) if k != 'scenario' else v for k, v in row.items()})
            except ValueError:
                continue
    return rows


def analyse_scenario(sid: str, rows: list):
    if not rows:
        return {}

    t0 = rows[0]['time']
    times = [r['time'] - t0 for r in rows]

    # Speed tracking
    v_act = [r['speed_actual'] for r in rows]
    v_ref = [r['speed_ref'] for r in rows]
    v_err = [a - b for a, b in zip(v_act, v_ref)]

    # Lateral / heading
    lat_err = [r['lateral_error'] for r in rows]
    head_err = [r['heading_error'] for r in rows]

    # Steering
    steer_f = [r['steering_front'] for r in rows]
    steer_r = [r['steering_rear'] for r in rows]

    # Accel
    accel = [r['accel_cmd'] for r in rows]

    # Distance travelled (approximate via hypot of x,y deltas)
    dist = 0.0
    for i in range(1, len(rows)):
        dx = rows[i]['x'] - rows[i-1]['x']
        dy = rows[i]['y'] - rows[i-1]['y']
        dist += math.hypot(dx, dy)

    # Curvature-weighted lateral error (extract high-curvature segments)
    curve_lat_err = []
    for i in range(1, len(rows)-1):
        dpsi = abs(rows[i+1]['psi'] - rows[i-1]['psi'])
        if dpsi > 0.02:  # roughly >1° change over 2 samples
            curve_lat_err.append(abs(lat_err[i]))

    return {
        'sid': sid,
        'label': SCENARIO_LABELS.get(sid, sid),
        'duration': times[-1],
        'distance': dist,
        'samples': len(rows),
        'v_rmse': rmse(v_err),
        'v_mae': mae(v_err),
        'v_maxe': max_abs(v_err),
        'lat_rmse': rmse(lat_err),
        'lat_mae': mae(lat_err),
        'lat_max': max_abs(lat_err),
        'curve_lat_max': max(curve_lat_err) if curve_lat_err else 0.0,
        'head_rmse': rmse(head_err),
        'head_mae': mae(head_err),
        'head_max': max_abs(head_err),
        'steer_f_max': max(abs(v) for v in steer_f) if steer_f else 0.0,
        'steer_r_max': max(abs(v) for v in steer_r) if steer_r else 0.0,
        'accel_max': max(accel) if accel else 0.0,
        'accel_min': min(accel) if accel else 0.0,
        'avg_speed': sum(v_act) / len(v_act) if v_act else 0.0,
        # Raw series for plotting
        'times': times,
        'v_act': v_act,
        'v_ref': v_ref,
        'lat_err': lat_err,
        'head_err': head_err,
        'steer_f': steer_f,
        'steer_r': steer_r,
        'accel': accel,
        'x': [r['x'] for r in rows],
        'y': [r['y'] for r in rows],
    }


def _save_figure(fig, path: Path):
    """Save with tight layout, no extra padding."""
    fig.savefig(path, dpi=150, bbox_inches='tight', pad_inches=0.15)
    plt.close(fig)


def generate_plots(results: list):
    if not HAS_MPL:
        print("[report] matplotlib not available, skipping plots")
        return

    # SCI / academic plotting style
    sns.set_theme(style="whitegrid", context="paper", font_scale=1.3,
                  rc={
                      "axes.linewidth": 0.8,
                      "axes.edgecolor": "0.15",
                      "axes.labelcolor": "0.15",
                      "xtick.color": "0.15",
                      "ytick.color": "0.15",
                      "grid.linewidth": 0.5,
                      "grid.alpha": 0.4,
                      "lines.linewidth": 1.8,
                      "legend.frameon": True,
                      "legend.edgecolor": "0.15",
                  })
    # Professional academic colour palette
    C_PRIMARY = '#2E5AAC'      # deep blue
    C_SECONDARY = '#C44E52'    # deep red
    C_TERTIARY = '#55A868'     # deep green
    C_QUAT = '#DD8452'         # orange

    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    for res in results:
        sid = res['sid']
        out_dir = REPORT_DIR / sid
        out_dir.mkdir(parents=True, exist_ok=True)
        t = res['times']

        # 1) speed_tracking.png
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, res['v_act'], color=C_PRIMARY, label='actual')
        ax.plot(t, res['v_ref'], color=C_SECONDARY, ls='--', label='ref')
        ax.set_xlabel("time (s)")
        ax.set_ylabel("v (m/s)")
        ax.legend(loc='upper right')
        _save_figure(fig, out_dir / "speed_tracking.png")

        # 2) lateral_error.png
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, res['lat_err'], color=C_PRIMARY)
        ax.axhline(0, color='0.3', lw=0.6, ls='--')
        ax.set_xlabel("time (s)")
        ax.set_ylabel("e_y (m)")
        _save_figure(fig, out_dir / "lateral_error.png")

        # 3) heading_error.png
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, [math.degrees(v) for v in res['head_err']], color=C_PRIMARY)
        ax.axhline(0, color='0.3', lw=0.6, ls='--')
        ax.set_xlabel("time (s)")
        ax.set_ylabel("e_ψ (deg)")
        _save_figure(fig, out_dir / "heading_error.png")

        # 4) steering.png
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, [math.degrees(v) for v in res['steer_f']], color=C_PRIMARY, label='front')
        ax.plot(t, [math.degrees(v) for v in res['steer_r']], color=C_SECONDARY, label='rear')
        ax.set_xlabel("time (s)")
        ax.set_ylabel("δ (deg)")
        ax.legend(loc='upper right')
        _save_figure(fig, out_dir / "steering.png")

        # 5) trajectory.png
        fig, ax = plt.subplots(figsize=(6, 5))
        ax.plot(res['x'], res['y'], color=C_PRIMARY, lw=1.8)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")

        # Fix: detect near-straight-line trajectories and avoid squashed y-axis
        y_vals = res['y']
        x_vals = res['x']
        y_range = max(y_vals) - min(y_vals)
        x_range = max(x_vals) - min(x_vals)
        ratio = y_range / x_range if x_range > 0 else 1.0

        if ratio < 0.08:
            # Nearly straight line — give y a minimum visible span
            ax.set_aspect('auto')
            y_mid = (max(y_vals) + min(y_vals)) / 2.0
            margin = max(3.0, y_range * 0.5 + 1.0)
            ax.set_ylim(y_mid - margin, y_mid + margin)
        elif ratio < 0.25:
            # Elongated (e.g. switch_lane) — auto aspect so y is readable
            ax.set_aspect('auto')
        else:
            # Normal curved path — equal aspect keeps geometry true
            ax.set_aspect('equal', adjustable='box')
        _save_figure(fig, out_dir / "trajectory.png")

        # 6) accel.png
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, res['accel'], color=C_PRIMARY)
        ax.axhline(0, color='0.3', lw=0.6, ls='--')
        ax.set_xlabel("time (s)")
        ax.set_ylabel("a_cmd (m/s²)")
        _save_figure(fig, out_dir / "accel.png")

        print(f"[report] plots saved: {sid}/")


def generate_markdown(results: list) -> str:
    lines = [
        "# Tram 自动驾驶场景测试报告",
        "",
        f"**生成时间**: 自动生成  ",
        f"**测试场景数**: {len(results)}  ",
        "",
        "## 1. 实验配置",
        "",
        "| 配置项 | 值 |",
        "|--------|-----|",
        "| 控制器 (标准/高速/短站台/道岔) | PI + LQR |",
        "| 控制器 (U-turn) | MPC |",
        "| 仿真频率 | 100 Hz |",
        "| 控制频率 (PI+LQR) | 50 Hz |",
        "| 控制频率 (MPC) | 20 Hz |",
        "| 数据记录频率 | 20 Hz |",
        "",
        "## 2. 综合指标汇总",
        "",
        "| 场景 | 时长(s) | 距离(m) | 均速(m/s) | 横向误差 RMSE(m) | 横向误差 Max(m) | 航向误差 RMSE(°) | 航向误差 Max(°) | 速度跟踪 RMSE(m/s) |",
        "|------|---------|---------|-----------|------------------|-----------------|------------------|-----------------|--------------------|",
    ]
    for res in results:
        lines.append(
            f"| {res['label']} | {res['duration']:.1f} | {res['distance']:.1f} | "
            f"{res['avg_speed']:.2f} | {res['lat_rmse']:.3f} | {res['lat_max']:.3f} | "
            f"{math.degrees(res['head_rmse']):.2f} | {math.degrees(res['head_max']):.2f} | "
            f"{res['v_rmse']:.3f} |"
        )

    lines += ["", "## 3. 各场景详细指标", ""]
    for res in results:
        lines += [
            f"### {res['label']}",
            "",
            "| 指标 | 值 |",
            "|------|-----|",
            f"| 运行时长 | {res['duration']:.2f} s |",
            f"| 行驶距离 | {res['distance']:.2f} m |",
            f"| 采样点数 | {res['samples']} |",
            f"| 平均车速 | {res['avg_speed']:.2f} m/s |",
            f"| 速度跟踪 RMSE | {res['v_rmse']:.3f} m/s |",
            f"| 速度跟踪 MAE | {res['v_mae']:.3f} m/s |",
            f"| 速度跟踪 Max Error | {res['v_maxe']:.3f} m/s |",
            f"| 横向误差 RMSE | {res['lat_rmse']:.3f} m |",
            f"| 横向误差 MAE | {res['lat_mae']:.3f} m |",
            f"| 横向误差 Max | {res['lat_max']:.3f} m |",
            f"| 弯道段横向误差 Max | {res['curve_lat_max']:.3f} m |",
            f"| 航向误差 RMSE | {math.degrees(res['head_rmse']):.2f} ° |",
            f"| 航向误差 MAE | {math.degrees(res['head_mae']):.2f} ° |",
            f"| 航向误差 Max | {math.degrees(res['head_max']):.2f} ° |",
            f"| 最大前轮转角 | {math.degrees(res['steer_f_max']):.1f} ° |",
            f"| 最大后轮转角 | {math.degrees(res['steer_r_max']):.1f} ° |",
            f"| 最大加速度指令 | {res['accel_max']:.2f} m/s² |",
            f"| 最大减速度指令 | {res['accel_min']:.2f} m/s² |",
            "",
        ]
        if HAS_MPL:
            sid = res['sid']
            lines.append(f"![speed]({sid}/speed_tracking.png)")
            lines.append(f"![lateral]({sid}/lateral_error.png)")
            lines.append(f"![heading]({sid}/heading_error.png)")
            lines.append(f"![steering]({sid}/steering.png)")
            lines.append(f"![trajectory]({sid}/trajectory.png)")
            lines.append(f"![accel]({sid}/accel.png)")
            lines.append("")

    lines += [
        "## 4. 结论与建议",
        "",
        "- **标准站点停靠**、**高速站点停靠**、**短站台停靠**：主要考核纵向速度跟踪与精准停车能力。",
        "- **道岔换道**：考核横向跟踪与前后轮协同，关注横向误差峰值是否收敛。",
        "- **180° 弯道掉头**：考核大曲率横向跟踪与姿态恢复，MPC 相比 LQR 在曲率阶跃处具有更好的预测能力。",
        "- 若某场景横向误差 RMSE > 0.15 m 或出现持续发散，建议进一步调大 LQR/MPC 的 e_y 权重，或降低该场景参考速度。",
        "",
    ]
    return "\n".join(lines)


def main():
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    results = []
    for sid in SCENARIO_LABELS.keys():
        csv_path = EXP_DIR / f"{sid}.csv"
        if not csv_path.exists():
            print(f"[report] SKIP {sid}: CSV not found")
            continue
        rows = load_csv(csv_path)
        res = analyse_scenario(sid, rows)
        results.append(res)
        print(f"[report] analysed {sid}: {res['samples']} samples, {res['duration']:.1f}s")

    generate_plots(results)

    md = generate_markdown(results)
    report_path = REPORT_DIR / "report.md"
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(md)
    print(f"[report] Markdown saved -> {report_path}")

    # Also print summary to console
    print("\n" + "="*80)
    print("SUMMARY")
    print("="*80)
    for res in results:
        print(f"\n{res['label']}")
        print(f"  duration={res['duration']:.1f}s  distance={res['distance']:.1f}m  "
              f"lat_rmse={res['lat_rmse']:.3f}m  lat_max={res['lat_max']:.3f}m  "
              f"head_max={math.degrees(res['head_max']):.2f}°  v_rmse={res['v_rmse']:.3f}m/s")


if __name__ == '__main__':
    main()
