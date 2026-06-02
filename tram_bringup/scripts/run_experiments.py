#!/usr/bin/env python3
"""Automated experiment runner for all tram scenarios.

For each scenario:
  1. Launch data_logger.py
  2. Launch the scenario (rviz:=false)
  3. Wait for data_logger to auto-detect completion
  4. Terminate launch, save CSV
  5. Move to next scenario

Usage:
  python3 run_experiments.py
"""
import subprocess
import time
import sys
import signal
from pathlib import Path

WS_PATH = Path("/home/cyun/Documents/project/tram_ws")
SETUP_BASH = WS_PATH / "devel" / "setup.bash"
SCRIPT_DIR = WS_PATH / "src" / "tram_bringup" / "scripts"
OUTPUT_DIR = WS_PATH / "src" / "tram_bringup" / "experiments"

SCENARIOS = [
    {"id": "standard_stop",  "launch": "tram_bringup/launch/sim_pi_lqr.launch", "timeout": 120},
    {"id": "highspeed_stop", "launch": "tram_bringup/launch/sim_pi_lqr.launch", "timeout": 120},
    {"id": "short_platform", "launch": "tram_bringup/launch/sim_pi_lqr.launch", "timeout": 120},
    {"id": "switch_lane",    "launch": "tram_bringup/launch/sim_pi_lqr.launch", "timeout": 120},
    {"id": "uturn_180",      "launch": "tram_bringup/launch/sim_mpc.launch",    "timeout": 150},
]


def _bash_cmd(cmd_str: str):
    return ["bash", "-c", f"source {SETUP_BASH} && {cmd_str}"]


def run_scenario(scenario: dict, roscore_already_running: bool = False):
    sid = scenario["id"]
    launch_file = scenario["launch"]
    timeout = scenario["timeout"]
    csv_path = OUTPUT_DIR / f"{sid}.csv"

    # Clean old CSV
    csv_path.unlink(missing_ok=True)

    print(f"\n{'='*60}")
    print(f"[EXPERIMENT] Starting scenario: {sid}")
    print(f"{'='*60}")

    # Start data logger
    logger_cmd = _bash_cmd(f"python3 {SCRIPT_DIR}/data_logger.py {csv_path}")
    logger_proc = subprocess.Popen(
        logger_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        preexec_fn=lambda: signal.signal(signal.SIGTERM, signal.SIG_DFL)
    )
    time.sleep(2.5)  # Give logger time to initialise and subscribe

    # Parse launch file
    parts = launch_file.split('/')
    pkg = parts[0]
    launch_name = parts[-1]

    # Start scenario
    launch_cmd = _bash_cmd(
        f"roslaunch {pkg} {launch_name} scenario:={sid} rviz:=false"
    )
    launch_proc = subprocess.Popen(
        launch_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        preexec_fn=lambda: signal.signal(signal.SIGTERM, signal.SIG_DFL)
    )

    start = time.time()
    completed = False
    try:
        # Wait for logger to auto-detect completion and exit
        try:
            logger_proc.wait(timeout=timeout)
            completed = True
            print(f"[EXPERIMENT] {sid} completed naturally ({time.time()-start:.1f}s)")
        except subprocess.TimeoutExpired:
            print(f"[EXPERIMENT] {sid} TIMEOUT after {timeout}s — forcing stop")
    except KeyboardInterrupt:
        print(f"[EXPERIMENT] {sid} interrupted by user")
    finally:
        # Stop launch first
        if launch_proc.poll() is None:
            launch_proc.terminate()
            try:
                launch_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                launch_proc.kill()
                launch_proc.wait()

        # Stop logger if still alive
        if logger_proc.poll() is None:
            logger_proc.terminate()
            try:
                logger_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger_proc.kill()
                logger_proc.wait()

        time.sleep(1.0)

    if csv_path.exists() and csv_path.stat().st_size > 128:
        print(f"[EXPERIMENT] {sid} data saved -> {csv_path}")
    else:
        print(f"[EXPERIMENT] {sid} WARNING: CSV missing or empty")
    return completed


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Start roscore
    print("[EXPERIMENT] Starting roscore...")
    roscore_proc = subprocess.Popen(
        _bash_cmd("roscore"),
        stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    time.sleep(4.0)

    results = {}
    try:
        for sc in SCENARIOS:
            ok = run_scenario(sc)
            results[sc["id"]] = ok
            time.sleep(2.0)  # Brief pause between scenarios
    finally:
        print("\n[EXPERIMENT] Stopping roscore...")
        roscore_proc.terminate()
        try:
            roscore_proc.wait(timeout=5)
        except Exception:
            roscore_proc.kill()

    print(f"\n{'='*60}")
    print("[EXPERIMENT] All scenarios finished")
    for sid, ok in results.items():
        status = "OK" if ok else "TIMEOUT/FAILED"
        print(f"  {sid:20s} {status}")
    print(f"{'='*60}")
    print(f"Data directory: {OUTPUT_DIR}")


if __name__ == '__main__':
    main()
