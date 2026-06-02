#!/usr/bin/env python3
"""Tram Scenario Launcher — GUI for starting/stopping ROS launch scenarios."""

import sys
import os
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QTextEdit, QGroupBox, QGridLayout,
    QSizePolicy, QMessageBox
)
from PyQt5.QtCore import QProcess, Qt, pyqtSignal
from PyQt5.QtGui import QFont, QColor, QPalette


WS_PATH = Path("/home/cyun/Documents/project/tram_ws")
SETUP_BASH = WS_PATH / "devel" / "setup.bash"

SCENARIOS = [
    {
        "id": "standard_stop",
        "name": "标准站点停靠",
        "desc": "60 km/h → 停 → 60 km/h\n(150m 进站 / 60m 减速 / 30m 站台 / 60m 加速 / 150m 出站)",
        "launch": "tram_bringup/launch/sim_pi_lqr.launch",
        "ctrl": "PI+LQR",
    },
    {
        "id": "highspeed_stop",
        "name": "高速站点停靠",
        "desc": "80 km/h → 停 → 80 km/h\n(80m 进站 / 60m 减速 / 30m 站台 / 60m 加速 / 150m 出站)",
        "launch": "tram_bringup/launch/sim_pi_lqr.launch",
        "ctrl": "PI+LQR",
    },
    {
        "id": "short_platform",
        "name": "短站台停靠",
        "desc": "40 km/h → 停 → 40 km/h\n(60m 进站 / 60m 减速 / 20m 站台 / 60m 加速 / 80m 出站)",
        "launch": "tram_bringup/launch/sim_pi_lqr.launch",
        "ctrl": "PI+LQR",
    },
    {
        "id": "switch_lane",
        "name": "道岔换道",
        "desc": "8° 分岔换道\n(150m 主轨 / 60m 过渡 / 150m 侧线)",
        "launch": "tram_bringup/launch/sim_pi_lqr.launch",
        "ctrl": "PI+LQR",
    },
    {
        "id": "uturn_180",
        "name": "180° 弯道掉头",
        "desc": "R=15m 半圆掉头\n(80m 直线 / 半圆 / 80m 反向直线)",
        "launch": "tram_bringup/launch/sim_mpc.launch",
        "ctrl": "MPC",
    },
]

RVIZ_CONFIG = "$(find tram_bringup)/config/tram_sim.rviz"


class ScenarioCard(QGroupBox):
    status_changed = pyqtSignal(str, str)

    def __init__(self, info, parent=None):
        super().__init__(parent)
        self.info = info
        self.process = None
        self._setup_ui()

    def _setup_ui(self):
        self.setTitle(f"{self.info['name']}  [{self.info['ctrl']}]")
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #aaaaaa;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
        """)

        layout = QGridLayout(self)
        layout.setColumnStretch(1, 1)

        # Description
        self.lbl_desc = QLabel(self.info["desc"])
        self.lbl_desc.setWordWrap(True)
        self.lbl_desc.setStyleSheet("color: #555555; font-size: 12px;")
        layout.addWidget(self.lbl_desc, 0, 0, 1, 3)

        # Status
        self.lbl_status = QLabel("● 已停止")
        self.lbl_status.setStyleSheet("color: #cc0000; font-weight: bold;")
        layout.addWidget(self.lbl_status, 1, 0)

        # Buttons
        self.btn_start = QPushButton("▶  启动")
        self.btn_start.setStyleSheet("""
            QPushButton {
                background-color: #28a745;
                color: white;
                font-weight: bold;
                padding: 6px 16px;
                border-radius: 4px;
            }
            QPushButton:hover { background-color: #218838; }
            QPushButton:pressed { background-color: #1e7e34; }
        """)
        self.btn_start.clicked.connect(self.start)
        layout.addWidget(self.btn_start, 1, 1)

        self.btn_stop = QPushButton("■  停止")
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet("""
            QPushButton {
                background-color: #dc3545;
                color: white;
                font-weight: bold;
                padding: 6px 16px;
                border-radius: 4px;
            }
            QPushButton:hover { background-color: #c82333; }
            QPushButton:pressed { background-color: #bd2130; }
            QPushButton:disabled {
                background-color: #e0e0e0;
                color: #999999;
            }
        """)
        self.btn_stop.clicked.connect(self.stop)
        layout.addWidget(self.btn_stop, 1, 2)

        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        self.setMinimumHeight(90)

    def start(self):
        if self.process is not None and self.process.state() != QProcess.NotRunning:
            return

        launch_parts = self.info['launch'].split('/')
        pkg = launch_parts[0]
        launch_file = launch_parts[-1]
        cmd = (
            f"export DISABLE_ROS1_EOL_WARNINGS=1 && "
            f"source {SETUP_BASH} && "
            f"roslaunch {pkg} {launch_file} scenario:={self.info['id']}"
        )
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._on_output)
        self.process.finished.connect(self._on_finished)

        self.process.start("bash", ["-c", cmd])
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.lbl_status.setText("● 运行中")
        self.lbl_status.setStyleSheet("color: #28a745; font-weight: bold;")
        self.status_changed.emit(self.info["id"], f"[启动] {self.info['name']}")

    def stop(self):
        if self.process is None:
            return
        if self.process.state() == QProcess.NotRunning:
            self._reset_ui()
            return

        self.status_changed.emit(self.info["id"], f"[停止] {self.info['name']}")
        self.process.terminate()
        if not self.process.waitForFinished(3000):
            self.process.kill()
        self._reset_ui()

    def _on_output(self):
        data = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        for line in data.splitlines():
            self.status_changed.emit(self.info["id"], line)

    def _on_finished(self, exit_code, exit_status):
        self._reset_ui()
        self.status_changed.emit(
            self.info["id"],
            f"[退出] {self.info['name']} (code={exit_code}, status={exit_status})"
        )

    def _reset_ui(self):
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.lbl_status.setText("● 已停止")
        self.lbl_status.setStyleSheet("color: #cc0000; font-weight: bold;")
        self.process = None


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tram 场景测试启动器")
        self.setMinimumSize(800, 850)
        self.resize(850, 950)
        self._setup_ui()
        self.rviz_process = None
        self.rqt_process = None

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(12)
        main_layout.setContentsMargins(16, 16, 16, 16)

        # Header
        header = QLabel("Tram 自动驾驶场景测试控制台")
        header.setFont(QFont("Microsoft YaHei", 18, QFont.Bold))
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("color: #333333; margin-bottom: 8px;")
        main_layout.addWidget(header)

        # Global controls
        global_box = QHBoxLayout()
        global_box.addStretch()

        self.btn_stop_all = QPushButton("■  停止全部场景")
        self.btn_stop_all.setStyleSheet("""
            QPushButton {
                background-color: #6c757d;
                color: white;
                font-weight: bold;
                padding: 8px 20px;
                border-radius: 4px;
            }
            QPushButton:hover { background-color: #5a6268; }
        """)
        self.btn_stop_all.clicked.connect(self.stop_all)
        global_box.addWidget(self.btn_stop_all)

        self.btn_rviz = QPushButton("👁  启动 RViz")
        self.btn_rviz.setStyleSheet("""
            QPushButton {
                background-color: #17a2b8;
                color: white;
                font-weight: bold;
                padding: 8px 20px;
                border-radius: 4px;
            }
            QPushButton:hover { background-color: #138496; }
        """)
        self.btn_rviz.clicked.connect(self.toggle_rviz)
        global_box.addWidget(self.btn_rviz)

        self.btn_rqt = QPushButton("📈  启动 rqt_plot")
        self.btn_rqt.setStyleSheet("""
            QPushButton {
                background-color: #fd7e14;
                color: white;
                font-weight: bold;
                padding: 8px 20px;
                border-radius: 4px;
            }
            QPushButton:hover { background-color: #e36c0a; }
        """)
        self.btn_rqt.clicked.connect(self.toggle_rqt)
        global_box.addWidget(self.btn_rqt)

        global_box.addStretch()
        main_layout.addLayout(global_box)

        # Scenario cards
        cards_layout = QVBoxLayout()
        self.cards = []
        for info in SCENARIOS:
            card = ScenarioCard(info)
            card.status_changed.connect(self.log)
            cards_layout.addWidget(card)
            self.cards.append(card)
        main_layout.addLayout(cards_layout)

        # Log area
        log_box = QGroupBox("运行日志")
        log_layout = QVBoxLayout(log_box)
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setFont(QFont("Consolas", 10))
        self.log_edit.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #d4d4d4;
                border: 1px solid #444444;
                border-radius: 4px;
                padding: 6px;
            }
        """)
        self.log_edit.document().setMaximumBlockCount(500)
        self.log_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.log_edit.setMinimumHeight(180)
        self.log_edit.setMaximumHeight(400)
        log_layout.addWidget(self.log_edit)
        main_layout.addWidget(log_box, 1)

        # Status bar hint
        hint = QLabel("提示: 启动前请确保 roscore 已在运行 (或某场景启动后会自动拉起)。先 source devel/setup.bash 再运行本脚本可获得最佳兼容性。")
        hint.setStyleSheet("color: #888888; font-size: 11px;")
        hint.setWordWrap(True)
        main_layout.addWidget(hint)

    def log(self, scenario_id, message):
        prefix = f"[{scenario_id}]"
        self.log_edit.append(f"{prefix:18s} {message}")

    def stop_all(self):
        for card in self.cards:
            card.stop()
        self.log("system", "已请求停止所有场景")

    def toggle_rviz(self):
        if self.rviz_process is not None and self.rviz_process.state() != QProcess.NotRunning:
            self.rviz_process.terminate()
            if not self.rviz_process.waitForFinished(2000):
                self.rviz_process.kill()
            self.rviz_process = None
            self.btn_rviz.setText("👁  启动 RViz")
            self.log("system", "RViz 已停止")
            return

        cmd = f"export DISABLE_ROS1_EOL_WARNINGS=1 && source {SETUP_BASH} && rviz -d {RVIZ_CONFIG}"
        self.rviz_process = QProcess(self)
        self.rviz_process.start("bash", ["-c", cmd])
        self.btn_rviz.setText("■  停止 RViz")
        self.log("system", "RViz 已启动")

    def toggle_rqt(self):
        if self.rqt_process is not None and self.rqt_process.state() != QProcess.NotRunning:
            self.rqt_process.terminate()
            if not self.rqt_process.waitForFinished(2000):
                self.rqt_process.kill()
            self.rqt_process = None
            self.btn_rqt.setText("📈  启动 rqt_plot")
            self.log("system", "rqt_plot 已停止")
            return

        cmd = (
            f"export DISABLE_ROS1_EOL_WARNINGS=1 && "
            f"source {SETUP_BASH} && "
            "rqt_plot /debug/speed_actual/data /debug/speed_ref/data /debug/lateral_error/data"
        )
        self.rqt_process = QProcess(self)
        self.rqt_process.start("bash", ["-c", cmd])
        self.btn_rqt.setText("■  停止 rqt_plot")
        self.log("system", "rqt_plot 已启动")

    def closeEvent(self, event):
        reply = QMessageBox.question(
            self, "确认退出",
            "退出将停止所有正在运行的场景进程，是否继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.stop_all()
            if self.rviz_process:
                self.rviz_process.terminate()
            if self.rqt_process:
                self.rqt_process.terminate()
            event.accept()
        else:
            event.ignore()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark-ish palette for Fusion
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(245, 245, 245))
    palette.setColor(QPalette.WindowText, QColor(30, 30, 30))
    palette.setColor(QPalette.Base, QColor(255, 255, 255))
    palette.setColor(QPalette.AlternateBase, QColor(233, 233, 233))
    palette.setColor(QPalette.ToolTipBase, QColor(255, 255, 255))
    palette.setColor(QPalette.ToolTipText, QColor(30, 30, 30))
    palette.setColor(QPalette.Text, QColor(30, 30, 30))
    palette.setColor(QPalette.Button, QColor(240, 240, 240))
    palette.setColor(QPalette.ButtonText, QColor(30, 30, 30))
    palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
    palette.setColor(QPalette.Highlight, QColor(0, 120, 215))
    palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)

    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
