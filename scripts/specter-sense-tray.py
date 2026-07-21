#!/usr/bin/python3
"""Qt/StatusNotifier tray indicator for the specter-sense user service."""

import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import threading
import time

from PyQt6.QtCore import QObject, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QIcon
from PyQt6.QtWidgets import QApplication, QMenu, QSystemTrayIcon


SERVICE = "specter-sense.service"
ROOT = Path(__file__).resolve().parent.parent
ICONS = ROOT / "assets" / "tray"


def read_dotenv(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        return values
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        if line.startswith("export "):
            line = line[7:].lstrip()
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip("'\"")
    return values


def socket_path() -> str:
    env_path = Path(os.environ.get("SPECTER_SENSE_ENV", ROOT / ".env"))
    configured = os.environ.get("SPECTER_SENSE_SOCKET") or read_dotenv(env_path).get("SPECTER_SENSE_SOCKET")
    if configured:
        return configured
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    return str(Path(runtime) / "specter-sense.sock") if runtime else f"/tmp/specter-sense-{os.getuid()}.sock"


def service_active() -> bool:
    result = subprocess.run(
        ["systemctl", "--user", "is-active", "--quiet", SERVICE],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


class StateBridge(QObject):
    changed = pyqtSignal(str, str)


class Tray:
    def __init__(self, app: QApplication) -> None:
        self.app = app
        self.state = "stopped"
        self.detail = "Sensor service is stopped"
        self.bridge = StateBridge()
        self.bridge.changed.connect(self.update)

        self.icon = QSystemTrayIcon()
        self.icon.setContextMenu(self.make_menu())
        self.icon.activated.connect(self.activated)
        self.update(self.state, self.detail)
        self.icon.show()

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_service)
        self.timer.start(3000)
        threading.Thread(target=self.watch_socket, daemon=True).start()

    def make_menu(self) -> QMenu:
        menu = QMenu()
        self.summary_action = QAction(self.detail, menu)
        self.summary_action.setEnabled(False)
        menu.addAction(self.summary_action)
        menu.addSeparator()
        self.toggle_action = QAction("Start sensor", menu)
        self.toggle_action.triggered.connect(self.toggle)
        menu.addAction(self.toggle_action)
        logs_action = QAction("Open recent logs", menu)
        logs_action.triggered.connect(self.open_logs)
        menu.addAction(logs_action)
        menu.addSeparator()
        quit_action = QAction("Quit tray", menu)
        quit_action.triggered.connect(self.app.quit)
        menu.addAction(quit_action)
        return menu

    def update(self, state: str, detail: str) -> None:
        self.state, self.detail = state, detail
        self.icon.setIcon(QIcon(str(ICONS / f"specter-sense-{state}.svg")))
        self.icon.setToolTip(f"specter-sense: {detail}")
        self.summary_action.setText(detail)
        self.toggle_action.setText("Stop sensor" if service_active() else "Start sensor")

    def refresh_service(self) -> None:
        if not service_active() and self.state != "stopped":
            self.update("stopped", "Sensor service is stopped")

    def watch_socket(self) -> None:
        path = socket_path()
        while True:
            if not service_active():
                self.bridge.changed.emit("stopped", "Sensor service is stopped")
                time.sleep(1)
                continue
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                    client.settimeout(3)
                    client.connect(path)
                    stream = client.makefile("r", encoding="utf-8")
                    for line in stream:
                        message = json.loads(line)
                        snapshot = message.get("state", {})
                        sensors = snapshot.get("sensors", {})
                        statuses = {
                            name: sensor.get("health", {}).get("status", "unknown")
                            for name, sensor in sensors.items()
                        }
                        occupied = [
                            f"{sensor_name}/{zone_name}"
                            for sensor_name, sensor in sensors.items()
                            for zone_name, zone in sensor.get("zones", {}).items()
                            if zone.get("occupied")
                        ]
                        unhealthy = [f"{name}: {status.replace('_', ' ')}"
                                     for name, status in statuses.items() if status != "streaming"]
                        if sensors and not unhealthy:
                            state = "occupied" if occupied else "clear"
                            detail = "Occupied: " + ", ".join(occupied) if occupied else "Streaming; all zones clear"
                            plane_activity = []
                            for sensor_name, sensor in sensors.items():
                                for name, plane in sensor.get("ignore_planes", {}).items():
                                    threshold = plane.get("noise_threshold_points")
                                    if threshold is None or not plane.get("enabled", False):
                                        continue
                                    activity = int(plane.get("activity_points", 0))
                                    disposition = "passing" if activity > threshold else "suppressed"
                                    plane_activity.append(
                                        f"{sensor_name}/{name} {activity}/{threshold} {disposition}")
                            if plane_activity:
                                detail += "; planes: " + ", ".join(plane_activity)
                        else:
                            state, detail = "warning", "; ".join(unhealthy) if unhealthy else "No sensors configured"
                        self.bridge.changed.emit(state, detail)
            except (FileNotFoundError, ConnectionError, OSError, json.JSONDecodeError):
                self.bridge.changed.emit("warning", "Service active; waiting for sensor socket")
                time.sleep(1)

    def activated(self, reason: QSystemTrayIcon.ActivationReason) -> None:
        if reason == QSystemTrayIcon.ActivationReason.Trigger:
            self.toggle()

    def toggle(self) -> None:
        action = "stop" if service_active() else "start"
        subprocess.Popen(["systemctl", "--user", action, SERVICE])
        self.update("warning", f"Requesting service {action}")

    def open_logs(self) -> None:
        terminal = shutil.which("konsole") or shutil.which("x-terminal-emulator")
        if terminal:
            subprocess.Popen([terminal, "-e", "journalctl", "--user", "-u", SERVICE, "-f", "-n", "40"])


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("specter-sense")
    app.setQuitOnLastWindowClosed(False)
    if not QSystemTrayIcon.isSystemTrayAvailable():
        print("specter-sense tray: no system tray is available in this session", file=sys.stderr)
        return 1
    tray = Tray(app)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
