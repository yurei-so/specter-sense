#!/usr/bin/python3
"""Small GTK status icon for the specter-sense user service."""

import json
import os
from pathlib import Path
import socket
import subprocess
import threading
import time

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk  # noqa: E402


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


class Tray:
    def __init__(self) -> None:
        self.status = Gtk.StatusIcon()
        self.status.set_title("specter-sense")
        self.status.connect("popup-menu", self.show_menu)
        self.status.connect("activate", self.toggle)
        self.state = "stopped"
        self.detail = "Sensor service is stopped"
        self.update("stopped", self.detail)
        threading.Thread(target=self.watch_socket, daemon=True).start()
        GLib.timeout_add_seconds(3, self.refresh_service)

    def update(self, state: str, detail: str) -> bool:
        self.state, self.detail = state, detail
        icon = ICONS / f"specter-sense-{state}.svg"
        self.status.set_from_file(str(icon))
        self.status.set_tooltip_text(f"specter-sense: {detail}")
        self.status.set_visible(True)
        return False

    def refresh_service(self) -> bool:
        if not service_active() and self.state != "stopped":
            self.update("stopped", "Sensor service is stopped")
        return True

    def watch_socket(self) -> None:
        path = socket_path()
        while True:
            if not service_active():
                GLib.idle_add(self.update, "stopped", "Sensor service is stopped")
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
                        sensor = snapshot.get("sensor", {})
                        status = sensor.get("status", "unknown")
                        zones = snapshot.get("zones", {})
                        occupied = [name for name, zone in zones.items() if zone.get("occupied")]
                        if status == "streaming":
                            state = "occupied" if occupied else "clear"
                            detail = "Occupied: " + ", ".join(occupied) if occupied else "Streaming; all zones clear"
                        else:
                            state, detail = "warning", f"Sensor {status.replace('_', ' ')}"
                        GLib.idle_add(self.update, state, detail)
            except (FileNotFoundError, ConnectionError, OSError, json.JSONDecodeError):
                GLib.idle_add(self.update, "warning", "Service active; waiting for sensor socket")
                time.sleep(1)

    def toggle(self, *_args) -> None:
        action = "stop" if service_active() else "start"
        subprocess.Popen(["systemctl", "--user", action, SERVICE])
        self.update("warning", f"Requesting service {action}")

    def show_menu(self, _icon, button: int, activate_time: int) -> None:
        menu = Gtk.Menu()
        summary = Gtk.MenuItem(label=self.detail)
        summary.set_sensitive(False)
        menu.append(summary)
        menu.append(Gtk.SeparatorMenuItem())
        toggle = Gtk.MenuItem(label="Stop sensor" if service_active() else "Start sensor")
        toggle.connect("activate", self.toggle)
        menu.append(toggle)
        logs = Gtk.MenuItem(label="Open recent logs")
        logs.connect("activate", lambda *_: subprocess.Popen(["journalctl", "--user", "-u", SERVICE, "-f", "-n", "40"]))
        menu.append(logs)
        quit_item = Gtk.MenuItem(label="Quit tray")
        quit_item.connect("activate", lambda *_: Gtk.main_quit())
        menu.append(quit_item)
        menu.show_all()
        menu.popup(None, None, Gtk.StatusIcon.position_menu, self.status, button, activate_time)


if __name__ == "__main__":
    Tray()
    Gtk.main()
