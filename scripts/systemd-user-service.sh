#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="$repo_dir/build/systemd"
install_units=false
start_units=true

usage() {
  echo "Usage: $0 [--install] [--no-start] [--output-dir DIR]"
  echo "Generates sensor and tray user units; --install copies and enables them."
}

while (($#)); do
  case "$1" in
    --install) install_units=true ;;
    --no-start) start_units=false ;;
    --output-dir)
      shift
      (($#)) || { echo "missing value after --output-dir" >&2; exit 2; }
      output_dir="$1"
      ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

binary="$repo_dir/build/specter-sense"
tray="$repo_dir/scripts/specter-sense-tray.py"
env_file="$repo_dir/.env"
if [[ ! -x "$binary" ]]; then
  echo "Build the project first; executable not found: $binary" >&2
  exit 1
fi
if [[ ! -f "$env_file" ]]; then
  cp "$repo_dir/.env.example" "$env_file"
  echo "Created $env_file from .env.example; review it before using real hardware."
fi

mkdir -p "$output_dir"
sed -e "s|@REPO_DIR@|$repo_dir|g" -e "s|@BINARY@|$binary|g" \
  "$repo_dir/systemd/specter-sense.service.in" > "$output_dir/specter-sense.service"
sed -e "s|@REPO_DIR@|$repo_dir|g" -e "s|@TRAY@|$tray|g" \
  "$repo_dir/systemd/specter-sense-tray.service.in" > "$output_dir/specter-sense-tray.service"
echo "Generated units in $output_dir"

if [[ "$install_units" == true ]]; then
  unit_dir="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
  mkdir -p "$unit_dir"
  install -m 0644 "$output_dir/specter-sense.service" "$unit_dir/specter-sense.service"
  install -m 0644 "$output_dir/specter-sense-tray.service" "$unit_dir/specter-sense-tray.service"
  systemctl --user daemon-reload
  if [[ "$start_units" == true ]]; then
    systemctl --user enable --now specter-sense.service
    systemctl --user reenable specter-sense-tray.service
    systemctl --user start specter-sense-tray.service
  else
    systemctl --user enable specter-sense.service
    systemctl --user reenable specter-sense-tray.service
  fi
  echo "Installed user units. Toggle the sensor with the tray icon or systemctl --user start/stop specter-sense."
fi
