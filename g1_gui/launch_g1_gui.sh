#!/usr/bin/env bash
# Run g1_gui_server: HTTP + WebSocket for live G1 lowstate (same DDS as g1_ctl / MuJoCo sim).
#
# Usage:
#   ./launch_g1_gui.sh [network_interface] [http_port]
#
# Examples:
#   ./launch_g1_gui.sh lo
#   ./launch_g1_gui.sh lo 4710
#
# Then open http://127.0.0.1:<port>/ in a browser.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="${ROOT}/build/g1_gui_server"

if [[ ! -x "$BIN" ]]; then
  echo "error: not found or not executable: $BIN" >&2
  echo "  mkdir -p \"$ROOT/build\" && cd \"$ROOT/build\" && cmake .. && cmake --build ." >&2
  exit 1
fi

IFACE="${1:-lo}"
PORT="${2:-4710}"

exec "$BIN" "$IFACE" "$PORT"
