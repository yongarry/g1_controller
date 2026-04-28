#!/usr/bin/env bash
# Run unitree_mujoco and g1_ctl together (similar idea to roslaunch: one command, both die together).
#
# Usage:
#   ./scripts/launch_g1_mujoco.sh [network_interface] [-- extra args for unitree_mujoco only...]
#
# Examples:
#   ./scripts/launch_g1_mujoco.sh lo
#   ./scripts/launch_g1_mujoco.sh eth0 -- -n eth0 -i 0
#
# network_interface is passed to g1_ctl as argv[1]. Arguments after "--" go only to unitree_mujoco.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MUJOCO_BIN="${ROOT}/unitree_mujoco/simulate/build/unitree_mujoco"

# Prefer the unified superbuild outputs (g1_controller/CMakeLists.txt at repo root),
# but keep backward-compatible fallbacks for older per-project builds.
G1_BIN="${ROOT}/g1_controller/build/g1_controller/g1_ctl"
GUI_BIN="${ROOT}/g1_controller/build/g1_gui/g1_gui_server"
if [[ ! -x "$G1_BIN" ]]; then
  G1_BIN="${ROOT}/g1_controller/build/g1_ctl"
fi
if [[ ! -x "$GUI_BIN" ]]; then
  GUI_BIN="${ROOT}/g1_gui/build/g1_gui_server"
fi

if [[ ! -x "$MUJOCO_BIN" ]]; then
  echo "error: not found or not executable: $MUJOCO_BIN" >&2
  echo "  build: cmake --build unitree_mujoco/simulate/build" >&2
  exit 1
fi
if [[ ! -x "$G1_BIN" ]]; then
  echo "error: not found or not executable: $G1_BIN" >&2
  echo "  build: mkdir -p g1_controller/build && cd g1_controller/build && cmake .. && cmake --build ." >&2
  exit 1
fi
if [[ ! -x "$GUI_BIN" ]]; then
  echo "error: not found or not executable: $GUI_BIN" >&2
  echo "  build: mkdir -p g1_controller/build && cd g1_controller/build && cmake .. && cmake --build ." >&2
  exit 1
fi

IFACE="lo"
MUJOCO_EXTRA=()
if [[ "${1:-}" == "--" ]]; then
  shift
  MUJOCO_EXTRA=("$@")
else
  IFACE="${1:-lo}"
  shift || true
  if [[ "${1:-}" == "--" ]]; then
    shift
    MUJOCO_EXTRA=("$@")
  fi
fi

MJ_PID=""
G1_PID=""
GUI_PID=""
cleanup() {
  local code="${1:-0}"
  trap - EXIT INT TERM
  [[ -n "${GUI_PID}" ]] && kill "$GUI_PID" 2>/dev/null || true
  [[ -n "${G1_PID}" ]] && kill "$G1_PID" 2>/dev/null || true
  [[ -n "${MJ_PID}" ]] && kill "$MJ_PID" 2>/dev/null || true
  wait 2>/dev/null || true
  exit "$code"
}
trap 'cleanup 130' INT
trap 'cleanup 143' TERM

echo "[launch] unitree_mujoco: $MUJOCO_BIN ${MUJOCO_EXTRA[*]:-}"
"$MUJOCO_BIN" "${MUJOCO_EXTRA[@]}" &
MJ_PID=$!

echo "[launch] g1_ctl: $G1_BIN $IFACE"
"$G1_BIN" "$IFACE" &
G1_PID=$!

echo "[launch] g1_gui_server: $GUI_BIN $IFACE 4710"
"$GUI_BIN" "$IFACE" 4710 &
GUI_PID=$!

# When either process exits, stop the other (like launch shutting down all nodes).
set +e
wait -n
ec=$?
set -e
cleanup "$ec"
