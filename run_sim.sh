#!/usr/bin/env bash
# Launch the MuJoCo simulator (unitree_mujoco) and the G1 footstep controller
# (g1_ctrl) together. The simulator is started first; once it is up the
# controller connects over the chosen network (default: loopback "lo").
#
# The controller runs in the FOREGROUND so keyboard FSM keys (f/h/v/g/m/p)
# reach g1_ctrl. Type in this terminal, not the MuJoCo window.
#
# Ctrl+C (or either process exiting) tears down both.
#
# Usage:
#   ./run_sim.sh            # network = lo
#   ./run_sim.sh enp3s0     # use a different network interface

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- paths (relative to this script / g1_controller) ---------------------
MUJOCO_DIR="$SCRIPT_DIR/../unitree_mujoco/simulate/build"
MUJOCO_BIN="./unitree_mujoco"
CTRL_DIR="$SCRIPT_DIR/build"
CTRL_BIN="./g1_ctrl"
ONNX_LIB="$SCRIPT_DIR/thirdparty/onnxruntime-linux-x64-1.22.0/lib"

NETWORK="${1:-lo}"
SIM_WAIT="${SIM_WAIT:-2}"   # seconds to let the simulator come up before the controller

# Overrides unitree_mujoco/simulate/config.yaml (no need to edit that file).
ROBOT="g1"
SCENE="scene_29dof_footstep.xml"

# --- sanity checks ---------------------------------------------------------
[[ -x "$MUJOCO_DIR/$MUJOCO_BIN" ]] || { echo "[run] missing simulator: $MUJOCO_DIR/$MUJOCO_BIN" >&2; exit 1; }
[[ -x "$CTRL_DIR/$CTRL_BIN" ]]     || { echo "[run] missing controller: $CTRL_DIR/$CTRL_BIN" >&2; exit 1; }
[[ -d "$ONNX_LIB" ]]               || { echo "[run] missing onnxruntime lib dir: $ONNX_LIB" >&2; exit 1; }

pids=()
cleanup() {
  trap - INT TERM EXIT
  echo ""
  echo "[run] shutting down..."
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  stty sane 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# --- 1) simulator (background; stdin detached so it cannot steal keys) -----
echo "[run] starting simulator: $MUJOCO_DIR/$MUJOCO_BIN -r $ROBOT -s $SCENE"
( cd "$MUJOCO_DIR" && exec "$MUJOCO_BIN" -r "$ROBOT" -s "$SCENE" ) </dev/null &
pids+=($!)

sleep "$SIM_WAIT"
if ! kill -0 "${pids[0]}" 2>/dev/null; then
  echo "[run] simulator exited early" >&2
  exit 1
fi

# --- 2) controller (foreground so this terminal's keys reach g1_ctrl) ------
echo "[run] starting controller: $CTRL_BIN --network $NETWORK"
echo "[run] sim pid ${pids[0]}. Type keys in THIS terminal (f/h/v/g/m/p). Ctrl+C stops both."

cd "$CTRL_DIR"
export LD_LIBRARY_PATH="$ONNX_LIB:${LD_LIBRARY_PATH:-}"
"$CTRL_BIN" --network "$NETWORK"
