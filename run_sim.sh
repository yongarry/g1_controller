#!/usr/bin/env bash
# Launch the MuJoCo simulator (unitree_mujoco) and the G1 footstep controller
# (g1_ctrl) together. The simulator is started first; once it is up the
# controller connects over the chosen network (default: loopback "lo").
#
# Ctrl+C (or either process exiting) tears down both.
#
# Usage:
#   ./run_sim.sh            # network = lo
#   ./run_sim.sh enp3s0     # use a different network interface

set -uo pipefail

# --- paths (edit here if your layout differs) ------------------------------
MUJOCO_DIR="/home/yong/unitree_ws/unitree_mujoco/simulate/build"
MUJOCO_BIN="./unitree_mujoco"
CTRL_DIR="/home/yong/unitree_ws/g1_controller/build"
CTRL_BIN="./g1_ctrl"
ONNX_LIB="/home/yong/unitree_ws/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib"

NETWORK="${1:-lo}"
SIM_WAIT="${SIM_WAIT:-2}"   # seconds to let the simulator come up before the controller

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
}
trap cleanup INT TERM EXIT

# --- 1) simulator ----------------------------------------------------------
echo "[run] starting simulator: $MUJOCO_DIR/$MUJOCO_BIN"
( cd "$MUJOCO_DIR" && exec "$MUJOCO_BIN" ) &
pids+=($!)

sleep "$SIM_WAIT"

# --- 2) controller ---------------------------------------------------------
echo "[run] starting controller: $CTRL_BIN --network $NETWORK"
( cd "$CTRL_DIR" && LD_LIBRARY_PATH="$ONNX_LIB:${LD_LIBRARY_PATH:-}" exec "$CTRL_BIN" --network "$NETWORK" ) &
pids+=($!)

echo "[run] both running (sim pid ${pids[0]}, ctrl pid ${pids[1]}). Ctrl+C to stop."

# Exit (and trigger cleanup) as soon as either process terminates.
wait -n
