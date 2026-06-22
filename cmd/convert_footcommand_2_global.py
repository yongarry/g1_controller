#!/usr/bin/env python3
# Copyright (c) 2025, DYROS.
#
# Convert per-step LOCAL foot commands (config/footcommands.csv) into absolute
# WORLD-frame swing-foot targets (config/footcommands_global.csv) consumed by
# State_Footstep when `command_source: csv_global`.
#
# World frame convention (must match the controller):
#   * stance[0] is the INITIAL STANCE FOOT world pose (--init-lfoot/--init-rfoot),
#     which must equal deploy.yaml global_init_lfoot/rfoot so the controller's
#     accumulated stance frame and these targets share the same world frame.
#   * Each output row is the SWING foot's absolute landing pose for that step.
#   * On-device, the planner accumulates the achieved stance pose from measured
#     foot landings and recomputes the local command from the current stance foot
#     to the indexed target, so this file only needs the nominal world targets.
#
# Local CSV format  (one row per step):
#   foot,step_x,step_y,step_yaw,ssp_t,dsp_t,height[,step_z][,com_z]
#     foot   : L / R  -> which foot swings (sets swing side / y sign)
#     step_* : per-step LOCAL displacement, expressed in the stance-foot frame
#              (step_y is a positive magnitude; the sign is applied from `foot`)
#
# Global CSV format (one row per step):
#   foot,pos_x,pos_y,pos_z,yaw,ssp_t,dsp_t,height[,com_z]
#     pos_*  : absolute SWING-foot landing pose in the world frame
#
# Usage:
#   python3 cmd/convert_footcommand_2_global.py
#   python3 cmd/convert_footcommand_2_global.py --input config/footcommands.csv \
#                                               --output config/footcommands_global.csv

import argparse
import csv
import math
import os

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJ_DIR = os.path.dirname(_THIS_DIR)
DEFAULT_INPUT = os.path.join(_PROJ_DIR, "config", "footcommands.csv")
DEFAULT_OUTPUT = os.path.join(_PROJ_DIR, "config", "footcommands_global.csv")


def wrap_to_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def read_local_csv(path):
    """Read the per-row local foot command CSV into a list of dicts."""
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = [h.strip().lower() for h in next(reader)]
        idx = {name: i for i, name in enumerate(header)}

        required = ["foot", "step_x", "step_y", "step_yaw", "ssp_t", "dsp_t", "height"]
        for c in required:
            if c not in idx:
                raise ValueError(f"missing required column '{c}' in {path}")

        def opt(cells, name, default):
            return float(cells[idx[name]]) if name in idx else default

        rows = []
        for cells in reader:
            cells = [c.strip() for c in cells]
            if not cells or all(c == "" for c in cells):
                continue
            foot = cells[idx["foot"]].strip().lower()
            if foot in ("r", "right"):
                phase = 0  # right swing -> foot lands on -y
            elif foot in ("l", "left"):
                phase = 1  # left swing  -> foot lands on +y
            else:
                raise ValueError(f"invalid foot label '{cells[idx['foot']]}' in {path}")
            rows.append({
                "foot": "R" if phase == 0 else "L",
                "phase": phase,
                "step_x": float(cells[idx["step_x"]]),
                "step_y": float(cells[idx["step_y"]]),
                "step_yaw": float(cells[idx["step_yaw"]]),
                "step_z": opt(cells, "step_z", 0.0),
                "ssp_t": float(cells[idx["ssp_t"]]),
                "dsp_t": float(cells[idx["dsp_t"]]),
                "height": float(cells[idx["height"]]),
                "com_z": opt(cells, "com_z", 0.0),
            })
    if not rows:
        raise ValueError(f"no data rows in {path}")
    return rows


def to_global(rows, init_stance):
    """Accumulate local steps into absolute world swing-foot targets.

    stance = pose of the foot the swing foot pushes off from. stance[0] is the
    initial stance foot world pose (`init_stance`); this MUST match the anchor the
    controller uses (deploy.yaml global_init_lfoot/rfoot). For each step:
        step_y_signed = +|step_y| for a left swing, -|step_y| for a right swing
        swing = stance + R(stance.yaw) * (step_x, step_y_signed, step_z)
        swing.yaw = wrap(stance.yaw + step_yaw)
        next stance = swing
    """
    stance = dict(init_stance)
    out = []
    for r in rows:
        sign = -1.0 if r["phase"] == 0 else 1.0
        sy = sign * abs(r["step_y"])
        c, s = math.cos(stance["yaw"]), math.sin(stance["yaw"])
        swing = {
            "x": stance["x"] + c * r["step_x"] - s * sy,
            "y": stance["y"] + s * r["step_x"] + c * sy,
            "z": stance["z"] + r["step_z"],
            "yaw": wrap_to_pi(stance["yaw"] + r["step_yaw"]),
        }
        out.append({
            "foot": r["foot"],
            "pos_x": swing["x"],
            "pos_y": swing["y"],
            "pos_z": swing["z"],
            "yaw": swing["yaw"],
            "ssp_t": r["ssp_t"],
            "dsp_t": r["dsp_t"],
            "height": r["height"],
            "com_z": r["com_z"],
        })
        stance = swing
    return out


def write_global_csv(path, targets, with_com_z):
    cols = ["foot", "pos_x", "pos_y", "pos_z", "yaw", "ssp_t", "dsp_t", "height"]
    if with_com_z:
        cols.append("com_z")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for t in targets:
            row = [
                t["foot"],
                f"{t['pos_x']:.4f}", f"{t['pos_y']:.4f}", f"{t['pos_z']:.4f}",
                f"{t['yaw']:.4f}", f"{t['ssp_t']:.2f}", f"{t['dsp_t']:.2f}",
                f"{t['height']:.3f}",
            ]
            if with_com_z:
                row.append(f"{t['com_z']:.3f}")
            w.writerow(row)
    print(f"Saved: {path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert local foot commands to absolute world-frame targets.")
    parser.add_argument("--input", default=DEFAULT_INPUT, help="local footcommands.csv path")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="output footcommands_global.csv path")
    # Spawn foot world poses [x, y, z, yaw]; must match deploy.yaml global_init_*foot.
    parser.add_argument("--init-lfoot", nargs=4, type=float, default=[-0.02179, 0.118506, 0.0, 0.0],
                        metavar=("X", "Y", "Z", "YAW"), help="left foot world pose at spawn")
    parser.add_argument("--init-rfoot", nargs=4, type=float, default=[-0.02179, -0.118506, 0.0, 0.0],
                        metavar=("X", "Y", "Z", "YAW"), help="right foot world pose at spawn")
    args = parser.parse_args()

    rows = read_local_csv(args.input)
    # initial stance foot = the foot NOT swinging first (phase 0 = right swings).
    init_foot = args.init_lfoot if rows[0]["phase"] == 0 else args.init_rfoot
    init_stance = {"x": init_foot[0], "y": init_foot[1], "z": init_foot[2], "yaw": init_foot[3]}
    targets = to_global(rows, init_stance)
    with_com_z = any(r["com_z"] != 0.0 for r in rows)
    write_global_csv(args.output, targets, with_com_z)

    print(f"World-frame swing-foot targets (stance[0] = {init_stance}):")
    for i, t in enumerate(targets):
        print(f"  step {i:3d} [{t['foot']}]: "
              f"x={t['pos_x']:7.4f}  y={t['pos_y']:7.4f}  z={t['pos_z']:6.3f}  "
              f"yaw={math.degrees(t['yaw']):7.2f}deg")
