#!/usr/bin/env python3
# Copyright (c) 2025, DYROS.
#
# Convert a Footstep local command CSV (config/footcommands.csv) into a
# MindYourStep replay CSV (config/footstepcommands_mys.csv) so both controllers
# execute the same stance-frame footholds for experiment comparison.
#
# Footstep and MindYourStep share the same local-command semantics:
#   foot,step_x,step_y,step_z,step_yaw[,ssp_t,dsp_t,height]
#     foot   : L / R  -> which foot swings
#     step_* : displacement of the swing foot in the stance-foot yaw frame
#              (step_y is a positive magnitude; the controller applies +/-)
#
# Timing note: MindYourStep advances one foothold every half gait cycle
# (0.5 / gait_frequency). With gait_frequency = 1.0 Hz that is 1.0 s/step,
# matching a typical Footstep ssp_t + 2*dsp_t = 1.0 s plan. The converter
# keeps ssp_t/dsp_t/height for eval logging only; they do not change the MYS
# gait clock (set gait_frequency in deploy.yaml if you need a different rate).
#
# Usage:
#   python3 cmd/convert2mys.py
#   python3 cmd/convert2mys.py --input config/footcommands.csv \
#                              --output config/footstepcommands_mys.csv

import argparse
import csv
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJ_DIR = os.path.dirname(_THIS_DIR)
DEFAULT_INPUT = os.path.join(_PROJ_DIR, "config", "footcommands.csv")
DEFAULT_OUTPUT = os.path.join(_PROJ_DIR, "config", "footcommands_mys.csv")

OUT_COLS = ["foot", "step_x", "step_y", "step_z", "step_yaw", "ssp_t", "dsp_t", "height"]


def read_rows(path):
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = [h.strip().lower() for h in next(reader)]
        idx = {name: i for i, name in enumerate(header)}
        for c in ("foot", "step_x", "step_y", "step_yaw"):
            if c not in idx:
                raise ValueError(f"missing required column '{c}' in {path}")

        def opt(cells, name, default):
            return float(cells[idx[name]]) if name in idx else default

        rows = []
        for line_no, cells in enumerate(reader, start=2):
            cells = [c.strip() for c in cells]
            if not cells or all(c == "" for c in cells):
                continue
            foot = cells[idx["foot"]].strip().lower()
            if foot in ("r", "right"):
                label = "R"
            elif foot in ("l", "left"):
                label = "L"
            else:
                raise ValueError(f"invalid foot '{cells[idx['foot']]}' at line {line_no}")

            step_y = abs(float(cells[idx["step_y"]]))
            rows.append({
                "foot": label,
                "step_x": float(cells[idx["step_x"]]),
                "step_y": step_y,
                "step_z": opt(cells, "step_z", 0.0),
                "step_yaw": float(cells[idx["step_yaw"]]),
                "ssp_t": opt(cells, "ssp_t", 0.70),
                "dsp_t": opt(cells, "dsp_t", 0.15),
                "height": opt(cells, "height", 0.08),
            })
    if not rows:
        raise ValueError(f"no data rows in {path}")
    return rows


def check_alternation(rows):
    for i in range(1, len(rows)):
        if rows[i]["foot"] == rows[i - 1]["foot"]:
            print(f"[warn] feet do not strictly alternate at rows {i}/{i+1} "
                  f"(both {rows[i]['foot']})", file=sys.stderr)
            break


def write_rows(path, rows):
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(OUT_COLS)
        for r in rows:
            w.writerow([
                r["foot"],
                f"{r['step_x']:.6f}",
                f"{r['step_y']:.6f}",
                f"{r['step_z']:.6f}",
                f"{r['step_yaw']:.6f}",
                f"{r['ssp_t']:.6f}",
                f"{r['dsp_t']:.6f}",
                f"{r['height']:.6f}",
            ])


def main():
    p = argparse.ArgumentParser(
        description="Convert footcommands.csv -> footstepcommands_mys.csv for MYS CSV deploy.")
    p.add_argument("--input", "-i", default=DEFAULT_INPUT,
                   help="Footstep local command CSV")
    p.add_argument("--output", "-o", default=DEFAULT_OUTPUT,
                   help="MindYourStep replay CSV")
    args = p.parse_args()

    rows = read_rows(args.input)
    check_alternation(rows)
    write_rows(args.output, rows)

    # Nominal step duration from the first row (informational).
    t0 = rows[0]["ssp_t"] + 2.0 * rows[0]["dsp_t"]
    # MindYourStep half-step duration = 0.5 / gait_frequency; match footstep period.
    freq = 0.5 / t0 if t0 > 1e-6 else 0.5
    print(f"[convert2mys] {len(rows)} steps: {args.input}")
    print(f"              -> {args.output}")
    print(f"              first foot={rows[0]['foot']}, "
          f"nominal footstep period≈{t0:.3f}s "
          f"(set gait.gait_frequency: {freq:.3f} in mindyourstep deploy.yaml)")


if __name__ == "__main__":
    main()
