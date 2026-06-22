#!/usr/bin/env python3
# Copyright (c) 2025, DYROS.
#
# Generate a local per-step foot command CSV (config/footcommands.csv) by
# sampling step_x / step_y / step_yaw uniformly from the trained command ranges.
# Feet alternate (R, L, R, ...) and the final step is a "stop" step (step_x=0,
# step_yaw=0) so the robot squares up at the end.
#
# The default ranges match `footstep.ranges` in the footstep policy deploy.yaml
# (the controller clamps to these anyway). ssp_t / dsp_t / height are written as
# fixed values by default (override with flags, or pass a range to randomize).
#
# Usage:
#   python3 cmd/gen_cmd.py 10                     # 10 steps -> config/footcommands.csv
#   python3 cmd/gen_cmd.py 20 --start L --seed 0
#   python3 cmd/gen_cmd.py 12 --x -0.1 0.2 --yaw -0.1 0.1 --y 0.22 0.26
#   python3 cmd/gen_cmd.py 8 --no-stop -o /tmp/fc.csv

import argparse
import csv
import os
import random

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJ_DIR = os.path.dirname(_THIS_DIR)
DEFAULT_OUTPUT = os.path.join(_PROJ_DIR, "config", "footcommands.csv")

# Trained command ranges (must match footstep deploy.yaml `footstep.ranges`).
RANGE_X = (0.1, 0.2)     # forward step length [m]
RANGE_Y = (0.237, 0.237)      # lateral step width (positive magnitude) [m]
RANGE_YAW = (0., 0.2)   # per-step turn [rad]
NOMINAL_Y = 0.237         # lateral width used for the final stop step [m]

HEADER = ["foot", "step_x", "step_y", "step_yaw", "ssp_t", "dsp_t", "height"]


def sample(lo, hi):
    return random.uniform(lo, hi)


def build_rows(n, start, rx, ry, ryaw, ssp, dsp, height, stop_last):
    rows = []
    for i in range(n):
        foot = start if i % 2 == 0 else ("L" if start == "R" else "R")
        is_last_stop = stop_last and (i == n - 1)
        if is_last_stop:
            step_x, step_y, step_yaw = 0.0, NOMINAL_Y, 0.0
        else:
            step_x = sample(*rx)
            step_y = sample(*ry)
            step_yaw = sample(*ryaw)
        rows.append([
            foot,
            f"{step_x:.3f}", f"{step_y:.3f}", f"{step_yaw:.3f}",
            f"{ssp:.2f}", f"{dsp:.2f}", f"{height:.3f}",
        ])
    return rows


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Generate footcommands.csv by sampling step ranges.")
    p.add_argument("step", type=int, help="number of footsteps to generate")
    p.add_argument("-o", "--output", default=DEFAULT_OUTPUT, help="output CSV path")
    p.add_argument("--start", choices=["R", "L"], default="R", help="first swing foot")
    p.add_argument("--seed", type=int, default=None, help="RNG seed for reproducibility")
    p.add_argument("--x", nargs=2, type=float, default=list(RANGE_X),
                   metavar=("MIN", "MAX"), help="step_x range [m]")
    p.add_argument("--y", nargs=2, type=float, default=list(RANGE_Y),
                   metavar=("MIN", "MAX"), help="step_y range [m]")
    p.add_argument("--yaw", nargs=2, type=float, default=list(RANGE_YAW),
                   metavar=("MIN", "MAX"), help="step_yaw range [rad]")
    p.add_argument("--ssp", type=float, default=0.7, help="single support time [s]")
    p.add_argument("--dsp", type=float, default=0.1, help="double support time [s]")
    p.add_argument("--height", type=float, default=0.075, help="swing apex height [m]")
    p.add_argument("--no-stop", action="store_true",
                   help="do not force the last step to be a stop step")
    args = p.parse_args()

    if args.step <= 0:
        raise ValueError("step must be a positive integer")
    if args.seed is not None:
        random.seed(args.seed)

    rows = build_rows(args.step, args.start, args.x, args.y, args.yaw,
                      args.ssp, args.dsp, args.height, stop_last=not args.no_stop)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        w.writerows(rows)

    print(f"Wrote {len(rows)} footsteps to {args.output} "
          f"(start={args.start}, x={tuple(args.x)}, y={tuple(args.y)}, yaw={tuple(args.yaw)})")
    
    # execute convert_footcommand_2_global.py to generate footcommands_global.csv
    import subprocess
    subprocess.run(["python3", os.path.join(_THIS_DIR, "convert_footcommand_2_global.py"), "--input", args.output, "--output", os.path.join(_PROJ_DIR, "config", "footcommands_global.csv")])  

    # execute gen_footstep_scene.py to generate footstep cubes in the MuJoCo scene XML
    subprocess.run(["python3", os.path.join(_THIS_DIR, "gen_footstep_scene.py")])
