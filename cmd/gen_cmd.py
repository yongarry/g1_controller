#!/usr/bin/env python3
# Copyright (c) 2025, DYROS.
#
# Generate a local per-step foot command CSV (config/footcommands.csv) by
# sampling step_x / step_y / step_z / step_yaw uniformly from the trained command
# ranges. Feet alternate (R, L, R, ...) and the final step is a "stop" step
# (step_x=0, step_z=0, step_yaw=0) so the robot squares up at the end.
#
# --realistic: after a large step_x / step_y (upper half of that axis range) or
# a large |step_z| (outer half toward either extreme), the next sample on that
# axis is drawn from the non-extreme band. Consecutive extremes would otherwise
# stack landing error into the following global-plan command and push it past
# the trained limits.
#
# The default ranges match `footstep.ranges` in the footstep policy deploy.yaml
# (the controller clamps to these anyway). ssp_t / dsp_t / height are written as
# fixed values by default (override with flags, or pass a range to randomize).
#
# Usage:
#   python3 cmd/gen_cmd.py 10                     # 10 steps -> config/footcommands.csv
#   python3 cmd/gen_cmd.py 20 --start L --seed 0
#   python3 cmd/gen_cmd.py 12 --x -0.1 0.2 --yaw -0.1 0.1 --y 0.22 0.26 --z -0.1 0.15
#   python3 cmd/gen_cmd.py 8 --no-stop -o /tmp/fc.csv
#   python3 cmd/gen_cmd.py 10 --real              # real robot: skip MuJoCo terrain XML
#   python3 cmd/gen_cmd.py 10 --realistic         # no two consecutive large xyz steps

import argparse
import csv
import os
import random

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJ_DIR = os.path.dirname(_THIS_DIR)
DEFAULT_OUTPUT = os.path.join(_PROJ_DIR, "config", "footcommands.csv")

# Trained command ranges (must match footstep deploy.yaml `footstep.ranges`).
# 01. 2d random footstep sampling
# RANGE_X = (-0.3, 0.4)     # forward step length [m]
# RANGE_Y = (0.2, 0.4)      # lateral step width (positive magnitude) [m]
# RANGE_Z = (0.0, 0.0)   # per-step height change [m]
# RANGE_YAW = (-0.5, 0.5)   # per-step turn [rad]
# NOMINAL_Y = 0.237         # lateral width used for the final stop step [m]

# 02. 3d random footstep sampling 
RANGE_X = (0.2, 0.4)     # forward step length [m]
RANGE_Y = (0.2, 0.4)      # lateral step width (positive magnitude) [m]
RANGE_Z = (-0.15, 0.2)   # per-step height change [m]
RANGE_YAW = (-0.4, 0.4)   # per-step turn [rad]
NOMINAL_Y = 0.237         # lateral width used for the final stop step [m]

# --realistic: x/y are "large" in the upper (1 - REALISTIC_SMALL_FRAC) of the
# range; z is "large" outside the inner REALISTIC_SMALL_FRAC band around 0
# (signed). After a large sample the next draw on that axis stays in the
# non-extreme band so consecutive extremes cannot stack landing error past
# the trained limits.
REALISTIC_SMALL_FRAC = 0.5

# 03. 3d random visual footstep sampling 
# RANGE_X = (0.25, 0.3)     # forward step length [m]
# RANGE_Y = (0.237, 0.3)      # lateral step width (positive magnitude) [m]
# RANGE_Z = (-0.1, 0.1)   # per-step height change [m]
# RANGE_YAW = (-0.5, 0.5)   # per-step turn [rad]
# NOMINAL_Y = 0.237         # lateral width used for the final stop step [m]

# 05. real robot stair experiment
# Fixed per-step z scene (uncomment to use). Length must equal `step` (incl. stop).
# x/y/yaw still come from --x/--y/--yaw (or RANGE_*). Example 10-step climb:
# SCENE_Z = [0.128, 0.12, 0.12, 0.12, 0.12, 0.00, 0.00, -0.15, -0.15, -0.15, -0.158,  0.0, 0.0]
# SCENE_X = [0.400, 0.25, 0.25, 0.25, 0.25, 0.25, 0.0,  0.26,  0.26,  0.25,  0.25,  0.25, 0.0]
# SCENE_Z = [0.128, 0.12, 0.12, 0.12, 0.12, 0.0, 0.0, 0.0, -0.15, -0.15, -0.15, -0.158,  0.0, 0.0]
# SCENE_X = [0.4, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.0]
SCENE_Z = None
SCENE_X = None

HEADER = ["foot", "step_x", "step_y", "step_z", "step_yaw", "ssp_t", "dsp_t", "height"]


def sample(lo, hi):
    return random.uniform(lo, hi)


def sample_capped(lo, hi, cap_frac=None):
    """Uniform sample on [lo, hi], or [lo, lo + cap_frac*(hi-lo)] when capped."""
    if cap_frac is not None and hi > lo:
        hi = lo + cap_frac * (hi - lo)
    return sample(lo, hi)


def axis_is_large(val, lo, hi, small_frac=REALISTIC_SMALL_FRAC):
    if hi <= lo:
        return False
    return val > lo + small_frac * (hi - lo)


def z_inner_range(lo, hi, small_frac=REALISTIC_SMALL_FRAC):
    """Non-extreme z band. If the range straddles 0, shrink toward 0 on each
    side; otherwise the lower small_frac (same as x/y)."""
    if hi <= lo:
        return (lo, hi)
    if lo < 0.0 < hi:
        return (small_frac * lo, small_frac * hi)
    return (lo, lo + small_frac * (hi - lo))


def z_is_large(val, lo, hi, small_frac=REALISTIC_SMALL_FRAC):
    inner_lo, inner_hi = z_inner_range(lo, hi, small_frac)
    return val < inner_lo or val > inner_hi


def build_rows(n, start, rx, ry, rz, ryaw, ssp, dsp, height, stop_last,
               scene_z=None, scene_x=None, realistic=False):
    if scene_z is not None and len(scene_z) != n:
        raise ValueError(f"SCENE_Z length {len(scene_z)} != step {n}")
    rows = []
    recover_x = False
    recover_y = False
    recover_z = False
    for i in range(n):
        foot = start if i % 2 == 0 else ("L" if start == "R" else "R")
        is_last_stop = stop_last and (i == n - 1)
        if is_last_stop:
            step_x, step_y, step_z, step_yaw = 0.0, NOMINAL_Y, 0.0, 0.0
        elif scene_z is not None:
            step_x = float(scene_x[i])
            step_y = 0.237
            step_z = float(scene_z[i])
            step_yaw = 0.0
        elif i == 0 and scene_z is None:
            step_x = 0.2
            step_y = 0.237
            step_z = 0.0
            step_yaw = 0.0
        else:
            cap_x = REALISTIC_SMALL_FRAC if (realistic and recover_x) else None
            cap_y = REALISTIC_SMALL_FRAC if (realistic and recover_y) else None
            step_x = round(sample_capped(*rx, cap_x), 3)
            step_y = round(sample_capped(*ry, cap_y), 3)
            if realistic and recover_z:
                step_z = round(sample(*z_inner_range(*rz)), 3)
            else:
                step_z = round(sample(*rz), 3)
            step_yaw = sample(*ryaw)
            recover_x = realistic and axis_is_large(step_x, *rx)
            recover_y = realistic and axis_is_large(step_y, *ry)
            recover_z = realistic and z_is_large(step_z, *rz)
        rows.append([
            foot,
            f"{step_x:.3f}", f"{step_y:.3f}", f"{step_z:.3f}", f"{step_yaw:.3f}",
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
    p.add_argument("--z", nargs=2, type=float, default=list(RANGE_Z),
                   metavar=("MIN", "MAX"), help="step_z range [m]")
    p.add_argument("--yaw", nargs=2, type=float, default=list(RANGE_YAW),
                   metavar=("MIN", "MAX"), help="step_yaw range [rad]")
    p.add_argument("--ssp", type=float, default=0.8, help="single support time [s]")
    p.add_argument("--dsp", type=float, default=0.15, help="double support time [s]")
    p.add_argument("--height", type=float, default=0.07, help="swing apex height [m]")
    p.add_argument("--no-stop", action="store_true",
                   help="do not force the last step to be a stop step")
    p.add_argument("--real", action="store_true",
                   help="real robot: skip MuJoCo rocky-mountain scene generation")
    p.add_argument("--realistic", action="store_true",
                   help="after a large x/y/|z| step, the next sample on that axis "
                        "is drawn from the non-extreme band (avoids stacking "
                        "landing error past the trained command limits)")
    args = p.parse_args()

    if args.step <= 0:
        raise ValueError("step must be a positive integer")
    if args.seed is None:
        args.seed = int.from_bytes(os.urandom(4), "little")
    random.seed(args.seed)

    rows = build_rows(args.step, args.start, args.x, args.y, args.z, args.yaw,
                      args.ssp, args.dsp, args.height, stop_last=not args.no_stop,
                      scene_z=SCENE_Z, scene_x=SCENE_X, realistic=args.realistic)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        w.writerows(rows)

    print(f"Wrote {len(rows)} footsteps to {args.output} "
          f"(seed={args.seed}, start={args.start}, x={tuple(args.x)}, y={tuple(args.y)}, "
          f"z={tuple(args.z)}, yaw={tuple(args.yaw)}, realistic={args.realistic})")
    
    # execute convert_footcommand_2_global.py to generate footcommands_global.csv
    import subprocess
    subprocess.run(["python3", os.path.join(_THIS_DIR, "convert_footcommand_2_global.py"), "--input", args.output, "--output", os.path.join(_PROJ_DIR, "config", "footcommands_global.csv")])  

    # execute gen_footstep_scene.py to generate footstep cubes in the MuJoCo scene XML
    subprocess.run(["python3", os.path.join(_THIS_DIR, "convert2mys.py")])


    subprocess.run(["python3", os.path.join(_THIS_DIR, "gen_footstep_scene.py")])
    # subprocess.run(["python3", os.path.join(_THIS_DIR, "gen_rocky_mountain.py")])
    # subprocess.run(["python3", os.path.join(_THIS_DIR, "gen_aruco_footstep_scene.py")])
    # subprocess.run(["python3", os.path.join(_THIS_DIR, "gen_aruco_footstep_scene.py"),"--stair", "--size", "0.125", "0.5"])



# good seed mountain
#     3609553359
