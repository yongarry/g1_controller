#!/usr/bin/env python3
# Copyright (c) 2025, DYROS.
#
# Summarize footstep tracking accuracy from a per-step eval CSV written by
# State_Footstep (log/footstep_eval_<time>.csv) or State_MindYourStep
# (log/footstep_eval_mys_<time>.csv). Errors are commanded minus measured
# swing-foot landing, in the stance-foot frame. Both share the same schema.
#
# Footstep:
#   ./run_sim.sh   # enter Footstep -> writes log/footstep_eval_YYMMDD_HHMMSS.csv
#   python3 cmd/eval_footstep.py
#
# MindYourStep:
#   ./run_sim.sh   # enter MindYourStep -> log/footstep_eval_mys_YYMMDD_HHMMSS.csv
#   python3 cmd/eval_footstep.py --mys
#
# With no path args, the newest matching timestamped file is used. Pass paths
# explicitly to pool several runs.

import argparse
import csv
import glob
import math
import os

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJ_DIR = os.path.dirname(_THIS_DIR)
_LOG_DIR = os.path.join(_PROJ_DIR, "log")
DEFAULT_GLOB = os.path.join(_LOG_DIR, "footstep_eval_*.csv")
DEFAULT_GLOB_MYS = os.path.join(_LOG_DIR, "footstep_eval_mys_*.csv")
DEFAULT_PLOT = os.path.join(_LOG_DIR, "footstep_eval.png")
DEFAULT_PLOT_MYS = os.path.join(_LOG_DIR, "footstep_eval_mys.png")

AXES = [("err_x", "x [m]"), ("err_y", "y [m]"), ("err_z", "z [m]"), ("err_yaw", "yaw [rad]")]


def latest_eval_csv(mys: bool) -> str:
    """Newest timestamped eval CSV; ignore the non-mys pattern when --mys."""
    if mys:
        cands = sorted(glob.glob(DEFAULT_GLOB_MYS), key=os.path.getmtime)
    else:
        # Footstep files are footstep_eval_<time>.csv, not footstep_eval_mys_*.
        cands = [
            p for p in glob.glob(DEFAULT_GLOB)
            if not os.path.basename(p).startswith("footstep_eval_mys")
        ]
        cands.sort(key=os.path.getmtime)
    if not cands:
        pattern = DEFAULT_GLOB_MYS if mys else "log/footstep_eval_*.csv (excluding _mys_)"
        raise SystemExit(f"no eval CSV matching {pattern}")
    return cands[-1]


def read_rows(paths):
    rows = []
    for p in paths:
        if not os.path.exists(p):
            raise SystemExit(f"no such file: {p}")
        with open(p, newline="") as f:
            r = list(csv.DictReader(f))
        if not r:
            raise SystemExit(f"no data rows in {p}")
        # Old schema had no z columns; fill zeros so downstream stats work.
        for row in r:
            for k in ("cmd_z", "meas_z", "err_z"):
                row.setdefault(k, "0")
        rows.extend(r)
    return rows


def stats(vals):
    n = len(vals)
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / n
    rms = math.sqrt(sum(v * v for v in vals) / n)
    a = sorted(abs(v) for v in vals)

    def pct(q):
        if n == 1:
            return a[0]
        i = q * (n - 1)
        lo = int(math.floor(i))
        hi = min(lo + 1, n - 1)
        return a[lo] + (a[hi] - a[lo]) * (i - lo)

    return dict(n=n, mean=mean, std=math.sqrt(var), rms=rms,
                mae=sum(a) / n, p50=pct(0.50), p90=pct(0.90),
                p95=pct(0.95), max=a[-1])


def table(title, rows):
    if not rows:
        return
    print(f"\n{title}  (n = {len(rows)})")
    print(f"  {'':10s} {'mean':>9s} {'std':>9s} {'RMS':>9s} {'MAE':>9s} "
          f"{'p50':>9s} {'p90':>9s} {'p95':>9s} {'max':>9s}")
    for key, label in AXES:
        s = stats([float(r[key]) for r in rows])
        print(f"  {label:10s} {s['mean']:+9.4f} {s['std']:9.4f} {s['rms']:9.4f} "
              f"{s['mae']:9.4f} {s['p50']:9.4f} {s['p90']:9.4f} "
              f"{s['p95']:9.4f} {s['max']:9.4f}")
    # planar distance error, the single number usually quoted
    d = [math.hypot(float(r["err_x"]), float(r["err_y"])) for r in rows]
    s = stats(d)
    print(f"  {'|xy| [m]':10s} {s['mean']:9.4f} {s['std']:9.4f} {s['rms']:9.4f} "
          f"{s['mae']:9.4f} {s['p50']:9.4f} {s['p90']:9.4f} "
          f"{s['p95']:9.4f} {s['max']:9.4f}")


def worst(rows, n):
    keyed = sorted(rows, key=lambda r: -math.hypot(float(r["err_x"]), float(r["err_y"])))
    print(f"\nworst {min(n, len(rows))} steps by |xy| error")
    print(f"  {'step':>5s} {'foot':>4s} {'cmd(x,y,yaw)':>26s} {'err(x,y,yaw)':>26s} {'|xy|':>8s}")
    for r in keyed[:n]:
        c = (float(r["cmd_x"]), float(r["cmd_y"]), float(r["cmd_yaw"]))
        e = (float(r["err_x"]), float(r["err_y"]), float(r["err_yaw"]))
        print(f"  {r['step']:>5s} {r['foot']:>4s} "
              f"({c[0]:+.3f},{c[1]:+.3f},{c[2]:+.3f})".rjust(27) +
              f"({e[0]:+.3f},{e[1]:+.3f},{e[2]:+.3f})".rjust(27) +
              f"{math.hypot(e[0], e[1]):8.4f}")


def write_plot(rows, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print(f"\n[plot] matplotlib not available, skipping {path}")
        return
    steps = [int(r["step"]) for r in rows]
    n_err = len(AXES)
    fig, ax = plt.subplots(n_err + 1, 1, figsize=(9, 2.2 * (n_err + 1)), sharex=True)
    for i, (key, label) in enumerate(AXES):
        ax[i].plot(steps, [float(r[key]) for r in rows], ".-", lw=0.8, ms=3)
        ax[i].axhline(0, color="k", lw=0.5)
        ax[i].set_ylabel(f"err {label}")
        ax[i].grid(alpha=0.3)
    ax[n_err].plot(steps, [math.hypot(float(r["err_x"]), float(r["err_y"])) for r in rows],
                   ".-", lw=0.8, ms=3, color="tab:red")
    ax[n_err].set_ylabel("|xy| [m]")
    ax[n_err].set_xlabel("footstep")
    ax[n_err].grid(alpha=0.3)
    fig.suptitle("Footstep landing error (commanded - measured, stance frame)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"\n[plot] wrote {path}")


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description="Summarize footstep landing error from footstep_eval[_mys].csv.")
    p.add_argument("csv", nargs="*", default=[],
                   help="eval CSV path(s); several are pooled")
    p.add_argument("--mys", action="store_true",
                   help="use newest log/footstep_eval_mys_*.csv when no csv args given")
    p.add_argument("--worst", type=int, default=5,
                   help="how many worst steps to list (0 to skip)")
    p.add_argument("--plot", nargs="?", const="", default=None, metavar="PNG",
                   help="also write a plot (default path depends on --mys)")
    args = p.parse_args()

    paths = args.csv if args.csv else [latest_eval_csv(args.mys)]
    plot_path = None
    if args.plot is not None:
        plot_path = args.plot if args.plot else (
            DEFAULT_PLOT_MYS if args.mys or any("mys" in os.path.basename(p) for p in paths)
            else DEFAULT_PLOT)

    rows = read_rows(paths)
    print(f"footstep landing error (commanded - measured, stance frame)")
    print(f"source: {', '.join(paths)}")

    table("all steps", rows)
    right = [r for r in rows if r["foot"].strip().upper() == "R"]
    left = [r for r in rows if r["foot"].strip().upper() == "L"]
    if right and left:
        table("right swing", right)
        table("left swing", left)

    if args.worst > 0:
        worst(rows, args.worst)
    if plot_path:
        write_plot(rows, plot_path)
