#!/usr/bin/env python3
# Copyright (c) 2025, DYROS.
#
# Convert a Footstep local command CSV (config/footcommands.csv) into a
# MindYourStep replay CSV (config/footcommands_mys.csv) so both controllers
# execute the same stance-frame footholds for experiment comparison.
#
# Footstep and MindYourStep share the same local-command semantics:
#   foot,step_x,step_y,step_z,step_yaw[,ssp_t,dsp_t,height]
#     foot   : L / R  -> which foot swings
#     step_* : displacement of the swing foot in the stance-foot yaw frame
#              (step_y is a positive magnitude; the controller applies +/-)
#     step_yaw : swing-foot yaw RELATIVE to the current stance foot
#
# ---------------------------------------------------------------------------
# Heading
# ---------------------------------------------------------------------------
# CSV mode in both controllers uses each row as a stance-frame command:
#   step_yaw is THAT step's heading relative to the current stance foot.
# It is not a running sum. The policy / Footstep planner sees only this
# relative value. Summing the column ("accumulated heading" below) is only a
# diagnostic of the path if every increment were tracked perfectly.
#
#   keep   (default) copy step_x / step_y / step_yaw through. Same command
#          Footstep csv mode would run, except step_yaw is clipped to the
#          trained yaw_range_deg +/-15 deg, and step_x keeps the first
#          non-zero sign so consecutive forward/back flips are removed.
#   anchor rewrite step_yaw so the feet hold a global heading. Training
#          samples yaw back to theta_feet, so this is the strafing equivalent.
#          Clipped to yaw_range afterwards.

#
# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------
# MindYourStep advances one foothold every HALF gait cycle, i.e. every
# 0.5 / gait_frequency seconds. The policy is trained with gait_frequency = 1.0
# (MindYourStepFootCommandCfg.gait_frequency_range), so one foothold takes
# 0.5 s and gait.gait_frequency in deploy.yaml MUST stay at 1.0 -- changing it
# desynchronizes the policy from the gait clock it learned. ssp_t/dsp_t/height
# are copied for eval logging only and do NOT drive the MYS gait clock, so a
# plan authored for 1.0 s footholds is replayed at 0.5 s per foothold.
#
# Usage:
#   python3 cmd/convert2mys.py
#   python3 cmd/convert2mys.py --heading anchor
#   python3 cmd/convert2mys.py --input config/footcommands.csv \
#                              --output config/footcommands_mys.csv

import argparse
import csv
import math
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJ_DIR = os.path.dirname(_THIS_DIR)
DEFAULT_INPUT = os.path.join(_PROJ_DIR, "config", "footcommands.csv")
DEFAULT_OUTPUT = os.path.join(_PROJ_DIR, "config", "footcommands_mys.csv")

OUT_COLS = ["foot", "step_x", "step_y", "step_z", "step_yaw", "ssp_t", "dsp_t", "height"]

# GoalDoubleFootPlacement in conf_g1.yaml (training envelope).
TRAIN_D_MIN, TRAIN_D_MAX = 0.2, 0.5      # xy_distance_range
TRAIN_FEET_DISTANCE = 0.2                # lateral leg-crossing clip
TRAIN_YAW_ABS = math.radians(15.0)       # yaw_range_deg (beta)
TRAIN_GAIT_FREQUENCY = 1.0               # gait_frequency_range


def wrap_to_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clip_yaw(rows, yaw_abs):
    """Clamp step_yaw to the trained yaw_range_deg (+/-yaw_abs rad)."""
    n = 0
    for r in rows:
        y = max(-yaw_abs, min(yaw_abs, wrap_to_pi(r["step_yaw"])))
        if abs(y - r["step_yaw"]) > 1e-9:
            n += 1
        r["step_yaw"] = y
    return n


def same_x_sign(rows):
    """Keep every non-zero step_x on the first non-zero step's sign.

    Footstep samples RANGE_X across zero, so consecutive rows often flip
    forward/back. Training holds movement_direction fixed inside a gait.
    Magnitude is unchanged; only the sign is aligned. Zero (stop) rows stay 0.
    """
    sign = 0.0
    for r in rows:
        if abs(r["step_x"]) > 1e-9:
            sign = 1.0 if r["step_x"] > 0.0 else -1.0
            break
    if sign == 0.0:
        return 0
    n = 0
    for r in rows:
        if abs(r["step_x"]) < 1e-9:
            continue
        new_x = sign * abs(r["step_x"])
        if abs(new_x - r["step_x"]) > 1e-9:
            n += 1
        r["step_x"] = new_x
    return n


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
    bad = [i for i in range(1, len(rows)) if rows[i]["foot"] == rows[i - 1]["foot"]]
    if bad:
        print(f"[warn] feet do not strictly alternate at {len(bad)} row(s); "
              f"first at rows {bad[0]}/{bad[0] + 1} (both {rows[bad[0]]['foot']}). "
              f"The controller warns and applies them anyway.", file=sys.stderr)


def heading_profile(rows):
    """World heading if every per-step yaw were tracked (diagnostic only)."""
    psi, cum = 0.0, []
    for r in rows:
        psi += r["step_yaw"]
        cum.append(psi)
    return cum


def anchor_heading(rows, theta_feet=0.0):
    """Rewrite step_yaw so every swing foot lands on the global heading theta_feet.

    Mirrors the training goal sampler: the yaw target is measured back to a global
    heading rather than accumulated, so the relative command stops drifting.
    """
    stance_yaw = 0.0  # the foot the robot starts on defines the frame
    changed = 0
    for r in rows:
        new_yaw = wrap_to_pi(theta_feet - stance_yaw)
        if abs(new_yaw - r["step_yaw"]) > 1e-9:
            changed += 1
        r["step_yaw"] = new_yaw
        stance_yaw = wrap_to_pi(stance_yaw + new_yaw)  # the swing foot becomes stance
    return changed


def validate(rows, feet_distance, d_min, d_max, yaw_abs):
    """Report rows outside the envelope the MindYourStep policy was trained on."""
    narrow, far, near, wide_yaw = [], [], [], []
    for i, r in enumerate(rows, start=1):
        d = math.hypot(r["step_x"], r["step_y"])
        if r["step_y"] < feet_distance - 1e-9:
            narrow.append(i)
        if d > d_max + 1e-9:
            far.append(i)
        if d < d_min - 1e-9:
            near.append(i)
        if abs(r["step_yaw"]) > yaw_abs + 1e-9:
            wide_yaw.append(i)

    def report(name, bad, detail):
        if not bad:
            return
        head = ", ".join(str(b) for b in bad[:5])
        more = f" (+{len(bad) - 5} more)" if len(bad) > 5 else ""
        print(f"[warn] {len(bad)} row(s) {detail}: rows {head}{more}", file=sys.stderr)

    report("narrow", narrow,
           f"have |step_y| < feet_distance {feet_distance:.3f} m; training clips the "
           f"lateral offset to at least this, so the policy never saw a narrower stance")
    report("far", far,
           f"exceed the trained max step length {d_max:.3f} m "
           f"(sqrt(step_x^2 + step_y^2))")
    report("near", near,
           f"are shorter than the trained min step length {d_min:.3f} m")
    report("yaw", wide_yaw,
           f"exceed the trained per-step yaw range +/-{math.degrees(yaw_abs):.1f} deg")
    return len(narrow) + len(far) + len(near) + len(wide_yaw)


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
        description="Convert footcommands.csv -> footcommands_mys.csv for MYS CSV deploy.")
    p.add_argument("--input", "-i", default=DEFAULT_INPUT,
                   help="Footstep local command CSV")
    p.add_argument("--output", "-o", default=DEFAULT_OUTPUT,
                   help="MindYourStep replay CSV")
    p.add_argument("--heading", choices=("keep", "anchor"), default="keep",
                   help="keep: pass step_x/y/yaw through (default, same as Footstep csv). "
                        "anchor: rewrite step_yaw to hold a global heading.")
    p.add_argument("--theta-feet", type=float, default=0.0,
                   help="[rad] global foot heading used by --heading anchor "
                        "(0 = the heading the robot starts on).")
    p.add_argument("--feet-distance", type=float, default=TRAIN_FEET_DISTANCE,
                   help="[m] lateral clip used in training (validation only)")
    p.add_argument("--d-min", type=float, default=TRAIN_D_MIN,
                   help="[m] min trained step length (validation only)")
    p.add_argument("--d-max", type=float, default=TRAIN_D_MAX,
                   help="[m] max trained step length (validation only)")
    p.add_argument("--allow-x-flip", action="store_false",
                   help="keep original step_x signs (consecutive forward/back flips). "
                        "Default is to align all non-zero step_x to the first step's sign.")
    args = p.parse_args()

    rows = read_rows(args.input)
    check_alternation(rows)

    cum_before = heading_profile(rows)
    net_before, peak_before = cum_before[-1], max(abs(c) for c in cum_before)

    if args.heading == "anchor":
        changed = anchor_heading(rows, args.theta_feet)
    else:
        changed = 0

    n_clip = clip_yaw(rows, TRAIN_YAW_ABS)
    n_x = 0 if args.allow_x_flip else same_x_sign(rows)
    cum_after = heading_profile(rows)
    net_after, peak_after = cum_after[-1], max(abs(c) for c in cum_after)

    n_bad = validate(rows, args.feet_distance, args.d_min, args.d_max, TRAIN_YAW_ABS)
    write_rows(args.output, rows)

    t0 = rows[0]["ssp_t"] + 2.0 * rows[0]["dsp_t"]
    mys_step_t = 0.5 / TRAIN_GAIT_FREQUENCY

    print(f"[convert2mys] {len(rows)} steps: {args.input}")
    print(f"              -> {args.output}")
    print(f"              first foot={rows[0]['foot']}, heading mode={args.heading}"
          + (f" (rewrote {changed} step_yaw values)" if changed else "")
          + (f", clipped {n_clip} to +/-{math.degrees(TRAIN_YAW_ABS):.0f} deg" if n_clip else "")
          + (f", aligned {n_x} step_x signs" if n_x else ""))
    print(f"              accumulated heading: net {math.degrees(net_before):+.1f} deg, "
          f"peak |{math.degrees(peak_before):.1f}| deg"
          + (f"  ->  net {math.degrees(net_after):+.1f} deg, "
             f"peak |{math.degrees(peak_after):.1f}| deg"
             if n_clip or args.heading == "anchor" else ""))
    print(f"              timing: source plan {t0:.3f} s/step, MindYourStep replays at "
          f"{mys_step_t:.3f} s/step (gait.gait_frequency must stay "
          f"{TRAIN_GAIT_FREQUENCY:.1f} to match training)")
    yaws = [r["step_yaw"] for r in rows]
    print(f"              per-step yaw: [{min(yaws):+.3f}, {max(yaws):+.3f}] rad "
          f"([{math.degrees(min(yaws)):+.1f}, {math.degrees(max(yaws)):+.1f}] deg)")
    if args.heading == "keep" and abs(net_before) > math.pi:
        print(f"[note] summing the yaw column is {math.degrees(abs(net_before)):.0f} deg "
              f"(path heading if every increment is tracked). Each command is still "
              f"just that row's step_yaw vs current stance, same as Footstep csv.",
              file=sys.stderr)
    if n_bad:
        print(f"[warn] {n_bad} envelope violation(s) above; the policy may track those "
              f"steps poorly.", file=sys.stderr)


if __name__ == "__main__":
    main()
