#!/usr/bin/env python3
# Copyright (c) 2025, DYROS.
#
# Write a "rocky mountain" terrain into a MuJoCo scene XML from
# config/footcommands_global.csv: rock columns with heights interpolated from
# the footholds fill a corridor around the path, plus stepping stones
# (--offset-* in the foot yaw frame) and translucent foot-mesh targets on the
# raw CSV. Re-running strips every prior generated region and aruco side-files.
#
# Usage:
#   python3 cmd/gen_rocky_mountain.py
#   python3 cmd/gen_rocky_mountain.py --rock-shape cylinder --layout hex --spacing 0.14 --rock-width 0.17

import argparse
import math
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)
from gen_footstep_scene import (
    clear_aruco_artifacts, inject as _inject_shared, write_scene,
    read_targets, yaw_to_quat, stone_landing_poses, strip_start_platform,
    ROCKY_BEGIN as BEGIN_MARK, ROCKY_END as END_MARK,
)

_PROJ_DIR = os.path.dirname(_THIS_DIR)
_WS_DIR = os.path.dirname(_PROJ_DIR)
DEFAULT_CSV = os.path.join(_PROJ_DIR, "config", "footcommands_global.csv")
DEFAULT_XML = os.path.join(_WS_DIR, "unitree_mujoco", "unitree_robots", "g1", "scene_29dof_footstep.xml")

COLOR_STONE = "0.6 0.5 0.4 1"
# COLOR_STONE = "0.7 0.6 0.5 1"
# COLOR_GROUND = "0.7 0.6 0.5 1"
# COLOR_GROUND = "0.3 0.3 0.3 1"
COLOR_GROUND = "0.6 0.5 0.4 1"
COLOR_PLATFORM = COLOR_STONE
COLOR_TARGET_R = "0.6 0.2 0.25"
COLOR_TARGET_L = "0.2 0.25 0.6"
# CSV pos_x/y/z/yaw is the ankle_roll_link frame, so the foot mesh drops straight
# onto the target with no offset (its sole is 0.035 m below that origin).
MESH_TARGET_L = "left_ankle_roll_link"
MESH_TARGET_R = "right_ankle_roll_link"
ROCK_BASE_RGB = (0.6, 0.5, 0.4)
# ROCK_BASE_RGB = (0.48, 0.44, 0.40)   # brown-gray rock

DEFAULT_PLATFORM_SIZE = (0.1, 0.17)
DEFAULT_PLATFORM_TOP = 0.0
DEFAULT_PLATFORM_HALF_HEIGHT = 0.10  # 10 cm; top face always at z=0


def stone_local_x(cx, cy, sx, sy, yaw):
    """Cell (cx, cy) in the stone yaw frame: +x forward along yaw."""
    dx, dy = cx - sx, cy - sy
    return math.cos(yaw) * dx + math.sin(yaw) * dy


def directional_flush_drop(cx, cy, sx, sy, yaw, drop):
    """--flush-drop behind the stone (local -x); 0 ahead (local +x)."""
    if stone_local_x(cx, cy, sx, sy, yaw) < 0.0:
        return drop
    return 0.0


def cell_hash(i, j, seed, salt=0.0):
    """Deterministic pseudo-random value in [0, 1) for grid cell (i, j)."""
    v = math.sin(i * 12.9898 + j * 78.233 + seed * 37.719 + salt * 4.581) * 43758.5453
    return v - math.floor(v)


def rock_rgba(top, z_lo, z_span, i, j, seed, salt=2.0):
    """Height-shaded rock color with per-cell variation → 'rgba=\"...\"'."""
    rel = (top - z_lo) / z_span
    shade = 0.72 + 0.38 * max(0.0, min(1.0, rel))
    var = (cell_hash(i, j, seed, salt=salt) - 0.5) * 0.12
    r = max(0.0, min(1.0, (ROCK_BASE_RGB[0] + var) * shade))
    g = max(0.0, min(1.0, (ROCK_BASE_RGB[1] + var * 0.9) * shade))
    b = max(0.0, min(1.0, (ROCK_BASE_RGB[2] + var * 0.8) * shade))
    return f'rgba="{r:.3f} {g:.3f} {b:.3f} 1"'


def point_segment_dist(px, py, ax, ay, bx, by):
    """2D distance from point P to segment AB."""
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12:
        return math.hypot(apx, apy)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    return math.hypot(px - (ax + t * abx), py - (ay + t * aby))


def path_distance(px, py, path):
    """Min 2D distance from (px, py) to the footstep polyline."""
    d = float("inf")
    for k in range(len(path) - 1):
        d = min(d, point_segment_dist(px, py, path[k][0], path[k][1], path[k + 1][0], path[k + 1][1]))
    return d


def idw_height(px, py, points, power=2.0, eps=0.02):
    """Inverse-distance-weighted surface height at (px, py). points: list of (x, y, z). eps keeps weights finite on top of a point."""
    wsum = 0.0
    zsum = 0.0
    for (x, y, z) in points:
        d2 = (px - x) ** 2 + (py - y) ** 2 + eps * eps
        w = 1.0 / (d2 ** (power / 2.0))
        wsum += w
        zsum += w * z
    return zsum / wsum


HEX_ROW_DY = math.sqrt(3.0) / 2.0  # triangular-lattice row pitch, in cell units


def lattice_cells(x0, y0, x1, y1, cell, layout):
    """Yield (i, j, cx, cy) column centers over the box; 'hex' is a staggered triangular lattice (tighter packing for round columns)."""
    row_dy = cell * (HEX_ROW_DY if layout == "hex" else 1.0)
    nx = max(1, int(math.ceil((x1 - x0) / cell)) + (1 if layout == "hex" else 0))
    ny = max(1, int(math.ceil((y1 - y0) / row_dy)))
    for j in range(ny):
        cy = y0 + (j + 0.5) * row_dy
        x_off = 0.5 * cell if (layout == "hex" and j % 2) else 0.0
        for i in range(nx):
            yield i, j, x0 + (i + 0.5) * cell + x_off, cy


def column_width(args):
    """Rock column width [m]: --rock-width, else --spacing * --overlap."""
    if args.rock_width is not None:
        return args.rock_width
    return args.spacing * args.overlap


def required_width(shape, layout, spacing):
    """Narrowest gap-free column width [m] for a lattice at this spacing."""
    return spacing * required_overlap(shape, layout)


def required_overlap(shape, layout):
    """Smallest gap-free --overlap: a cylinder must reach the worst-covered point (cell/sqrt(2) on grid, cell/sqrt(3) on hex); boxes seal at 1.0."""
    if shape == "box":
        return 1.0
    return 2.0 / math.sqrt(3.0) if layout == "hex" else math.sqrt(2.0)


def build_rock_cells(stones, args, z_ground):
    """XML geom lines for the rock columns under the footsteps (stones from stone_landing_poses)."""
    # Virtual start point so the terrain descends smoothly onto the spawn platform.
    stone_pose = [(s["sx"], s["sy"], s["sz"], s["yaw"]) for s in stones]
    stone_xyz = [(sx, sy, sz) for sx, sy, sz, _ in stone_pose]
    points = [(0.0, 0.0, args.platform_top)] + stone_xyz
    path = [(p[0], p[1]) for p in points]

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    z_lo, z_hi = min(zs), max(zs)
    z_span = max(z_hi - z_lo, 1e-6)

    cell = args.spacing
    half = 0.5 * column_width(args)
    margin = args.terrain_margin
    x0, x1 = min(xs) - margin, max(xs) + margin
    y0, y1 = min(ys) - margin, max(ys) + margin

    w0, w1 = args.corridor  # full-height / zero-height distances from the path
    lines = []
    n_cells = 0
    reach = half * math.sqrt(2.0) if args.rock_shape == "box" else half
    start_keep_out = args.start_clear + reach

    for i, j, cx, cy in lattice_cells(x0, y0, x1, y1, cell, args.layout):
        # Keep rock out from behind the spawn so the robot never spawns inside it; `reach` covers any yaw.
        if args.start_clear > 0.0 and cx < args.start_clear_front and math.hypot(cx, cy) < start_keep_out:
            continue

        d = path_distance(cx, cy, path)
        if d >= w1:
            continue
        if d <= w0:
            falloff = 1.0
        else:
            falloff = 0.5 * (1.0 + math.cos(math.pi * (d - w0) / (w1 - w0)))

        z_surf = idw_height(cx, cy, points, power=args.idw_power)
        u = cell_hash(i, j, args.seed)
        base_top = z_surf - args.clearance - args.noise * u

        # Min over stones in range so a higher neighbor never blocks a lower landing.
        r0 = args.flush_radius
        r1 = r0 + args.flush_blend
        z_in = [
            z - directional_flush_drop(cx, cy, sx_, sy_, yaw, args.flush_drop)
            for (sx_, sy_, z, yaw) in stone_pose
            if (cx - sx_) ** 2 + (cy - sy_) ** 2 <= r0 * r0
        ]
        if z_in:
            top = min(z_in)
        else:
            ds_min, z_near, yaw_near, sx_n, sy_n = min(
                (math.hypot(cx - sx_, cy - sy_), z, yaw, sx_, sy_)
                for (sx_, sy_, z, yaw) in stone_pose
            )
            if ds_min < r1:
                t = 0.5 * (1.0 - math.cos(math.pi * (ds_min - r0) / args.flush_blend))
                drop = directional_flush_drop(cx, cy, sx_n, sy_n, yaw_near, args.flush_drop)
                top = (1.0 - t) * (z_near - drop) + t * base_top
            else:
                top = base_top
        top = z_ground + (top - z_ground) * falloff

        h = top - z_ground
        if h < args.min_height:
            continue
        n_cells += 1

        hz = 0.5 * h
        zc = z_ground + hz

        rgba = rock_rgba(top, z_lo, z_span, i, j, args.seed)

        if args.rock_shape == "cylinder":
            lines.append(f'    <geom name="rock_{n_cells:04d}" type="cylinder" group="4" size="{half:.4f} {hz:.4f}" pos="{cx:.4f} {cy:.4f} {zc:.4f}" {rgba}/>')
        else:
            # small random yaw makes the lattice read as tumbled rock
            yaw = (cell_hash(i, j, args.seed, salt=1.0) - 0.5) * 2.0 * args.max_cell_yaw
            qw, qx, qy, qz = yaw_to_quat(yaw)
            lines.append(f'    <geom name="rock_{n_cells:04d}" type="box" group="4" size="{half:.4f} {half:.4f} {hz:.4f}" pos="{cx:.4f} {cy:.4f} {zc:.4f}" quat="{qw:.6f} {qx:.6f} {qy:.6f} {qz:.6f}" {rgba}/>')
    return lines


def build_scene(rows, off, args):
    # Stones/rocks use yaw-frame XY offset; markers stay on raw CSV targets.
    stones = stone_landing_poses(rows, off)
    zs_top = [s["sz"] for s in stones]
    z_min = min(zs_top)
    platform_top = args.platform_top

    z_ground = platform_top if z_min >= platform_top else z_min
    plat_hz = DEFAULT_PLATFORM_HALF_HEIGHT
    plat_zc = -plat_hz  # top face at z=0
    plat_x = off[0]    # world +x only

    xs = [s["sx"] for s in stones]
    ys = [s["sy"] for s in stones]
    plane_cx = 0.5 * (min(xs) + max(xs))
    plane_cy = 0.5 * (min(ys) + max(ys))
    plane_hx = 0.5 * (max(xs) - min(xs)) + args.plane_margin
    plane_hy = 0.5 * (max(ys) - min(ys)) + args.plane_margin

    plat_hx, plat_hy = args.platform_size

    lines = [
        f'    <geom name="start_platform" type="box" group="3" size="{plat_hx:.4f} {plat_hy:.4f} {plat_hz:.4f}" pos="{plat_x:.4f} 0 {plat_zc:.4f}" rgba="{COLOR_PLATFORM}"/>',
        f'    <geom name="footstep_ground" type="plane" pos="{plane_cx:.4f} {plane_cy:.4f} {z_ground:.4f}" size="{plane_hx:.4f} {plane_hy:.4f} 0.1" rgba="{COLOR_GROUND}" material="MatPlane2" group="2"/>',
    ]

    rock_lines = build_rock_cells(stones, args, z_ground)
    lines += rock_lines

    hx, hy = args.stone_size
    z_lo = min(zs_top + [platform_top])
    z_span = max(max(zs_top) - z_lo, 1e-6)
    n_stones = 0
    for i, s in enumerate(stones):
        x, y, z_top = s["sx"], s["sy"], s["sz"]
        height = z_top - z_ground
        if height < 1e-4:
            continue  # sits on the ground plane; no pillar needed
        n_stones += 1
        hz = 0.5 * height
        zc = z_ground + hz
        rgba = rock_rgba(z_top, z_lo, z_span, i, 0, args.seed, salt=3.0)
        if args.stone_shape == "cylinder":
            lines.append(f'    <geom name="stone_{i:02d}" type="cylinder" group="3" size="{hx:.4f} {hz:.4f}" pos="{x:.4f} {y:.4f} {zc:.4f}" {rgba}/>')
        else:
            qw, qx, qy, qz = yaw_to_quat(s["yaw"])
            lines.append(f'    <geom name="stone_{i:02d}" type="box" group="3" size="{hx:.4f} {hy:.4f} {hz:.4f}" pos="{x:.4f} {y:.4f} {zc:.4f}" quat="{qw:.6f} {qx:.6f} {qy:.6f} {qz:.6f}" {rgba}/>')

    # Translucent foot-link mesh at each raw CSV target (no offset).
    a = args.marker_alpha
    for i, r in enumerate(rows):
        x, y, z = r["x"], r["y"], r["z"]
        left = r["foot"] == "L"
        color = COLOR_TARGET_L if left else COLOR_TARGET_R
        mesh = MESH_TARGET_L if left else MESH_TARGET_R
        qw, qx, qy, qz = yaw_to_quat(r["yaw"])
        lines.append(f'    <geom name="target_{r["foot"]}_{i:02d}" type="mesh" mesh="{mesh}" group="5" pos="{x:.4f} {y:.4f} {z:.4f}" quat="{qw:.6f} {qx:.6f} {qy:.6f} {qz:.6f}" rgba="{color} {a:.2f}" contype="0" conaffinity="0"/>')

    block = f"    {BEGIN_MARK}\n" + "\n".join(lines) + f"\n    {END_MARK}"
    return block, z_ground, z_min, len(rock_lines), n_stones


def inject(xml_text, block):
    """Strip footstep/rocky/aruco regions, then insert this script's block."""
    return _inject_shared(xml_text, block)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Generate a rocky-mountain footstep terrain in a MuJoCo scene XML.")
    p.add_argument("--csv", default=DEFAULT_CSV, help="footcommands_global.csv path")
    p.add_argument("--xml", default=DEFAULT_XML, help="MuJoCo scene XML to edit")

    t = p.add_argument_group("terrain")
    t.add_argument("--rock-shape", choices=["box", "cylinder"], default="box", help="box: yaw-jittered blocks (default); cylinder: round basalt-like columns (yaw has no effect, and they need to be wider than the spacing so neighbors still touch)")
    t.add_argument("--layout", choices=["grid", "hex"], default="grid", help="grid: square lattice (default); hex: staggered triangular lattice, which packs round columns much tighter (needs 1.155x the spacing, not 1.414x)")
    t.add_argument("--spacing", "--cell", type=float, default=0.2, dest="spacing", metavar="M", help="distance between neighboring rock columns [m]")
    t.add_argument("--rock-width", type=float, default=None, metavar="M", help="width of each rock column [m]: box edge-to-edge, or cylinder diameter. Set this to size the columns directly; leave it out to derive the width from --spacing x --overlap instead")
    t.add_argument("--overlap", type=float, default=1., help="fallback way to size columns when --rock-width is not given: width = spacing * overlap. 1.0 makes columns exactly meet, >1 overlaps them to hide seams")

    t.add_argument("--start-clear", type=float, default=0.3, metavar="M", help="radius behind the spawn origin kept free of rock columns [m], so the robot starts on the bare platform instead of in the rocks (0 disables)")
    t.add_argument("--start-clear-front", type=float, default=0.3, metavar="X", help="forward edge of that clear zone [x, m]. The default 0 clears only behind the spawn point and leaves the rock the robot walks onto intact; raise it to also clear ahead, or pass a large value for a full circle")

    t.add_argument("--clearance", type=float, default=0., help="how far the filler rock stays below the interpolated foot-landing surface [m], outside the flush zones")
    t.add_argument("--flush-radius", type=float, default=0.3, help="rock cells within this distance of a stepping stone match its landing height exactly [m]")
    t.add_argument("--flush-blend", type=float, default=0.3, help="transition band from flush height back to the clearance surface [m]")
    t.add_argument("--flush-drop", type=float, default=0.2, help="how far below the stone top the flush rock sits on the approach side (stone local -x) [m]; ahead (local +x) is 0 so the landing is not undercut")

    t.add_argument("--noise", type=float, default=0.03, help="max downward height jitter per rock column [m]")
    t.add_argument("--max-cell-yaw", type=float, default=1.79, help="max random yaw of each rock column [rad]")
    t.add_argument("--corridor", nargs=2, type=float, default=[0.45, 10.5], metavar=("W0", "W1"), help="distance from the footstep path: full height inside W0, tapered to ground at W1 [m]")
    t.add_argument("--terrain-margin", type=float, default=1.0, help="extent of the terrain grid beyond the footstep bounding box [m]")
    t.add_argument("--idw-power", type=float, default=2.0, help="inverse-distance-weighting exponent for the surface")
    t.add_argument("--min-height", type=float, default=0.015, help="rock columns shorter than this are skipped [m]")
    t.add_argument("--seed", type=float, default=7.0, help="seed for the deterministic per-cell noise")

    s = p.add_argument_group("stepping stones & markers")
    s.add_argument("--stone-shape", choices=["box", "cylinder"], default="box", help="box: yaw-aligned rectangular foothold (default); cylinder: round pillar of radius HX (yaw ignored)")
    s.add_argument("--stone-size", nargs=2, type=float, default=[0.1, 0.1], metavar=("HX", "HY"), help="stepping-stone half-extents [m] (cylinder: radius=HX, HY unused)")
    s.add_argument("--marker-alpha", type=float, default=0.5, help="opacity of the foot-target meshes (left red, right blue)")

    g = p.add_argument_group("platform & ground")
    g.add_argument("--plane-margin", type=float, default=1.0, help="extra half-length around the footstep bounding box for the ground plane [m]")
    g.add_argument("--platform-size", nargs=2, type=float, default=list(DEFAULT_PLATFORM_SIZE), metavar=("HX", "HY"), help="spawn platform horizontal half-extents [m]")
    g.add_argument("--platform-top", type=float, default=DEFAULT_PLATFORM_TOP, help="spawn platform top face height [m] (default: z=0)")

    o = p.add_argument_group("offsets (stones/rocks only; foot markers stay on CSV targets)")
    # o.add_argument("--offset-x", type=float, default=0.02, help="stone XY offset in each foot's yaw frame, forward [m]")
    o.add_argument("--offset-x", type=float, default=0.035, help="stone XY offset in each foot's yaw frame, forward [m]")
    o.add_argument("--offset-y", type=float, default=0.0, help="stone XY offset in each foot's yaw frame, left [m]")
    # o.add_argument("--offset-z", type=float, default=-0.035, help="stone top height offset in world-up [m] (CSV pos_z is already the sole/terrain plane; do not re-apply the training ankle-to-sole -0.035)")
    o.add_argument("--offset-z", type=float, default=-0.03458, help="stone top height offset in world-up [m] (CSV pos_z is already the sole/terrain plane; do not re-apply the training ankle-to-sole -0.035)")

    args = p.parse_args()
    if args.corridor[1] <= args.corridor[0]:
        p.error("--corridor W1 must be greater than W0")

    width = column_width(args)
    if width <= 0.0:
        p.error("rock column width must be positive (check --rock-width / --overlap)")
    need = required_width(args.rock_shape, args.layout, args.spacing)
    if width < need - 1e-9:
        hint = ""
        if args.layout == "grid":
            hex_need = required_width(args.rock_shape, "hex", args.spacing)
            hint = f", or --layout hex which only needs {1000*hex_need:.0f} mm"
        print(f"warning: {args.rock_shape} columns {1000*width:.0f} mm wide on a {1000*args.spacing:.0f} mm {args.layout} lattice leave ~{1000*0.5*(need - width):.0f} mm gaps; use --rock-width {need:.3f}{hint}")

    rows = read_targets(args.csv)
    off = (args.offset_x, args.offset_y, args.offset_z)
    block, z_ground, z_min, n_rocks, n_stones = build_scene(rows, off, args)

    with open(args.xml) as f:
        xml_text = f.read()
    xml_text = strip_start_platform(xml_text)
    write_scene(args.xml, inject(xml_text, block))
    clear_aruco_artifacts(args.xml)

    ground_note = f"at platform_top={args.platform_top:.4f} (min pos_z={z_min:.4f} >= platform)" if z_min >= args.platform_top else f"z_ground={z_ground:.4f}"
    print(f"Rock columns: {args.rock_shape}, {1000*width:.0f} mm wide, {1000*args.spacing:.0f} mm apart, {args.layout} lattice")
    print(f"Wrote start_platform + ground plane + {n_rocks} {args.rock_shape} rock columns ({args.layout} lattice) + {n_stones} {args.stone_shape} stepping stones + {len(rows)} target markers into {args.xml} ({ground_note})")
