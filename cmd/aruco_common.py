#!/usr/bin/env python3
# Copyright (c) 2025, DYROS.
#
# Shared ArUco definitions for the vision-based footstep pipeline:
#   * cmd/gen_aruco_footstep_scene.py  - writes marker textures + scene geoms
#   * cmd/aruco_footstep_perception.py - detects markers and estimates targets
#
# Everything that must agree between the texture generator and the detector
# (dictionary, marker layout on a footstep target, corner conventions) lives
# here and in the generated board file (config/aruco_board.json).
#
# Conventions
# -----------
# Target frame (one per footstep target):
#   origin  = footstep center, ON the top surface
#   x       = footstep yaw direction (walking direction)
#   y       = left
#   z       = up
# Four square markers per target, ids = 4*target_index + j:
#   j=0: front-left  (+c, +c)      j=1: front-right (+c, -c)
#   j=2: back-right  (-c, -c)      j=3: back-left   (-c, +c)
# where c = marker_spread and marker black-border side length = marker_size.
# Every marker is axis-aligned with the target frame.  On the printable sheet
# (+x = arrow up, +y = left) and on a correctly mounted real target:
#   image-up   = +x_target  (OpenCV row 0 toward walking direction)
#   image-left = +y_target
# MuJoCo sim textures are pre-rotated 90° CW in gen_aruco_footstep_scene.py so
# the offscreen render matches the same layout (see write_marker_textures).
#
# OpenCV canonical marker corners (detectMarkers order) are
#   [top-left, top-right, bottom-right, bottom-left]
# of the marker image, independent of the viewing rotation (the detector
# decodes the marker's own orientation from its bits).

import json
import os

import numpy as np

try:
    import cv2
except ImportError:  # board-geometry-only users (no texture gen / detection)
    cv2 = None

# ---------------------------------------------------------------------------
# Defaults (all overridable via gen_aruco_footstep_scene.py CLI, recorded in
# the board JSON, and read back by the perception node)
# ---------------------------------------------------------------------------
DEFAULT_DICT_BITS = 4      # 4x4-bit markers (standard DICT_4X4_*: large, well-
                           # separated id space; a 3-bit custom dict with many
                           # ids mis-decodes badly - see make_dictionary)
DEFAULT_DICT_SIZE = 250    # number of ids in the dictionary
DEFAULT_DICT_SEED = 7      # extendDictionary is deterministic given the seed
DEFAULT_MARKER_SIZE = 0.03     # [m] black border side length ("3x3 cm")
DEFAULT_MARKER_SPREAD = 0.04   # [m] |x|=|y| distance of marker centers
DEFAULT_QUIET_MODULES = 1      # white quiet-zone width in marker modules
MARKERS_PER_TARGET = 4

# If True, marker texture PNGs are flipped vertically before being written.
# MuJoCo +Z face maps PNG row 0 toward +y; a further 90° CW rotation is
# applied in write_marker_textures() so sim matches the printable sheet layout
# (image-up = +x_target).  Validated by cmd/test_aruco_sim_render.py.
MARKER_TEXTURE_FLIP_V = False


def make_dictionary(bits=DEFAULT_DICT_BITS, size=DEFAULT_DICT_SIZE,
                    seed=DEFAULT_DICT_SEED):
    """Deterministic ArUco dictionary shared by generator & detector.

    bits == 4 uses the standard predefined DICT_4X4_{50,100,250,1000}
    (smallest one that fits `size` ids): these are designed for maximum
    inter-marker distance, unlike extendDictionary output. Other bit sizes
    fall back to a deterministic custom dictionary (weak for many ids -
    a 3-bit dictionary is only usable with a handful of ids).
    """
    if cv2 is None:
        raise ImportError("OpenCV (cv2) with aruco is required")
    if bits == 4:
        for n, name in ((50, cv2.aruco.DICT_4X4_50),
                        (100, cv2.aruco.DICT_4X4_100),
                        (250, cv2.aruco.DICT_4X4_250),
                        (1000, cv2.aruco.DICT_4X4_1000)):
            if size <= n:
                return cv2.aruco.getPredefinedDictionary(name)
        raise ValueError(f"too many ids for DICT_4X4 ({size} > 1000)")
    return cv2.aruco.extendDictionary(size, bits, cv2.aruco.Dictionary(), seed)


def detector_parameters(min_marker_perimeter_rate=0.04):
    """Shared detector tuning (subpixel corners; reject tiny blobs that a
    small-dictionary decoder could hallucinate into valid ids)."""
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.minMarkerPerimeterRate = float(min_marker_perimeter_rate)
    return params


def marker_centers(spread=DEFAULT_MARKER_SPREAD):
    """Marker-center (x, y) offsets in the target frame, index j = 0..3."""
    c = float(spread)
    return [(+c, +c), (+c, -c), (-c, -c), (-c, +c)]


def marker_object_points(center_xy, marker_size=DEFAULT_MARKER_SIZE):
    """3D corner positions (target frame, z=0 top surface) of one marker.

    Order matches cv2.aruco detectMarkers corners:
      [top-left, top-right, bottom-right, bottom-left]
    with image-up = +x_target, image-left = +y_target (print-sheet layout).
    """
    cx, cy = center_xy
    h = 0.5 * float(marker_size)
    return np.array([
        [cx + h, cy + h, 0.0],   # top-left    (+x, +y)
        [cx + h, cy - h, 0.0],   # top-right   (+x, -y)
        [cx - h, cy - h, 0.0],   # bottom-right (-x, -y)
        [cx - h, cy + h, 0.0],   # bottom-left  (-x, +y)
    ], dtype=np.float64)


def marker_image(dictionary, marker_id, modules_px=32,
                 quiet_modules=DEFAULT_QUIET_MODULES, bits=DEFAULT_DICT_BITS,
                 flip_v=MARKER_TEXTURE_FLIP_V):
    """Marker image incl. white quiet zone, ready to be used as a texture.

    Returns (img, scale) where scale = image_side / marker_side, i.e. the
    factor by which the textured geom must be larger than marker_size so the
    black border keeps its physical size.
    """
    n_marker = bits + 2                     # bit grid + black border modules
    n_total = n_marker + 2 * quiet_modules  # + white quiet zone
    img = cv2.aruco.generateImageMarker(dictionary, int(marker_id),
                                        n_marker * modules_px)
    pad = quiet_modules * modules_px
    img = cv2.copyMakeBorder(img, pad, pad, pad, pad,
                             cv2.BORDER_CONSTANT, value=255)
    if flip_v:
        img = np.flipud(img)
    return img, n_total / float(n_marker)


# ---------------------------------------------------------------------------
# Board file (config/aruco_board.json)
# ---------------------------------------------------------------------------
def save_board(path, targets, dict_bits, dict_size, dict_seed,
               marker_size, marker_spread, quiet_modules):
    """targets: list of dicts with keys index, foot, world{x,y,z,yaw}."""
    board = {
        "dictionary": {"bits": int(dict_bits), "size": int(dict_size),
                       "seed": int(dict_seed)},
        "marker_size": float(marker_size),
        "marker_spread": float(marker_spread),
        "quiet_zone_modules": int(quiet_modules),
        "markers_per_target": MARKERS_PER_TARGET,
        "targets": [
            {
                "index": int(t["index"]),
                "foot": t["foot"],
                "marker_ids": [MARKERS_PER_TARGET * int(t["index"]) + j
                               for j in range(MARKERS_PER_TARGET)],
                "world": {k: float(v) for k, v in t["world"].items()},
            }
            for t in targets
        ],
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        json.dump(board, f, indent=2)
    return board


def load_board(path):
    with open(path) as f:
        return json.load(f)


def board_object_points(board):
    """{marker_id: (target_index, 4x3 object points in target frame)}."""
    spread = board["marker_spread"]
    msize = board["marker_size"]
    centers = marker_centers(spread)
    out = {}
    for t in board["targets"]:
        for j, mid in enumerate(t["marker_ids"]):
            out[int(mid)] = (int(t["index"]),
                             marker_object_points(centers[j], msize))
    return out
