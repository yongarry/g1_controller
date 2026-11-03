#!/usr/bin/env python3
# Copyright (c) 2025, DYROS.
#
# ArUco definitions shared by gen_aruco_footstep_scene.py (textures) and
# aruco_footstep_perception.py (detector); also recorded in config/aruco_board.json.
#
# Target frame: origin = footstep center on the top surface, x = walking
# direction, y = left, z = up. Marker ids = 4*target_index + j, centers
# j=0 (+c,+c), 1 (+c,-c), 2 (-c,-c), 3 (-c,+c), c = marker_spread.
# Printed/real targets: image-up = +x_target, image-left = +y_target; sim
# textures are pre-rotated 90° CW to match (see write_marker_textures).
# OpenCV corners are [TL, TR, BR, BL] of the marker image, whatever the view rotation.

import json
import os

import numpy as np

try:
    import cv2
except ImportError:  # board-geometry-only users (no texture gen / detection)
    cv2 = None

# Defaults; overridable via gen_aruco_footstep_scene.py and recorded in the board JSON.
DEFAULT_DICT_BITS = 4      # 3-bit custom dicts mis-decode badly with many ids
DEFAULT_DICT_SIZE = 250    # number of ids in the dictionary
DEFAULT_DICT_SEED = 7      # extendDictionary is deterministic given the seed
DEFAULT_MARKER_SIZE = 0.03     # [m] black border side length ("3x3 cm")
DEFAULT_MARKER_SPREAD = 0.04   # [m] |x|=|y| distance of marker centers
DEFAULT_QUIET_MODULES = 1      # white quiet-zone width in marker modules
MARKERS_PER_TARGET = 4

# MuJoCo +Z face maps PNG row 0 toward +y; write_marker_textures() rotates
# 90° CW on top of this. Validated by cmd/test_aruco_sim_render.py.
MARKER_TEXTURE_FLIP_V = False


def make_dictionary(bits=DEFAULT_DICT_BITS, size=DEFAULT_DICT_SIZE,
                    seed=DEFAULT_DICT_SEED):
    """Deterministic ArUco dictionary shared by generator & detector.

    bits == 4 uses the smallest predefined DICT_4X4_* that fits (better
    inter-marker distance than extendDictionary, which is weak for many ids).
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


def detector_parameters(min_marker_perimeter_rate=0.04, fast=False):
    """Shared detector tuning; min perimeter rejects tiny blobs a small
    dictionary could hallucinate into valid ids. fast=True is for half-res passes.
    """
    params = cv2.aruco.DetectorParameters()
    params.minMarkerPerimeterRate = float(min_marker_perimeter_rate)
    if fast:
        # One mid-size threshold is enough once the image is downscaled.
        params.adaptiveThreshWinSizeMin = 5
        params.adaptiveThreshWinSizeMax = 15
        params.adaptiveThreshWinSizeStep = 10
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        params.minMarkerPerimeterRate = max(
            float(min_marker_perimeter_rate), 0.05)
    else:
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # Fewer iterations than default: negligible loss for 3 cm markers.
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 20
        params.cornerRefinementMinAccuracy = 0.05
    return params


def marker_centers(spread=DEFAULT_MARKER_SPREAD):
    """Marker-center (x, y) offsets in the target frame, index j = 0..3."""
    c = float(spread)
    return [(+c, +c), (+c, -c), (-c, -c), (-c, +c)]


def marker_object_points(center_xy, marker_size=DEFAULT_MARKER_SIZE):
    """3D corners (target frame, z=0) in detectMarkers order [TL, TR, BR, BL]."""
    cx, cy = center_xy
    h = 0.5 * float(marker_size)
    return np.array([
        [cx + h, cy + h, 0.0],
        [cx + h, cy - h, 0.0],
        [cx - h, cy - h, 0.0],
        [cx - h, cy + h, 0.0],
    ], dtype=np.float64)


def marker_image(dictionary, marker_id, modules_px=32,
                 quiet_modules=DEFAULT_QUIET_MODULES, bits=DEFAULT_DICT_BITS,
                 flip_v=MARKER_TEXTURE_FLIP_V):
    """Marker image incl. white quiet zone, as a texture.

    Returns (img, scale); the textured geom must be scale * marker_size so the
    black border keeps its physical size.
    """
    n_marker = bits + 2
    n_total = n_marker + 2 * quiet_modules
    img = cv2.aruco.generateImageMarker(dictionary, int(marker_id),
                                        n_marker * modules_px)
    pad = quiet_modules * modules_px
    img = cv2.copyMakeBorder(img, pad, pad, pad, pad,
                             cv2.BORDER_CONSTANT, value=255)
    if flip_v:
        img = np.flipud(img)
    return img, n_total / float(n_marker)


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
