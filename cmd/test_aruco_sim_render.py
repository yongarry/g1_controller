#!/usr/bin/env python3
# Copyright (c) 2025, DYROS.
#
# End-to-end validation of the sim ArUco footstep pipeline WITHOUT DDS:
# load the footstep scene, pose the robot at the spawn keyframe, render the
# D435i camera offscreen, detect the markers, run per-target PnP, and compare
# the estimated target poses against the model ground truth.
#
# Run after cmd/gen_aruco_footstep_scene.py:
#   python3 cmd/test_aruco_sim_render.py [--save /tmp/d435i_view.png]

import argparse
import math
import os
import sys

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import aruco_common as ac
from aruco_footstep_perception import TargetEstimator

_PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_XML = os.path.join(_PROJ, "..", "unitree_mujoco", "unitree_robots",
                           "g1", "scene_29dof_footstep.xml")
DEFAULT_BOARD = os.path.join(_PROJ, "config", "aruco_board.json")

# OpenCV optical frame in the MuJoCo camera frame (x right in both;
# cv y down = -muj y, cv z forward = -muj z)
R_MUJ_CV = np.diag([1.0, -1.0, -1.0])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--xml", default=DEFAULT_XML)
    p.add_argument("--board", default=DEFAULT_BOARD)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--save", default="", help="save the annotated view PNG")
    args = p.parse_args()

    board = ac.load_board(args.board)
    obj_points = ac.board_object_points(board)

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "d435i")
    assert cam_id >= 0, "d435i camera not found in model"

    model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
    renderer = mujoco.Renderer(model, args.height, args.width)
    # Default vis option hides geom groups 3+; ArUco markers are group 4.
    scene_option = mujoco.MjvOption()
    for i in range(len(scene_option.geomgroup)):
        scene_option.geomgroup[i] = 1
    renderer.update_scene(data, camera="d435i", scene_option=scene_option)
    rgb = renderer.render()
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

    # intrinsics from fovy
    fovy = math.radians(model.cam_fovy[cam_id])
    fy = fx = (args.height / 2.0) / math.tan(fovy / 2.0)
    K = np.array([[fx, 0, args.width / 2.0],
                  [0, fy, args.height / 2.0],
                  [0, 0, 1.0]])

    # Same detector the perception node runs (not a standalone ArucoDetector).
    est = TargetEstimator(board, min_markers=2)
    corners, ids = est._detect(gray)
    ids = ids.ravel() if ids is not None else np.array([], int)
    print(f"detected {len(ids)} markers: {sorted(ids.tolist())}")

    # ground-truth camera pose (world)
    cw_p = data.cam_xpos[cam_id].copy()
    cw_R = data.cam_xmat[cam_id].reshape(3, 3).copy()
    cv_R = cw_R @ R_MUJ_CV          # world <- cv optical
    vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
    if len(ids):
        cv2.aruco.drawDetectedMarkers(vis, corners, ids.reshape(-1, 1))

    # group by target and solve PnP
    groups = {}
    for c, mid in zip(corners, ids):
        if int(mid) not in obj_points:
            continue
        ti, op = obj_points[int(mid)]
        groups.setdefault(ti, ([], []))
        groups[ti][0].append(op)
        groups[ti][1].append(c.reshape(4, 2))

    # mirror the perception-node quality gates
    MIN_MARKERS = 2
    MAX_REPROJ = 2.0

    print(f"{'tgt':>3} {'nmk':>3} {'pos err[mm]':>12} {'yaw err[deg]':>12} {'reproj[px]':>10}  gate")
    worst = 0.0
    n_pass = 0
    for ti in sorted(groups):
        op = np.concatenate(groups[ti][0]).astype(np.float64)
        ip = np.concatenate(groups[ti][1]).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(op, ip, K, None,
                                      flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            print(f"{ti:>3} PnP failed")
            continue
        rvec, tvec = cv2.solvePnPRefineLM(op, ip, K, None, rvec, tvec)
        proj, _ = cv2.projectPoints(op, rvec, tvec, K, None)
        reproj = float(np.linalg.norm(proj.reshape(-1, 2) - ip, axis=1).mean())

        R_ct, _ = cv2.Rodrigues(rvec)  # cv cam <- target
        # estimated target pose in world
        tw_R = cv_R @ R_ct
        tw_p = cv_R @ tvec.ravel() + cw_p

        t = board["targets"][ti]["world"]
        gt_p = np.array([t["x"], t["y"], t["z"]])
        gt_yaw = t["yaw"]
        est_yaw = math.atan2(tw_R[1, 0], tw_R[0, 0])
        yaw_err = math.degrees((est_yaw - gt_yaw + math.pi) % (2 * math.pi) - math.pi)
        pos_err = float(np.linalg.norm(tw_p - gt_p) * 1000.0)
        passed = len(groups[ti][0]) >= MIN_MARKERS and reproj <= MAX_REPROJ
        if passed:
            worst = max(worst, pos_err)
            n_pass += 1
        print(f"{ti:>3} {len(groups[ti][0]):>3} {pos_err:>12.2f} {yaw_err:>12.3f} "
              f"{reproj:>10.3f}  {'PASS' if passed else 'rejected'}")
        cv2.drawFrameAxes(vis, K, None, rvec, tvec, 0.05)

    if args.save:
        cv2.imwrite(args.save, vis)
        print(f"saved annotated view to {args.save}")
    if not groups:
        print("NO targets detected - check texture orientation / camera view")
        sys.exit(1)
    if n_pass == 0:
        print("no target passed the perception gates")
        sys.exit(1)
    print(f"{n_pass} target(s) pass the gates; "
          f"worst gated position error: {worst:.2f} mm")


if __name__ == "__main__":
    main()
