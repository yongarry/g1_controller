#!/usr/bin/env python3
# Copyright (c) 2025, DYROS.
#
# ArUco footstep-target perception node for the G1 footstep controller.
#
# Estimates the pose of ArUco-tagged footstep targets (4 markers per target,
# see cmd/gen_aruco_footstep_scene.py) with the head D435i and publishes them
# in the PELVIS frame over DDS as a JSON payload (std_msgs String on
# `rt/footstep_vision` by default). The C++ controller (command_source:
# "vision" in deploy.yaml) converts them to the stance-foot frame, keeps a
# short world-frame memory, picks the two nearest feasible targets, and feeds
# the footstep planner.
#
# Image sources (--source / deploy.yaml footstep.vision.camera.source):
#   sim       - subscribes rt/lowstate (+ rt/odommodestate) from unitree_mujoco,
#               mirrors the robot state into a local MuJoCo copy of the scene,
#               and renders the "d435i" camera offscreen. Ground-truth-free:
#               the detection pipeline is identical to the real one.
#   realsense - Intel RealSense D435i color stream via pyrealsense2
#               (factory intrinsics).
#
# Frames:
#   T_pelvis_target = T_pelvis_cam(q) * R_muj_cv * T_cvcam_target
#   - T_pelvis_cam from an analytic FK over the 3 waist joints (rt/lowstate)
#     up to the HEAD-mounted D435i (numpy only - the mujoco package is NOT
#     required on the real robot; it is imported lazily for the sim renderer).
#     An optional hand-eye correction (vision.camera.extrinsic, head_link
#     frame) overrides the default URDF mount.
#   - PnP is solved per target on all detected marker corners (up to 16 pts).
#
# Usage:
#   python3 cmd/aruco_footstep_perception.py                     # sim, iface lo
#   python3 cmd/aruco_footstep_perception.py --network enp3s0 --source realsense
#   python3 cmd/aruco_footstep_perception.py --show              # debug window

import argparse
import json
import math
import os
import sys
import time

import numpy as np

import cv2
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import aruco_common as ac

from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                         ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HGLowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJ_DIR = os.path.dirname(_THIS_DIR)
_WS_DIR = os.path.dirname(_PROJ_DIR)
DEFAULT_DEPLOY = os.path.join(_PROJ_DIR, "config", "policy", "footstep", "v0",
                              "params", "deploy.yaml")
DEFAULT_BOARD = os.path.join(_PROJ_DIR, "config", "aruco_board.json")
SCENE_XML = os.path.join(_WS_DIR, "unitree_mujoco", "unitree_robots", "g1",
                         "scene_29dof_footstep.xml")  # sim source only

# OpenCV optical frame expressed in the MuJoCo camera frame
# (x right in both; cv y down = -muj y; cv z forward = -muj z)
R_MUJ_CV = np.diag([1.0, -1.0, -1.0])

NUM_JOINTS = 29


def rpy_to_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @
            np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
            np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def mat_to_quat_wxyz(R):
    q = np.empty(4)
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q[:] = [0.25 * s, (R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    return q / np.linalg.norm(q)


# ---------------------------------------------------------------------------
# Robot state (DDS)
# ---------------------------------------------------------------------------
class RobotState:
    """Latest joint positions + IMU quat from rt/lowstate (unitree_hg),
    plus base world position from rt/odommodestate (sim only)."""

    def __init__(self, odom_topic="rt/odommodestate", want_odom=True):
        self.q = np.zeros(NUM_JOINTS)
        self.quat = np.array([1.0, 0, 0, 0])  # w x y z
        self.base_pos = np.zeros(3)
        self.t_low = 0.0
        self.t_odom = 0.0

        self.low_sub = ChannelSubscriber("rt/lowstate", HGLowState_)
        self.low_sub.Init(self._on_low, 1)
        if want_odom:
            self.odom_sub = ChannelSubscriber(odom_topic, SportModeState_)
            self.odom_sub.Init(self._on_odom, 1)

    def _on_low(self, msg):
        for i in range(NUM_JOINTS):
            self.q[i] = msg.motor_state[i].q
        self.quat = np.array(msg.imu_state.quaternion)  # w x y z
        self.t_low = time.time()

    def _on_odom(self, msg):
        self.base_pos = np.array(msg.position)
        self.t_odom = time.time()

    def wait(self, need_odom, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.t_low > 0 and (not need_odom or self.t_odom > 0):
                return True
            time.sleep(0.1)
        return False


# ---------------------------------------------------------------------------
# Camera-in-pelvis FK (analytic, numpy only - no mujoco needed on the robot)
#
# Only the 3 waist joints (SDK 12: yaw, 13: roll, 14: pitch) sit between the
# pelvis and the HEAD-mounted D435i; everything else is a fixed transform.
# Chain constants from g1_29dof.xml / the official URDF:
#   pelvis -Rz(q12)-> (-0.0039635, 0, 0.035) -Rx(q13)->
#   (0, 0, 0.019) -Ry(q14)-> head_link (0.0039635, 0, -0.054) ->
#   d435 (0.05366, 0.01753, 0.47387) pitched down 0.8307767 rad.
# Validated against MuJoCo FK to machine precision over the full waist range.
# ---------------------------------------------------------------------------
# camera_link (x fwd, z up) -> MuJoCo cam (-z fwd, y up):
# columns = mujoco-cam axes expressed in camera_link coords
R_LINK_MUJCAM = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0.]]).T


class PelvisCamFK:
    P_ROLL = np.array([-0.0039635, 0.0, 0.035])
    P_TORSO = np.array([0.0, 0.0, 0.019])
    P_HEAD = np.array([0.0039635, 0.0, -0.054])
    D435_POS = np.array([0.05366, 0.01753, 0.47387])   # in head_link frame
    D435_PITCH = 0.8307767239493009                    # [rad] down

    def __init__(self, extrinsic=None):
        # Optional hand-eye override: D435i camera_link pose in the HEAD_LINK
        # frame ({pos: [x,y,z], rpy: [r,p,y]}, RealSense camera_link convention:
        # x forward, z up). Without it, the official URDF mount is used.
        if extrinsic:
            self.ext_pos = np.array(extrinsic.get("pos", [0, 0, 0]), dtype=float)
            R_link = rpy_to_mat(*[float(v) for v in
                                  extrinsic.get("rpy", [0, 0, 0])])
        else:
            self.ext_pos = self.D435_POS
            R_link = rpy_to_mat(0.0, self.D435_PITCH, 0.0)
        self.ext_R = R_link @ R_LINK_MUJCAM

    def cam_in_pelvis(self, q):
        """(pos, R) of the MuJoCo-convention camera frame in the pelvis frame."""
        R = rpy_to_mat(0.0, 0.0, float(q[12]))                 # waist yaw
        p = R @ self.P_ROLL
        R = R @ rpy_to_mat(float(q[13]), 0.0, 0.0)             # waist roll
        p = p + R @ self.P_TORSO
        R = R @ rpy_to_mat(0.0, float(q[14]), 0.0)             # waist pitch
        p_head = p + R @ self.P_HEAD                           # head_link
        return p_head + R @ self.ext_pos, R @ self.ext_R


# ---------------------------------------------------------------------------
# Image sources
# ---------------------------------------------------------------------------
class SimCamera:
    """Offscreen render of the d435i camera from the mirrored sim state.

    The mujoco package is only needed here (sim source) - the real-robot
    path (realsense) must run without it.
    """

    def __init__(self, scene_xml, state: RobotState, width, height):
        os.environ.setdefault("MUJOCO_GL", "egl")
        global mujoco
        import mujoco
        self.state = state
        self.model = mujoco.MjModel.from_xml_path(scene_xml)
        self.data = mujoco.MjData(self.model)
        self.cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA,
                                        "d435i")
        assert self.cam_id >= 0, "d435i camera missing from scene XML"
        # enlarge the offscreen framebuffer if needed (model default is 640x480)
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, width)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, height)
        self.renderer = mujoco.Renderer(self.model, height, width)
        fovy = math.radians(self.model.cam_fovy[self.cam_id])
        f = (height / 2.0) / math.tan(fovy / 2.0)
        self.K = np.array([[f, 0, width / 2.0],
                           [0, f, height / 2.0], [0, 0, 1.0]])
        self.dist = None

    def read(self):
        s = self.state
        q = s.q.copy()
        self.data.qpos[0:3] = s.base_pos
        self.data.qpos[3:7] = s.quat
        self.data.qpos[7:7 + NUM_JOINTS] = q
        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera="d435i")
        rgb = self.renderer.render()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), q


class RealsenseCamera:
    """D435i color stream. Works on USB 3 and USB 2.x: librealsense only
    reports the profiles the current connection can actually stream, so the
    requested (width, height, fps) is matched against that list and falls
    back automatically (e.g. on USB 2.1 the color camera cannot do
    1280x720@30 - typically 1280x720@6 or 640x480@30 are available).
    Intrinsics always come from the profile that actually started."""

    MIN_FPS = 6  # planner samples per step (~1 s), so >=6 Hz is sufficient

    def __init__(self, state: RobotState, width, height, fps):
        import pyrealsense2 as rs
        self.state = state

        ctx = rs.context()
        devs = ctx.query_devices()
        if len(devs) == 0:
            raise RuntimeError("no RealSense device connected")
        dev = devs[0]
        try:
            usb = dev.get_info(rs.camera_info.usb_type_descriptor)
        except Exception:
            usb = "unknown"

        # color profiles the CURRENT connection (USB2/USB3) can stream
        color = next(s for s in dev.query_sensors()
                     if s.get_info(rs.camera_info.name) == "RGB Camera")
        modes = sorted(
            {(p.as_video_stream_profile().width(),
              p.as_video_stream_profile().height(), p.fps())
             for p in color.get_stream_profiles()
             if p.stream_type() == rs.stream.color
             and p.format() == rs.format.bgr8},
            key=lambda m: (m[0] * m[1], m[2]), reverse=True)

        w, h, f = self._pick_mode(modes, width, height, fps)
        if (w, h, f) != (width, height, fps):
            print(f"[perception] RealSense on USB {usb}: requested "
                  f"{width}x{height}@{fps} unavailable, using {w}x{h}@{f}. "
                  f"(Lower resolution shortens the marker detection range - "
                  f"set vision.camera width/height to pick a mode explicitly.)")
        else:
            print(f"[perception] RealSense on USB {usb}: {w}x{h}@{f}")

        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, f)
        prof = self.pipe.start(cfg)
        intr = prof.get_stream(rs.stream.color) \
                   .as_video_stream_profile().get_intrinsics()
        self.K = np.array([[intr.fx, 0, intr.ppx],
                           [0, intr.fy, intr.ppy], [0, 0, 1.0]])
        self.dist = np.array(intr.coeffs, dtype=np.float64)
        if not np.any(self.dist):
            self.dist = None

    @classmethod
    def _pick_mode(cls, modes, width, height, fps):
        """Requested mode if streamable; else the same resolution at the
        highest available fps; else (among modes with fps >= MIN_FPS, falling
        back to all modes) the smallest resolution that still covers the
        requested pixel area - preserving detection range without wasting
        CPU - or the largest one available if none covers it."""
        if not modes:
            raise RuntimeError("RealSense reports no BGR8 color profiles")
        if (width, height, fps) in modes:
            return width, height, fps
        same_res = [m for m in modes if m[0] == width and m[1] == height]
        if same_res:
            return max(same_res, key=lambda m: m[2])
        pool = [m for m in modes if m[2] >= cls.MIN_FPS] or list(modes)
        req_area = width * height
        covering = [m for m in pool if m[0] * m[1] >= req_area]
        if covering:
            return min(covering, key=lambda m: (m[0] * m[1], -m[2]))
        return max(pool, key=lambda m: (m[0] * m[1], m[2]))

    def read(self):
        frames = self.pipe.wait_for_frames()
        img = np.asanyarray(frames.get_color_frame().get_data())
        return img, self.state.q.copy()


# ---------------------------------------------------------------------------
# Detector + PnP
# ---------------------------------------------------------------------------
class TargetEstimator:
    def __init__(self, board, min_markers=2):
        d = board["dictionary"]
        self.dictionary = ac.make_dictionary(d["bits"], d["size"], d["seed"])
        self.obj_points = ac.board_object_points(board)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary,
                                                ac.detector_parameters())
        self.min_markers = int(min_markers)

    def estimate(self, gray, K, dist, vis=None):
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return []
        ids = ids.ravel()
        if vis is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids.reshape(-1, 1))

        groups = {}
        for c, mid in zip(corners, ids):
            hit = self.obj_points.get(int(mid))
            if hit is None:
                continue
            ti, op = hit
            groups.setdefault(ti, ([], []))
            groups[ti][0].append(op)
            groups[ti][1].append(c.reshape(4, 2))

        out = []
        for ti, (ops, ips) in groups.items():
            if len(ops) < self.min_markers:
                continue
            op = np.concatenate(ops).astype(np.float64)
            ip = np.concatenate(ips).astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(op, ip, K, dist,
                                          flags=cv2.SOLVEPNP_IPPE)
            if not ok:
                continue
            rvec, tvec = cv2.solvePnPRefineLM(op, ip, K, dist, rvec, tvec)
            proj, _ = cv2.projectPoints(op, rvec, tvec, K, dist)
            err = float(np.linalg.norm(proj.reshape(-1, 2) - ip, axis=1).mean())
            R_ct, _ = cv2.Rodrigues(rvec)
            out.append({"index": ti, "R": R_ct, "t": tvec.ravel(),
                        "nmk": len(ops), "err": err})
            if vis is not None:
                cv2.drawFrameAxes(vis, K, dist, rvec, tvec, 0.05)
        return out


# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="ArUco footstep target perception")
    p.add_argument("--deploy", default=DEFAULT_DEPLOY,
                   help="deploy.yaml (footstep.vision section)")
    p.add_argument("--network", default="lo", help="DDS network interface")
    p.add_argument("--domain", type=int, default=0, help="DDS domain id")
    p.add_argument("--source", default=None, choices=[None, "sim", "realsense"])
    p.add_argument("--show", action="store_true", help="debug window")
    p.add_argument("--rate", type=float, default=15.0, help="publish rate [Hz]")
    args = p.parse_args()

    with open(args.deploy) as f:
        deploy = yaml.safe_load(f)
    vis_cfg = (deploy.get("footstep", {}) or {}).get("vision", {}) or {}
    cam_cfg = vis_cfg.get("camera", {}) or {}

    topic = vis_cfg.get("topic", "rt/footstep_vision")
    board_path = vis_cfg.get("board", DEFAULT_BOARD)
    if not os.path.isabs(board_path):
        board_path = os.path.join(_PROJ_DIR, board_path)
    source = args.source or cam_cfg.get("source", "sim")
    width = int(cam_cfg.get("width", 1280))
    height = int(cam_cfg.get("height", 720))
    fps = int(cam_cfg.get("fps", 30))
    min_markers = int(vis_cfg.get("min_markers", 2))
    max_reproj = float(vis_cfg.get("max_reproj_px", 2.0))

    board = ac.load_board(board_path)
    print(f"[perception] board: {board_path} "
          f"({len(board['targets'])} targets), source={source}, topic={topic}")

    ChannelFactoryInitialize(args.domain, args.network)
    state = RobotState(want_odom=(source == "sim"))
    print("[perception] waiting for rt/lowstate"
          + (" + rt/odommodestate ..." if source == "sim" else " ..."))
    if not state.wait(need_odom=(source == "sim")):
        print("[perception] ERROR: no robot state received", file=sys.stderr)
        sys.exit(1)

    if source == "sim":
        cam = SimCamera(SCENE_XML, state, width, height)
    else:
        cam = RealsenseCamera(state, width, height, fps)

    fk = PelvisCamFK(extrinsic=cam_cfg.get("extrinsic"))
    est = TargetEstimator(board, min_markers=min_markers)
    pub = ChannelPublisher(topic, String_)
    pub.Init()

    period = 1.0 / max(args.rate, 1e-3)
    n_pub = 0
    t_last_log = time.time()
    while True:
        t0 = time.time()
        img, q = cam.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        vis = img if args.show else None

        cam_p, cam_R = fk.cam_in_pelvis(q)   # MuJoCo cam frame in pelvis
        R_pc = cam_R @ R_MUJ_CV              # cv optical frame in pelvis

        targets = []
        for e in est.estimate(gray, cam.K, cam.dist, vis):
            if e["err"] > max_reproj:
                continue
            tp = R_pc @ e["t"] + cam_p       # target origin in pelvis
            tR = R_pc @ e["R"]               # target axes in pelvis
            qt = mat_to_quat_wxyz(tR)
            targets.append({
                "id": int(e["index"]),
                "pos": [round(float(v), 5) for v in tp],
                "quat": [round(float(v), 6) for v in qt],
                "nmk": int(e["nmk"]),
                "err": round(e["err"], 3),
            })

        msg = json.dumps({"stamp": time.time(), "targets": targets},
                         separators=(",", ":"))
        pub.Write(String_(data=msg))
        n_pub += 1

        if args.show:
            cv2.imshow("aruco_footstep_perception", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        if time.time() - t_last_log > 2.0:
            ids = [t["id"] for t in targets]
            print(f"[perception] {n_pub / (time.time() - t_last_log):5.1f} Hz, "
                  f"targets in view: {ids}")
            n_pub = 0
            t_last_log = time.time()

        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)


if __name__ == "__main__":
    main()
