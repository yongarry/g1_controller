#!/usr/bin/env python3
# Copyright (c) 2025, DYROS.
#
# ArUco footstep-target perception node for the G1 footstep controller.
#
# Estimates the pose of ArUco-tagged footstep targets (4 markers per target,
# see cmd/gen_aruco_footstep_scene.py) with the head D435i and publishes the
# RAW CAMERA-FRAME poses over DDS as a JSON payload (std_msgs String on
# `rt/footstep_vision` by default). This node does NO robot kinematics: the
# C++ controller (command_source: "vision" in deploy.yaml) transforms the
# targets camera -> pelvis (waist FK, D435PelvisCamTransform) -> stance-foot
# frame, keeps a short world-frame memory, picks the two nearest feasible
# targets, and feeds the footstep planner.
#
# Image sources (--source / deploy.yaml footstep.vision.camera.source):
#   sim       - subscribes rt/lowstate + rt/odommodestate from unitree_mujoco,
#               mirrors the robot state into a local MuJoCo copy of the scene,
#               and renders the "d435i" camera offscreen. Ground-truth-free:
#               the detection pipeline is identical to the real one.
#   realsense - Intel RealSense D435i color stream via pyrealsense2 (factory
#               intrinsics). Needs NO mujoco package: on the robot PC the
#               dependencies are only numpy, opencv (contrib), pyrealsense2,
#               pyyaml and unitree_sdk2py. rt/lowstate is subscribed to stamp
#               each frame with its capture-time waist angles (see below); if
#               it never arrives the node keeps publishing without them.
#
# Frames: published poses are T_cam_target in the D435i COLOR OPTICAL frame
# (OpenCV convention: x right, y down, z forward - the direct solvePnP
# output). PnP is solved per target on all detected marker corners (<=16 pts).
#
# Each frame also carries the waist joints (SDK 12 yaw, 13 roll, 14 pitch)
# sampled when the image was captured, so the controller can run the
# camera->pelvis FK with the pose the head actually had at exposure time
# instead of the one it has tens of ms later, when the frame is consumed.
#
# Exposure: auto-exposure settles around 16 ms indoors, which smears the
# marker corners while the robot walks and kills the decoder long before a
# color blob would degrade. The color exposure/gain are therefore fixed from
# deploy.yaml (vision.camera.exposure_us / gain); set exposure_us: 0 to go
# back to auto.
#
# Usage:
#   python3 cmd/aruco_footstep_perception.py                     # sim, iface lo
#   python3 cmd/aruco_footstep_perception.py --network enp3s0 --source realsense
#   python3 cmd/aruco_footstep_perception.py --show              # debug window
#   python3 cmd/aruco_footstep_perception.py --exposure-us 4000  # field tuning

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
DEFAULT_DEPLOY = os.path.join(_PROJ_DIR, "config", "policy", "footstep", "deploy_base.yaml")
DEFAULT_BOARD = os.path.join(_PROJ_DIR, "config", "aruco_board.json")
SCENE_XML = os.path.join(_WS_DIR, "unitree_mujoco", "unitree_robots", "g1",
                         "scene_29dof_footstep.xml")  # sim source only

NUM_JOINTS = 29
WAIST_IDS = (12, 13, 14)  # SDK indices: waist yaw, roll, pitch


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

    def waist(self):
        """[yaw, roll, pitch] of the waist chain, or None if lowstate is
        stale/absent (the controller then falls back to its own tick)."""
        if self.t_low <= 0.0 or time.time() - self.t_low > 0.5:
            return None
        return [float(self.q[i]) for i in WAIST_IDS]

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
        # Default MjvOption only draws geom groups 0-2. Stepping stones are
        # group 3 and ArUco markers are group 4, so both would be missing
        # from the D435i image (the simulate UI groups do not apply here).
        self.scene_option = mujoco.MjvOption()
        for i in range(len(self.scene_option.geomgroup)-1):
            self.scene_option.geomgroup[i] = 1
        fovy = math.radians(self.model.cam_fovy[self.cam_id])
        f = (height / 2.0) / math.tan(fovy / 2.0)
        self.K = np.array([[f, 0, width / 2.0],
                           [0, f, height / 2.0], [0, 0, 1.0]])
        self.dist = None

    def read(self):
        """(bgr image, capture-time waist joints)."""
        s = self.state
        q = s.q.copy()  # the pose this frame is rendered from
        self.data.qpos[0:3] = s.base_pos
        self.data.qpos[3:7] = s.quat
        self.data.qpos[7:7 + NUM_JOINTS] = q
        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera="d435i",
                                   scene_option=self.scene_option)
        rgb = self.renderer.render()
        waist = [float(q[i]) for i in WAIST_IDS]
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), waist


class RealsenseCamera:
    """D435i color stream. Works on USB 3 and USB 2.x: librealsense only
    reports the profiles the current connection can actually stream, so the
    requested (width, height, fps) is matched against that list and falls
    back automatically (e.g. on USB 2.1 the color camera cannot do
    1280x720@30 - typically 1280x720@6 or 640x480@30 are available).
    Intrinsics always come from the profile that actually started."""

    MIN_FPS = 6  # planner samples per step (~1 s), so >=6 Hz is sufficient
    # The RGB sensor reports exposure in UVC "absolute exposure time" units of
    # 100 us (unlike the depth sensor, which uses us), so 6 ms -> option 60.
    COLOR_EXPOSURE_UNIT_US = 100.0

    def __init__(self, width, height, fps, state=None, exposure_us=0, gain=None):
        import pyrealsense2 as rs

        self.state = state

        usb, modes = self._probe(rs)
        w, h, f = self._pick_mode(modes, width, height, fps)
        if (w, h, f) != (width, height, fps):
            print(f"[perception] RealSense on USB {usb}: requested "
                  f"{width}x{height}@{fps} unavailable, using {w}x{h}@{f}. "
                  f"(Lower resolution shortens the marker detection range - "
                  f"set vision.camera width/height to pick a mode explicitly.)")
        else:
            print(f"[perception] RealSense on USB {usb}: {w}x{h}@{f}")

        # Start color stream. On EBUSY (another process holding V4L2),
        # hardware-reset the device, wait for USB re-enumeration, then retry.
        last_err = None
        for attempt in range(3):
            self.pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, f)
            try:
                prof = self.pipe.start(cfg)
                break
            except RuntimeError as e:
                last_err = e
                msg = str(e).lower()
                busy = ("busy" in msg) or ("errno=16" in msg)
                try:
                    self.pipe.stop()
                except Exception:
                    pass
                if not busy or attempt == 2:
                    raise
                print(f"[perception] RealSense busy, hardware_reset "
                      f"(attempt {attempt + 1}/3) ...")
                self._hardware_reset(rs)
                usb, modes = self._probe(rs)
                w, h, f = self._pick_mode(modes, width, height, fps)
        else:
            raise last_err

        # Drop stale buffered frames: if detectMarkers is slower than the
        # stream for a stretch, wait_for_frames would otherwise hand us a
        # seconds-old image and the waist stamp would not match the scene.
        try:
            color_sensor = prof.get_device().first_color_sensor()
            if color_sensor.supports(rs.option.frames_queue_size):
                color_sensor.set_option(rs.option.frames_queue_size, 1)
        except Exception as e:
            print(f"[perception] could not set frames_queue_size=1 ({e})")

        self._configure_exposure(rs, prof, exposure_us, gain, f)
        self._stream_fps = float(f)

        intr = prof.get_stream(rs.stream.color) \
                   .as_video_stream_profile().get_intrinsics()
        self.K = np.array([[intr.fx, 0, intr.ppx],
                           [0, intr.fy, intr.ppy], [0, 0, 1.0]])
        self.dist = np.array(intr.coeffs, dtype=np.float64)
        if not np.any(self.dist):
            self.dist = None

    @classmethod
    def _configure_exposure(cls, rs, prof, exposure_us, gain, fps):
        """Pin the color exposure (and gain) so walking does not blur the
        markers away.

        Auto-exposure targets image brightness, not sharpness: indoors it
        settles around 16 ms, and with the head bobbing a 3 cm marker smears
        over several pixels - the corner refinement then fails, and a failed
        decode is a target the planner never sees. A short fixed exposure with
        the gain raised to compensate keeps the corners crisp at the cost of
        noise, which the decoder tolerates far better.

        exposure_us <= 0 restores auto-exposure.
        """
        try:
            sensor = prof.get_device().first_color_sensor()
        except Exception as e:
            print(f"[perception] cannot access color sensor, leaving exposure "
                  f"on auto ({e})")
            return

        def set_opt(opt, value, name):
            if not sensor.supports(opt):
                print(f"[perception] color sensor has no {name} option")
                return None
            try:
                r = sensor.get_option_range(opt)
                v = float(min(max(value, r.min), r.max))
                sensor.set_option(opt, v)
                return v
            except Exception as e:
                print(f"[perception] failed to set {name}={value}: {e}")
                return None

        if exposure_us is None or exposure_us <= 0:
            set_opt(rs.option.enable_auto_exposure, 1, "auto-exposure")
            print("[perception] color auto-exposure ON (markers will blur "
                  "while walking - set vision.camera.exposure_us to fix it)")
            return

        # A frame cannot expose longer than its period; warn rather than let
        # librealsense silently drop the framerate.
        max_us = 1e6 / max(fps, 1)
        if exposure_us > max_us:
            print(f"[perception] exposure {exposure_us:.0f} us exceeds the "
                  f"{fps} fps frame period ({max_us:.0f} us); the driver will "
                  f"clamp it or drop frames")

        if set_opt(rs.option.enable_auto_exposure, 0, "auto-exposure") is None:
            return
        v = set_opt(rs.option.exposure,
                    exposure_us / cls.COLOR_EXPOSURE_UNIT_US, "exposure")
        applied = "unchanged" if v is None else \
            f"{v * cls.COLOR_EXPOSURE_UNIT_US:.0f} us"
        msg = f"[perception] color exposure fixed at {applied}"

        if gain is not None:
            if sensor.supports(rs.option.enable_auto_white_balance):
                # WB keeps hunting on a fixed exposure and only shifts hue -
                # the detector works on gray, so freeze it for stable frames.
                set_opt(rs.option.enable_auto_white_balance, 0, "auto-WB")
            g = set_opt(rs.option.gain, gain, "gain")
            if g is not None:
                msg += f", gain {g:.0f}"
        print(msg)

    @staticmethod
    def _probe(rs):
        """Return (usb_descriptor, color BGR8 modes) for the first device."""
        ctx = rs.context()
        devs = ctx.query_devices()
        if len(devs) == 0:
            raise RuntimeError("no RealSense device connected")
        dev = devs[0]
        try:
            usb = dev.get_info(rs.camera_info.usb_type_descriptor)
        except Exception:
            usb = "unknown"
        color = next(s for s in dev.query_sensors()
                     if s.get_info(rs.camera_info.name) == "RGB Camera")
        modes = sorted(
            {(p.as_video_stream_profile().width(),
              p.as_video_stream_profile().height(), p.fps())
             for p in color.get_stream_profiles()
             if p.stream_type() == rs.stream.color
             and p.format() == rs.format.bgr8},
            key=lambda m: (m[0] * m[1], m[2]), reverse=True)
        return usb, modes

    @staticmethod
    def _hardware_reset(rs, timeout=15.0):
        """Reset the first RealSense and block until it reappears on USB."""
        ctx = rs.context()
        devs = ctx.query_devices()
        if len(devs) == 0:
            raise RuntimeError("no RealSense device connected (reset)")
        serial = devs[0].get_info(rs.camera_info.serial_number)
        print(f"[perception] resetting RealSense sn={serial} ...")
        devs[0].hardware_reset()
        t0 = time.time()
        time.sleep(2.0)  # disconnect takes a moment
        while time.time() - t0 < timeout:
            ctx = rs.context()
            for d in ctx.query_devices():
                try:
                    if d.get_info(rs.camera_info.serial_number) == serial:
                        time.sleep(1.0)  # settle before streaming
                        return
                except Exception:
                    pass
            time.sleep(0.5)
        raise RuntimeError(
            f"RealSense sn={serial} did not reappear within {timeout:.0f}s")

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

    @property
    def stream_fps(self):
        return self._stream_fps

    def read(self):
        """(bgr image, capture-time waist joints)."""
        frames = self.pipe.wait_for_frames()
        # Sampled here, right as the frame is delivered: this is the closest
        # we can get to the exposure instant without a hardware trigger, and
        # it is what makes the controller's camera->pelvis FK consistent with
        # the pose the head had when the markers were seen.
        waist = self.state.waist() if self.state is not None else None
        color = frames.get_color_frame()
        if not color:
            return None, waist
        img = np.asanyarray(color.get_data())
        return img, waist


# ---------------------------------------------------------------------------
# Detector + PnP
# ---------------------------------------------------------------------------
class TargetEstimator:
    def __init__(self, board, min_markers=2):
        d = board["dictionary"]
        self.dictionary = ac.make_dictionary(d["bits"], d["size"], d["seed"])
        self.obj_points = ac.board_object_points(board)
        # Always detect at full resolution. 3 cm markers at 1280x720 are
        # already ~17 px (~3 px/module); a half-res pass drops them below
        # the decoder's limit and yields zero targets.
        self.detector = cv2.aruco.ArucoDetector(self.dictionary,
                                                ac.detector_parameters())
        self.min_markers = int(min_markers)

    def _detect(self, gray):
        """Detect marker ids/corners in full-image coordinates."""
        return self.detector.detectMarkers(gray)[:2]

    def estimate(self, gray, K, dist, vis=None):
        corners, ids = self._detect(gray)
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
    p.add_argument("--rate", type=float, default=None,
                   help="max publish rate [Hz]; default = vision.camera.fps "
                        "from deploy.yaml; 0 = uncapped")
    p.add_argument("--exposure-us", type=float, default=None,
                   help="fixed color exposure [us]; 0 = auto "
                        "(overrides vision.camera.exposure_us)")
    p.add_argument("--gain", type=float, default=None,
                   help="color gain to go with the fixed exposure "
                        "(overrides vision.camera.gain)")
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
    exposure_us = args.exposure_us if args.exposure_us is not None \
        else float(cam_cfg.get("exposure_us", 0))
    gain = args.gain if args.gain is not None else cam_cfg.get("gain")
    gain = None if gain is None else float(gain)

    board = ac.load_board(board_path)
    print(f"[perception] board: {board_path} "
          f"({len(board['targets'])} targets), source={source}, topic={topic}")

    ChannelFactoryInitialize(args.domain, args.network)
    if source == "sim":
        # rt/lowstate + rt/odommodestate mirror the sim pose for rendering
        # (and supply the per-frame waist angles).
        state = RobotState(want_odom=True)
        print("[perception] waiting for rt/lowstate + rt/odommodestate ...")
        if not state.wait(need_odom=True):
            print("[perception] ERROR: no robot state received", file=sys.stderr)
            sys.exit(1)
        cam = SimCamera(SCENE_XML, state, width, height)
    else:
        # rt/lowstate is only needed to stamp each frame with the waist
        # angles it was captured at. Not fatal if absent: the controller then
        # falls back to the joint state of the tick that consumes the frame.
        state = RobotState(want_odom=False)
        print("[perception] waiting for rt/lowstate (capture-time waist "
              "angles) ...")
        if not state.wait(need_odom=False, timeout=3.0):
            # Keep the subscriber alive anyway: waist() reports None while the
            # state is stale and starts stamping frames by itself once the
            # controller comes up.
            print("[perception] WARNING: no rt/lowstate yet - publishing "
                  "without capture-time waist angles; the controller falls "
                  "back to the consuming tick's joints (adds head-motion "
                  "error). Will pick them up if lowstate appears.")
        cam = RealsenseCamera(width, height, fps, state=state,
                              exposure_us=exposure_us, gain=gain)

    est = TargetEstimator(board, min_markers=min_markers)
    pub = ChannelPublisher(topic, String_)
    pub.Init()

    # Default: vision.camera.fps (sim has no hardware clock, so this is the
    # only pace). --rate 0 disables the sleep; RealSense wait_for_frames still
    # blocks at the stream rate.
    if args.rate is None:
        rate = float(fps)
    else:
        rate = float(args.rate)
    period = (1.0 / rate) if rate > 0.0 else 0.0
    if period > 0.0:
        src = "--rate" if args.rate is not None else "vision.camera.fps"
        print(f"[perception] publish capped at {rate:.1f} Hz ({src})")
    else:
        print("[perception] publish uncapped")

    n_pub = 0
    t_last_log = time.time()
    n_no_waist = 0
    t_detect_ms = 0.0
    gray = None
    while True:
        t0 = time.time()
        img, waist = cam.read()
        if img is None:
            continue
        t_cap = time.time()
        # Reuse the gray buffer when the stream size is stable.
        if gray is None or gray.shape[:2] != img.shape[:2]:
            gray = np.empty(img.shape[:2], dtype=np.uint8)
        cv2.cvtColor(img, cv2.COLOR_BGR2GRAY, dst=gray)
        vis = img if args.show else None

        # raw camera-optical-frame poses (solvePnP output); the controller
        # does the camera -> pelvis -> stance-foot transforms itself.
        t_det0 = time.time()
        targets = []
        for e in est.estimate(gray, cam.K, cam.dist, vis):
            if e["err"] > max_reproj:
                continue
            qt = mat_to_quat_wxyz(e["R"])
            targets.append({
                "id": int(e["index"]),
                "pos": [round(float(v), 5) for v in e["t"]],
                "quat": [round(float(v), 6) for v in qt],
                "nmk": int(e["nmk"]),
                "err": round(e["err"], 3),
            })
        t_detect_ms += (time.time() - t_det0) * 1000.0

        payload = {"stamp": t_cap, "frame": "camera_optical",
                   "targets": targets}
        if waist is not None:
            # Waist yaw/roll/pitch at capture time; the controller runs the
            # camera->pelvis FK with these instead of its own tick's joints.
            payload["waist"] = [round(v, 6) for v in waist]
        else:
            n_no_waist += 1
        pub.Write(String_(data=json.dumps(payload, separators=(",", ":"))))
        n_pub += 1

        if args.show:
            cv2.imshow("aruco_footstep_perception", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        if time.time() - t_last_log > 2.0:
            note = f", {n_no_waist} frames without waist" if n_no_waist else ""
            avg_ms = t_detect_ms / max(n_pub, 1)
            print(f"[perception] {n_pub / (time.time() - t_last_log):5.1f} Hz, "
                  f"detect {avg_ms:4.1f} ms/frame, "
                  f"{len(targets)} target(s){note}")
            # Published camera-optical poses (what the controller turns into
            # footstep commands). quat = [w, x, y, z].
            # Published camera-optical poses (controller converts these into
            # footstep commands via pelvis FK + stance-frame planning).
            for t in targets:
                print(f"  id={t['id']:2d}  cam_pos=[{t['pos'][0]:7.4f}, "
                      f"{t['pos'][1]:7.4f}, {t['pos'][2]:7.4f}]  "
                      f"nmk={t['nmk']}  err={t['err']:.3f}px")
            n_pub = 0
            n_no_waist = 0
            t_detect_ms = 0.0
            t_last_log = time.time()

        if period > 0.0:
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)


if __name__ == "__main__":
    main()
