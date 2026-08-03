# Custom G1 Deploy Controller

This repository contains a custom deploy controller for the Unitree G1 29-DoF robot.
It is based on the deploy-side C++ controller code from
[unitreerobotics/unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab),
with the training and simulation workspace removed so this repository can focus on
running exported policies on the robot or in sim2sim.

The controller loads IsaacLab-style deploy configurations and ONNX policies, runs
them through ONNX Runtime, and publishes low-level Unitree motor commands through
`unitree_sdk2`.

## Features

- FSM-based controller with `Passive`, `FixStand`, velocity RL, mimic, and footstep policy states.
- Deploys exported RL policies from `params/deploy.yaml` and `exported/policy.onnx`.
- Supports policy directory version discovery, such as `config/policy/velocity/v0`.
- Uses joystick transition expressions defined in `config/config.yaml`.
- Footstep deploy with on-device `OnlineFootCommand` (VRP + ZMP preview + Pinocchio IK).
- Footstep command sources: joystick, CSV replay, world-frame plans, ArUco vision, and
  goal-reaching with per-goal CoM height, arrival heading and upper-body poses.
- Helper scripts under `cmd/` to generate foot-command CSVs and MuJoCo stepping-stone /
  goal-marker scenes.
- Designed for Unitree G1 29-DoF deployment workflows.

## Project Layout

```text
.
├── cmd/
│   ├── gen_cmd.py                    # sample local foot commands -> footcommands.csv
│   ├── convert_footcommand_2_global.py
│   ├── gen_footstep_scene.py         # write stepping stones into MuJoCo scene XML
│   └── gen_goals.py                  # write goal markers into MuJoCo scene XML
├── config/
│   ├── config.yaml                   # FSM states, transitions, and policy paths
│   ├── footcommands.csv              # local per-step commands (generated)
│   ├── footcommands_global.csv       # world-frame targets (generated)
│   ├── urdf/g1_29dof.urdf
│   └── policy/                       # exported deploy policies
├── include/
│   ├── FSM/                          # base FSM state classes
│   └── isaaclab/                     # deploy-side IsaacLab-style runtime helpers
├── src/
│   ├── State_RLBase.cpp              # generic RL policy deploy state
│   ├── State_Mimic.cpp               # motion/mimic policy deploy state
│   └── State_Footstep.cpp            # footstep policy + command generator
├── thirdparty/
│   └── onnxruntime-linux-x64-1.22.0/
├── run_sim.sh                        # launch unitree_mujoco + g1_ctrl together
├── CMakeLists.txt
└── main.cpp
```

Expected sibling repos (for sim2sim):

```text
g1_ws/
├── g1_controller/          # this repo
└── unitree_mujoco/         # MuJoCo simulator (unitree_sdk2 DDS)
```

## Dependencies

Install the system dependencies used by the deploy controller:

```bash
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```

The footstep controller (`State_Footstep`) additionally needs
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) for full-body forward
kinematics, CoM and Jacobians:

```bash
# robotpkg (recommended) — see install.pinocchio.org, then:
sudo apt install -y robotpkg-py3*-pinocchio
# or conda:  conda install pinocchio -c conda-forge
```

Install `unitree_sdk2` system-wide (headers + libs under `/opt/unitree_robotics`):

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF
sudo make install
```

`CMakeLists.txt` uses `find_package(unitree_sdk2)` with
`CMAKE_PREFIX_PATH=/opt/unitree_robotics/lib/cmake`. If you installed the SDK
elsewhere, point that path accordingly before running `cmake`.

ONNX Runtime is expected under:

```text
thirdparty/onnxruntime-linux-x64-1.22.0/
```

## Build

```bash
mkdir -p build
cd build
cmake ..
make -j
```

This builds the `g1_ctrl` executable.

## Run

### Controller only

For local simulation or loopback DDS:

```bash
cd build
LD_LIBRARY_PATH=../thirdparty/onnxruntime-linux-x64-1.22.0/lib:$LD_LIBRARY_PATH ./g1_ctrl --network lo
```

For the real robot, pass the network interface connected to the robot:

```bash
cd build
LD_LIBRARY_PATH=../thirdparty/onnxruntime-linux-x64-1.22.0/lib:$LD_LIBRARY_PATH ./g1_ctrl --network eth0
```

Make sure any other process publishing to the low-level command channel is closed
before running this controller.

### Sim2sim (MuJoCo + controller)

With `unitree_mujoco` built at `../unitree_mujoco/simulate/build`, launch both
processes from the repo root:

```bash
./run_sim.sh            # network = lo
./run_sim.sh enp3s0     # use a different network interface
```

`run_sim.sh` resolves paths relative to itself, so it works regardless of where
`g1_ws` lives on disk. Ctrl+C tears down both the simulator and `g1_ctrl`.

For footstep sim, pick the matching scene in `unitree_mujoco/simulate/config.yaml`
(`scene_29dof_footstep.xml` for stepping stones, `scene_29dof_goals.xml` for goal
markers). `footstep.foot_state_source: sim_odom` gives the planner ground-truth
foot poses; use `fk_odometry` to rehearse what the real robot will actually see —
see [Odometry](#odometry-footstepfoot_state_source).

## Policy Directory Format

Each RL policy directory should contain:

```text
params/deploy.yaml
exported/policy.onnx
```

For example:

```text
config/policy/velocity/v0/
├── params/deploy.yaml
└── exported/policy.onnx
```

In `config/config.yaml`, a policy can point either directly to a deploy directory:

```yaml
policy_dir: config/policy/velocity/v0
```

or to a parent directory:

```yaml
policy_dir: config/policy/velocity
```

When the parent directory does not contain `exported/`, the controller scans its
subdirectories, sorts them, and selects the latest directory that contains an
`exported/` folder.

## FSM Usage

Enabled states and joystick transitions are configured in `config/config.yaml`.
The default flow is:

```text
Passive --[LT + Up]--> FixStand --[RB + X]--> Velocity
```

From the velocity policy, configured transitions can enter mimic states or return
to `Passive`.

Generic RL policies can reuse the `RLBase` state type:

```yaml
MyPolicy:
  id: 4
  type: RLBase
  policy_dir: config/policy/my_policy
  transitions:
    Passive: LT + B.on_pressed
```

If the policy requires custom observations, actions, reset behavior, or transition
checks, add a dedicated state implementation under `src/` and register it with the
FSM system.

## Footstep Policy (G1-2d / 3d commands)

`State_Footstep` deploys the `G12DFootEnvCfg` footstep policy trained in
`isaaclab_dyros`. Unlike the velocity/mimic states, it reproduces the training-time
`OnlineFootCommand` on-device: a foot-step planner feeds a VRP generator + ZMP
preview controller, whose CoM/foot reference is solved with a Pinocchio differential
IK to produce the `joint_ik_target`, `phase`, and `foot_commands_3d` observations
the policy consumes. The policy outputs the 12 lower-body joint targets; the upper
body is held at its defaults.

Layout:

```text
config/policy/footstep/v0/
├── params/deploy.yaml      # joints, gains, obs layout, command/IK/preview params
└── exported/policy.onnx    # exported actor (place your trained policy here)
config/urdf/g1_29dof.urdf   # model used by Pinocchio for FK / IK
```

Generate `policy.onnx` + `deploy.yaml` from a trained checkpoint:

```bash
# in the isaaclab_dyros workspace
./isaaclab.sh -p scripts/rsl_rl/export_footstep_deploy.py \
    --task G1-2d-Play --checkpoint logs/rsl_rl/<exp>/<run>/model_<n>.pt \
    --out_dir <repo>/g1_controller/config/policy/footstep/v0
```

### FSM flow

See `config/config.yaml`:

```text
Passive --[LT+Up]--> FixStand --[RB+Y]--> Footstep
                     FixStand --[RB+X]--> Velocity --[RB+Y]--> Footstep
Footstep --[LT+B]--> Passive ,  Footstep --[RB+X]--> Velocity
```

### Starting and stopping

Entering `Footstep` does **not** start walking. The state holds a *standby*
command — phase frozen at the start of a step, IK target at the default joint
pose, zero foot command — so the robot stands still while already running the
footstep policy. Press **Y** to start:

```text
FixStand --[RB+Y]--> Footstep (standing, standby) --[Y]--> walking
```

The command generator is reset on the Y press rather than on state entry, so the
planner and preview controller anchor on the state the robot is actually in the
moment it starts moving. Re-entering the state returns to standby.

### Joystick mode (`command_source: joystick`)

In `Footstep`, the left stick commands forward step length (`ly`) and lateral
crab-walk (`lx`: left swing widens / right swing narrows via `lateral_bias`); the
right stick (`rx`) commands per-step turning. Nominal step width, support/swing
times and apex height come from `footstep.default_input` in `deploy.yaml`, and all
operator inputs are clamped to the trained `ranges` (including `foot_pos_z` for
per-step height change).

### Command sources (`deploy.yaml` → `footstep.command_source`)

| Mode | Description |
|------|-------------|
| `joystick` | Operator drives local per-step commands each control tick. |
| `csv` | Replay local per-step commands from `footstep.csv_path`. |
| `csv_global` | Follow absolute world-frame targets from `footstep.global_csv_path`; the planner recomputes the local command from the accumulated stance foot to each target every step (drift-corrected). |
| `goal` | Walk to the world-frame goal points under `footstep.goal`, stopping at each one. See [Goal-reaching mode](#goal-reaching-mode-command_source-goal). |
| `vision` | ArUco footstep targets estimated online with the head D435i. See [Vision-based footstep targets](#vision-based-footstep-targets-aruco--d435i). |

For `csv` / `csv_global`, generate the CSV files with the `cmd/` scripts (below).
Set `footstep.global_init_lfoot` / `global_init_rfoot` to the spawn foot poses in
the MuJoCo scene keyframe so the global plan frame matches the simulator. (The
`goal` mode does not use those keys — it anchors on the robot itself.)

### Generating foot commands and MuJoCo scene

End-to-end workflow for scripted footstep tests in sim:

```bash
# 1) sample local commands (x, y, z, yaw) -> config/footcommands.csv
python3 cmd/gen_cmd.py 30 --seed 0
#    also runs convert + scene generation at the end

# or run the steps individually:
python3 cmd/convert_footcommand_2_global.py \
    --input config/footcommands.csv \
    --output config/footcommands_global.csv

python3 cmd/gen_footstep_scene.py
```

**`gen_cmd.py`** — samples `step_x`, `step_y`, `step_z`, `step_yaw` from trained
ranges (defaults match `footstep.ranges` in deploy.yaml). Feet alternate R/L; the
last row is a stop step (`step_x=0`, `step_z=0`, `step_yaw=0`). Override ranges
with flags, e.g. `--z -0.1 0.15`.

Local CSV columns:

```text
foot,step_x,step_y,step_z,step_yaw,ssp_t,dsp_t,height
```

**`convert_footcommand_2_global.py`** — accumulates local steps into world-frame
swing-foot targets. `step_y` is a positive magnitude; sign comes from `foot` (L/R).
`step_z` is a per-step height change accumulated into `pos_z`.

Global CSV columns:

```text
foot,pos_x,pos_y,pos_z,yaw,ssp_t,dsp_t,height
```

**`gen_footstep_scene.py`** — writes stepping stones into
`unitree_mujoco/unitree_robots/g1/scene_29dof_footstep.xml` (idempotent; replaces
only the marked auto-generated region):

- A **ground plane** at the lowest `pos_z` in the global CSV.
- One **variable-height box** per footstep from that plane up to each `pos_z`
  (horizontal size via `--size HX HY`; height is computed automatically).

Then set `command_source: csv_global` in the footstep `deploy.yaml` and run
`./run_sim.sh`.

### Goal-reaching mode (`command_source: goal`)

Walk to a list of world-frame goal points, stopping at each one. The step
generation follows the goal-reaching gait generator from `mind-your-step`, mapped
onto this planner's stance-frame foot command.

At every step boundary the planner measures the active goal from the current
stance foot and emits the nominal step clamped toward it — so far from the goal
the robot takes full steps, and the last step before it is the residual
`goal - current position`. The swing foot is aimed at where it has to *stand for
the robot centre* (the midpoint of the two feet) to land on the goal, half a
stance width to the swing side, rather than at the goal itself: stepping onto the
goal converges with a foot on it and the body half a stance width beside it,
which a reach test measured at the centre would never accept.

```yaml
footstep:
  command_source: goal
  goal:
    points:
      - [1.5, -2.0, 0.0,    -0.08]   # walk here crouched, arriving facing +x
      - [2.5,  1.5, 1.5708,  0.04]   # then here standing taller, facing +y
    step_x_max: 0.2      # [m] nominal (== max) forward step
    step_y: 0.237        # [m] nominal lateral step width
    step_yaw_max: 0.2    # [rad] nominal (== max) per-step turn
    ssp_t: 0.7
    dsp_t: 0.15
    height: 0.08
    reach_radius: 0.05   # [m] reached within this of the robot centre
    reach_yaw: 0.1       # [rad] heading tolerance (points that specify a yaw)
    align_radius: 1.0    # [m] start blending into the goal heading here
    stop:
      move_time: 1.5     # [s] upper-body interpolation
      hold_time: 1.0     # [s] pause after it before walking on
      upper_body_pose:   # one per goal, layout of upper_body.default_joint_pos
        - [0.0, 0.0, 0.0,  -1.2, 0.2, 0.0, 0.4, 0.0, 0.0, 0.0, ...]
        - [0.0, 0.0, 0.0,   0.1, 0.2, 0.0, 1.1, 0.0, 0.0, 0.0, ...]
```

A point is one of:

| Form | Meaning |
|------|---------|
| `[x, y]` | pass through this point, arrival heading free |
| `[x, y, yaw]` | arrive at this point facing `yaw` [rad] |
| `[x, y, yaw, com_z]` | … and walk there with this CoM height offset [m] |

**Heading.** With a `yaw`, the turn steers at the goal *position* while far away
and blends into the goal *heading* between `align_radius` and `reach_radius`, so
the robot finishes by turning on the spot. The goal is only consumed once the
heading is within `reach_yaw` as well as the position within `reach_radius`.

**CoM height.** `com_z` shifts the VRP/CoM height reference (`vrp_height + com_z`)
for the whole leg of the walk leading **to** that point: negative crouches,
positive stands up. Keep it inside the range the policy was trained on —
`(-0.1, 0.05)` for `G13DFootFlatEnvCfg`, which is the config the shipped
`vrp_height: 0.6258` / `pelv_com_offset: 0.0678` deploy params come from. There
is no clamp on this value.

**Stopping at each goal.** On arrival the gait freezes (the same standby command
used before the Y press), the upper body interpolates to that goal's
`stop.upper_body_pose` over `move_time`, and after `hold_time` the robot walks on
**holding that pose**. The last goal ends stopped for good. A goal with no pose
entry still stops, keeping whatever pose is already held.

**Frame.** The goal frame is anchored on the robot itself at the moment walking
starts: the midpoint of its two feet is `x, y = 0, 0` and it faces `yaw = 0`. That
anchor is measured, not configured, so nothing has to be known about a spawn pose
— this mode ignores `global_init_lfoot` / `global_init_rfoot`.

Visualize the points in MuJoCo:

```bash
python3 cmd/gen_goals.py
```

**`gen_goals.py`** — resolves the Footstep policy dir from `config/config.yaml`
the same way the controller does, reads `footstep.goal.points` from that
`deploy.yaml`, and writes
`unitree_mujoco/unitree_robots/g1/scene_29dof_goals.xml`: a `reach_radius` disc,
a marker pole, and an arrival-heading arrow per goal (points without a `yaw` get
no arrow). Seeded from `scene_29dof.xml` on the first run and idempotent
afterwards — re-running replaces only the marked region. All markers are visual
only (`contype`/`conaffinity` = 0), so the robot walks through them. Then point
the simulator at the scene:

```yaml
# unitree_mujoco/simulate/config.yaml
robot_scene: "scene_29dof_goals.xml"
```

### Vision-based footstep targets (ArUco + D435i)

`command_source: vision` closes the loop through the head-mounted D435i instead
of a pre-planned CSV: every footstep target carries **four 3x3-bit ArUco
markers** (up to 16 PnP corner points per target), the perception node
estimates each target's 6-DoF pose, and the controller walks to the two
nearest feasible targets, re-planned at every step boundary in the stance-foot
frame (drift-free by construction).

Pipeline:

```text
gen_aruco_footstep_scene.py ──> scene XML (stones + marker textures)
                            └─> config/aruco_board.json (marker layout)
                            └─> printable marker sheets (real world)

aruco_footstep_perception.py:  image ──ArUco/PnP──> T_cam_target
      (sim: MuJoCo offscreen render of the d435i camera from rt/lowstate+odom;
       real: pyrealsense2 color stream)
      T_pelvis_target = FK(q) * T_cam_target   ──DDS──>  rt/footstep_vision

g1_ctrl (State_Footstep, command_source: vision):
      pelvis frame -> stance-foot frame -> accumulated world-frame memory
      step boundary: pick 2 nearest feasible targets -> foot command buffer
```

Quick start (sim):

```bash
# 1. generate stones + markers + board metadata into the footstep scene
python3 cmd/gen_aruco_footstep_scene.py

# 2. deploy.yaml: footstep.command_source: vision  (see footstep.vision: ...)

# 3. run simulator + controller
./run_sim.sh

# 4. run the perception node (separate terminal; --show for a debug window)
python3 cmd/aruco_footstep_perception.py --network lo --source sim
```

Real robot: print `unitree_robots/g1/aruco_markers/sheet_target_##.png` at
100% scale (300 dpi), fix them on the physical stepping targets, and run the
perception node with `--source realsense` (requires `pyrealsense2`). An
optional hand-eye correction can be set in `footstep.vision.camera.extrinsic`.

Two things decide whether detection survives an actual walk:

* **Exposure.** Auto-exposure settles near 16 ms indoors and the head motion of
  a step smears the marker corners until the decoder gives up, so the color
  exposure/gain are pinned from `footstep.vision.camera.exposure_us` / `gain`
  (`0` = auto). Tune in the field with `--exposure-us` / `--gain`: shorten it
  if markers hold while standing but drop out while walking.
* **Capture-time waist angles.** The node subscribes `rt/lowstate` and stamps
  every frame with the waist joints it was captured at; the controller runs the
  camera->pelvis FK on those instead of the joints of the tick that consumes
  the frame (tens of ms later, a different head pose). Without `rt/lowstate`
  the node still publishes and the controller falls back, warning once.

The robot steps in place until targets enter the camera view; targets are
remembered (`vision.memory`, world frame) while temporarily out of view.
`cmd/test_aruco_sim_render.py` validates the render->detect->PnP chain against
MuJoCo ground truth without DDS.

### Odometry (`footstep.foot_state_source`)

| Mode | Description |
|------|-------------|
| `fk_odometry` | Default, and the only option on hardware. Foot world poses are dead-reckoned from FK landing measurements; the heading comes from the pelvis IMU. |
| `sim_odom` | MuJoCo only: ground-truth base pose from `rt/odommodestate` plus the FK foot offset, re-measured every tick. |

With `fk_odometry` the accumulated world frame gets its **position** by composing
the measured foot-to-foot transform at each landing, but its **heading** from the
pelvis IMU — re-read every control tick, offset once so the frame starts at the
heading the robot had when walking began (so the arbitrary IMU boot yaw does not
matter). Integrating the per-step relative foot yaw instead, the obvious
alternative, accumulates every landing measurement error without bound and only
advances at step boundaries, so a rotation while the robot is *not* stepping —
a pivot during the standby hold or a stop at a goal — is never seen at all.

Position remains pure dead reckoning and does drift; only an absolute reference
(the `vision` mode, SLAM, mocap) can correct that. Heading matters more in
practice because its error compounds into position error with distance.

### Deployment notes / assumptions

* Single robot (`num_envs == 1`); the batched training tensors are reduced to
  Eigen per-step math.
* No floating-base state estimator is assumed: the pelvis is treated as fixed when
  computing CoM/foot velocities (linear & angular base velocity = 0). The planted
  stance foot makes this a good approximation for the quantities the command
  generator uses. If a base-velocity estimate is available it can be wired into
  `Kinematics`/`FootstepCommand` to improve fidelity.
* The "global" command frame is re-anchored at the pelvis each control tick; this
  is valid because all cross-tick state lives in relative (stance-foot / command)
  frames.
* Start on a flat floor with the robot already balanced (enter from `FixStand`).
  Validate first in sim2sim / loopback DDS (`--network lo`) before the real robot.

Extra care before running `command_source: goal` on hardware:

* Set `foot_state_source: fk_odometry` — `sim_odom` needs `rt/odommodestate`,
  which only the simulator publishes. It is also worth rehearsing in sim with
  `fk_odometry` first, since that is what the robot will actually see.
* The goal frame is fixed at the **Y press**, so have the robot standing settled
  and square before starting; everything downstream is measured from that pose.
* Goal positions are dead-reckoned. The robot reaches each goal in *its own
  estimate*; the true landing spot is off by whatever position drift has
  accumulated, which grows with distance walked.
* `com_z` is not clamped, and `stop.upper_body_pose` moves mass the footstep
  policy was trained with at its defaults. Both change the CoM the preview
  controller is tracking — ramp them up gradually rather than starting at the
  edge of the trained range.
* `vrp_height` / `pelv_com_offset` and the joint gains in `deploy.yaml` are tuned
  for the sim model; re-check them against the physical robot.

## Acknowledgements

This project is derived from the deploy controller architecture in
[unitreerobotics/unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab),
which provides reinforcement learning environments and deployment tools for
Unitree robots based on IsaacLab.
