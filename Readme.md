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
- Helper scripts under `cmd/` to generate foot-command CSVs and MuJoCo stepping-stone scenes.
- Designed for Unitree G1 29-DoF deployment workflows.

## Project Layout

```text
.
├── cmd/
│   ├── gen_cmd.py                    # sample local foot commands -> footcommands.csv
│   ├── convert_footcommand_2_global.py
│   └── gen_footstep_scene.py         # write stepping stones into MuJoCo scene XML
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

For footstep sim, use the footstep scene in `unitree_mujoco` and set
`footstep.foot_state_source: sim_odom` in the footstep `deploy.yaml`.

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

For `csv` / `csv_global`, generate the CSV files with the `cmd/` scripts (below).
Set `footstep.global_init_lfoot` / `global_init_rfoot` to the spawn foot poses in
the MuJoCo scene keyframe so the global plan frame matches the simulator.

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

## Acknowledgements

This project is derived from the deploy controller architecture in
[unitreerobotics/unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab),
which provides reinforcement learning environments and deployment tools for
Unitree robots based on IsaacLab.
