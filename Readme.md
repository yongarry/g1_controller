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

- FSM-based controller with `Passive`, `FixStand`, velocity RL, and mimic policy states.
- Deploys exported RL policies from `params/deploy.yaml` and `exported/policy.onnx`.
- Supports policy directory version discovery, such as `config/policy/velocity/v0`.
- Uses joystick transition expressions defined in `config/config.yaml`.
- Designed for Unitree G1 29-DoF deployment workflows.

## Project Layout

```text
.
├── config/
│   ├── config.yaml              # FSM states, transitions, and policy paths
│   └── policy/                  # Exported deploy policies
├── include/
│   ├── FSM/                     # Base FSM state classes
│   └── isaaclab/                # Deploy-side IsaacLab-style runtime helpers
├── src/
│   ├── State_RLBase.cpp         # Generic RL policy deploy state
│   └── State_Mimic.cpp          # Motion/mimic policy deploy state
├── thirdparty/
│   └── onnxruntime-linux-x64-1.22.0/
├── CMakeLists.txt
└── main.cpp
```

## Dependencies

Install the system dependencies used by the deploy controller:

```bash
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```

Install `unitree_sdk2` system-wide:

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF
sudo make install
```

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

## Acknowledgements

This project is derived from the deploy controller architecture in
[unitreerobotics/unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab),
which provides reinforcement learning environments and deployment tools for
Unitree robots based on IsaacLab.
