## g1_controller

This repository contains a minimal **Unitree G1 low-level controller** (`g1_ctl`) built on **Unitree SDK2 DDS channels**.
It is commonly used together with:

- **`yongarry/unitree_mujoco`**: MuJoCo simulator that publishes/subscribes Unitree SDK2 DDS topics (e.g. `rt/lowstate`, `rt/lowcmd`)
- **`g1_gui` (included in this repo)**: Web UI (served by `g1_gui_server`) to monitor joint state via WebSocket

### Expected workspace layout

`launch_g1_mujoco.sh` assumes `unitree_mujoco` is a sibling of this repo (same parent directory):

```
unitree_ws/
  g1_controller/        # this repo
  unitree_mujoco/       # cloned separately
```

### Dependencies to install

- **ROS distro Pinocchio** (Pinocchio from ROS packages)
- **Unitree SDK2** (`unitree_sdk2`)

## Install

### 1) Install Pinocchio (ROS distro package)

Install Pinocchio from your ROS distribution packages (the package name depends on your ROS distro and OS).
Make sure `pinocchio` is discoverable by CMake (`find_package(pinocchio REQUIRED)` works).

### 2) Install Unitree SDK2
It is recommended to install `unitree_sdk2` in `/opt/unitree_robotics` path.
```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```
For more details, see: https://github.com/unitreerobotics/unitree_sdk2

### 3) Install Mujoco Simulator
#### 1. Mujoco

Download the mujoco 3.8.0 [release](https://github.com/google-deepmind/mujoco/releases), and extract it to the `~/.mujoco` directory;

```
cd unitree_mujoco/simulate/
ln -s ~/.mujoco/mujoco-3.8.0 mujoco
```

#### 2. Compile unitree_mujoco
```bash
git clone https://github.com/yongarry/unitree_mujoco.git
cd unitree_mujoco/simulate
mkdir build && cd build
cmake ..
make -j4
```

## Build

From the repo root (`unitree_ws/g1_controller`):

```bash
mkdir -p build
cd build
cmake ..
make
```

This builds both binaries in one build directory:

- `build/g1_controller/g1_ctl`
- `build/g1_gui/g1_gui_server`

## Run

### Run `g1_ctl` directly

`g1_ctl` requires the DDS network interface as the first argument:

```bash
./build/g1_controller/g1_ctl lo
```

### Run MuJoCo + controller + GUI

If you have `unitree_mujoco` built, you can start everything together:

```bash
./launch_g1_mujoco.sh 
```

Then open the GUI (if enabled in your setup) in a browser (commonly `http://127.0.0.1:4710/`).

## Troubleshooting

- **`find_package(unitree_sdk2 REQUIRED)` fails**:
  - Verify Unitree SDK2 is installed under `/opt/unitree_robotics`
  - Verify `/opt/unitree_robotics/lib/cmake` exists and contains `unitree_sdk2Config.cmake`
- **`find_package(pinocchio REQUIRED)` fails**:
  - Ensure the Pinocchio ROS distro package is installed
  - Ensure CMake can find it (CMAKE_PREFIX_PATH / environment setup)
- **No DDS data in simulation**:
  - Use `lo` for local simulation unless you explicitly need another interface
