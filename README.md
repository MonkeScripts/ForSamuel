# bluerov_ws

> **Work in progress**

ROS 2 Humble + Gazebo Harmonic workspace for **BlueROV2** simulation with **ArduSub SITL** and **MAVROS**.

---

## Table of Contents

1. [Architecture](#1-architecture)
2. [Repository Layout](#2-repository-layout)
3. [Docker Setup](#3-docker-setup)
4. [Building the Workspace](#4-building-the-workspace)
5. [Running Simulations](#5-running-simulations)
6. [Launch Arguments](#6-launch-arguments)
7. [Useful ROS 2 Commands](#7-useful-ros-2-commands)

---

## 1. Architecture

```
                        ┌─────────────────────────────┐
                        │        Gazebo Harmonic        │
                        │                               │
                        │  BlueROV2 model (SDF)         │
                        │  ├─ 6 thrusters               │
                        │  ├─ BuoyancyPlugin             │
                        │  ├─ HydrodynamicsPlugin        │
                        │  ├─ IMU sensor                 │
                        │  └─ DVL sensor (Nortek 500)    │
                        └──────┬────────────────┬───────┘
                               │ JSON over UDP  │ Gazebo topics
                               ▼                ▼
                 ┌─────────────────────┐  ┌────────────────────────┐
                 │  ArduPilot Gazebo   │  │     ros_gz_bridge       │
                 │  Plugin             │  │                         │
                 │  (libArduPilot      │  │  /bluerov/odom          │
                 │   Plugin.so)        │  │  /bluerov/dvl/velocity  │
                 └──────────┬──────────┘  └──────────┬─────────────┘
                            │ MAVLink TCP :5760        │ ROS 2 topics
                            ▼                          ▼
                 ┌─────────────────────┐  ┌────────────────────────┐
                 │   ArduSub SITL      │  │   Odometry Node         │
                 │                     │  │                         │
                 │  EKF3 fuses:        │  │  ground_truth_to_       │
                 │  ├─ external nav    │  │  mavros.py              │
                 │  └─ barometer (Z)   │  │      ── OR ──           │
                 └──────────┬──────────┘  │  dvl_to_mavros.py      │
                            │ MAVLink TCP  └──────────┬─────────────┘
                            ▼                         │ /mavros/odometry/out
                 ┌─────────────────────────────────────────────────┐
                 │                    MAVROS                        │
                 │  /mavros/setpoint_position/local  (input)        │
                 │  /mavros/local_position/pose      (output)       │
                 │  /mavros/state                    (output)       │
                 └──────────────────────┬──────────────────────────┘
                                        │ ROS 2
                                        ▼
                       ┌────────────────────────────────┐
                       │   bluerov_movement.py           │
                       │                                 │
                       │  INIT → ARM → GUIDED →          │
                       │  WAYPOINT (4 pts, -2m depth) →  │
                       │  FINISHED                        │
                       └────────────────────────────────┘
                                        │ UDP 14550
                                        ▼
                            ┌────────────────────┐
                            │   QGroundControl   │
                            │   (GCS, optional)  │
                            └────────────────────┘
```

**ArduSub** is the production firmware for real BlueROV2 hardware. Running ArduSub SITL means parameter files, GCS connections, and MAVLink command sequences are identical between simulation and vehicle. **MAVROS** provides the ROS 2 abstraction layer so mission scripts require no changes when moving from simulation to hardware.

---

## 2. Repository Layout

```
bluerov_ws/
├── src/
│   ├── ardupilot_gazebo/        # ArduPilot Gazebo Plugin (JSON-UDP bridge to ArduSub SITL)
│   ├── dave/                    # DAVE underwater simulation ecosystem
│   │   ├── dave_interfaces/     # Custom msgs: DVL.msg, DVLBeam.msg, ocean current services
│   │   ├── dave_gz_model_plugins/
│   │   ├── dave_gz_sensor_plugins/
│   │   ├── dave_gz_world_plugins/
│   │   └── dave_ros_gz_plugins/
│   ├── bb_worlds/               # Competition worlds (RoboSub 2023/2025, SAUVC 2024)
│   └── bluerov_sim/             # Main orchestrator package
│       ├── config/
│       │   ├── ardusub.parm                # EKF3 external nav parameters
│       │   ├── bluerov_gz_bridge.yaml      # Gazebo bridge: odom only
│       │   └── bluerov_gz_bridge_full.yaml # Gazebo bridge: odom + DVL
│       ├── launch/
│       │   ├── bluerov_sim.launch.py       # Gazebo + ArduSub SITL + MAVROS
│       │   └── bluerov_mission.launch.py   # Odometry node + waypoint mission
│       ├── mavros_params/
│       │   └── sim_mavros_params.yaml
│       ├── models/bluerov2/               # BlueROV2 SDF model with DVL + IMU
│       ├── worlds/                        # World files owned by this package
│       │   └── robosub_2025_pool.world
│       └── scripts/
│           ├── ground_truth_to_mavros.py  # Gazebo odometry → MAVROS
│           ├── dvl_to_mavros.py           # DVL + GT pose → MAVROS
│           └── bluerov_movement.py        # Square waypoint mission
├── docker/
│   └── Dockerfile               # ROS 2 Humble + Gazebo Harmonic + ArduSub SITL
├── build.bash                   # Docker image build wrapper
├── run.bash                     # Rocker launcher (--nvidia --x11 --network=host)
└── bluerov_ws.repos             # vcs-tool manifest
```

### `.repos` manifest

| Package             | Source                     | Notes                                      |
| ------------------- | -------------------------- | ------------------------------------------ |
| `ardupilot_gazebo`  | ArduPilot/ardupilot_gazebo | Pinned commit; JSON-UDP bridge plugin      |
| `dave`              | IOES-Lab/dave              | Pinned commit; sensor plugins + interfaces |
| `bb_worlds`         | BumblebeeAS/bb_worlds      | `main` branch; RoboSub/SAUVC worlds        |

---

## 3. Docker Setup

The Docker image (`ros:humble-ros-base-jammy`) includes ROS 2 Humble, Gazebo Harmonic, ArduSub SITL, and all build dependencies.

### Install rocker

[rocker](https://github.com/osrf/rocker) is required to run the container. It wraps `docker run` with GPU and X11 support:

```bash
pip install rocker
```

### Build the image

From the workspace root:

```bash
./build.bash
# Tags image as bluerov_ws:humble and bluerov_ws:YYYY_MM_DD_HHMM
```

### Run the container

```bash
./run.bash bluerov_ws:humble
```

This launches the container with:
- NVIDIA GPU passthrough (`--nvidia`)
- X11 display forwarding (`--x11`)
- Host network (`--network=host`) — required for MAVROS and QGroundControl UDP
- `$HOME` mounted at `/root/HOST` — workspace persists across restarts

The workspace is accessible at `/root/HOST/bluerov_ws` inside the container.

**Run flags:**

| Flag   | Effect                                              |
| ------ | --------------------------------------------------- |
| (none) | Default: NVIDIA + X11 + joystick + host network     |
| `-c`   | Add CUDA library support                            |
| `-s`   | noVNC / TurboVNC image for CloudSim                 |
| `-t`   | CI/headless mode (no X11)                           |
| `-x`   | VRX competition server base image                   |

---

## 4. Building the Workspace

Run these steps inside the container. Packages must be built in dependency order — source after each tier to activate `setup.dsv` environment hooks.

```bash
cd /root/HOST/bluerov_ws
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic

# Import all repos
vcs import src < bluerov_ws.repos

# Install ROS deps
rosdep update
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "ardupilot_gazebo orb_slam_2_ros gz-harmonic"

# Tier 1: Gazebo plugin (sets GZ_SIM_SYSTEM_PLUGIN_PATH)
colcon build \
  --packages-select ardupilot_gazebo \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

# Tier 2: ROS interface definitions
colcon build --packages-select dave_interfaces

# Tier 3: Gazebo sensor and world plugins
colcon build \
  --packages-select dave_gz_world_plugins dave_gz_sensor_plugins

# Tier 4: ROS-Gazebo bridge plugins
colcon build \
  --packages-select dave_gz_model_plugins dave_ros_gz_plugins

# Tier 5: All remaining packages
colcon build \
  --packages-select dave bluerov_sim bb_worlds

source install/setup.bash
```

**Subsequent rebuilds** (no tier ordering needed):

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

---

## 5. Running Simulations

### Ground truth odometry (simplest)

```bash
# Terminal 1 — Simulation
source install/setup.bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  world_name:=empty.sdf \
  ardusub:=true \
  mavros:=true \
  gui:=true

# Terminal 2 — Square waypoint mission
source install/setup.bash
ros2 launch bluerov_sim bluerov_mission.launch.py use_dvl:=false
```

The mission arms the vehicle, switches to GUIDED mode, then navigates a square at -2 m depth:
`(0,0,-2)` → `(5,0,-2)` → `(5,5,-2)` → `(0,5,-2)` → `(0,0,-2)`

### DVL-based odometry

Uses the Nortek DVL500-300 sensor for velocity, fused with ground truth pose.

```bash
# Terminal 1 — same as above
ros2 launch bluerov_sim bluerov_sim.launch.py \
  world_name:=empty.sdf ardusub:=true mavros:=true gui:=true

# Terminal 2 — DVL odometry + mission
ros2 launch bluerov_sim bluerov_mission.launch.py use_dvl:=true
```

When `use_dvl:=true`, `dvl_to_mavros.py` combines ground truth pose and DVL twist into `/mavros/odometry/out`.

### Headless / CI mode

```bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  headless:=true ardusub:=true mavros:=true
```

### Competition worlds

Pass the world filename (without extension) as `world_name`. World files are resolved automatically via `GZ_SIM_RESOURCE_PATH` when the workspace is sourced.

Available worlds:

| `world_name`            | Package        | Description                        |
| ----------------------- | -------------- | ---------------------------------- |
| `robosub_2025_pool`     | `bluerov_sim`  | RoboSub 2025 competition pool      |
| `robosub_2023_pool`     | `bb_worlds`    | RoboSub 2023 competition pool      |
| `robosub_2025_prequali` | `bb_worlds`    | RoboSub 2025 pre-qualification     |
| `sauvc_2024`            | `bb_worlds`    | SAUVC 2024 competition pool        |
| `dave_ocean_waves`      | `bb_worlds`    | Open ocean with wave simulation    |
| `robotx_2026_sg_river`  | `bb_worlds`    | RobotX 2026 Singapore river        |

```bash
# Terminal 1 — Simulation with a competition world
source install/setup.bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  world_name:=robosub_2025_pool \
  ardusub:=true \
  mavros:=true \
  gui:=true

# Terminal 2 — Mission
source install/setup.bash
ros2 launch bluerov_sim bluerov_mission.launch.py use_dvl:=false
```

> The launch file parses `spherical_coordinates` from the world SDF to set ArduSub's home location automatically. If the world file is not found on `GZ_SIM_RESOURCE_PATH`, it falls back to default coordinates (33.810313, -118.393867).

### QGroundControl

Connect QGroundControl to the MAVROS GCS bridge over UDP:

```
Application Settings → Comm Links → Add
Type: UDP
Port: 14550
```

---

## 6. Launch Arguments

### `bluerov_sim.launch.py`

| Argument               | Default     | Description                                            |
| ---------------------- | ----------- | ------------------------------------------------------ |
| `world_name`           | `empty.sdf` | Gazebo world file (resolved via `GZ_SIM_RESOURCE_PATH`) |
| `ardusub`              | `true`      | Launch ArduSub SITL                                    |
| `mavros`               | `true`      | Launch MAVROS node                                     |
| `gui`                  | `true`      | Show Gazebo GUI                                        |
| `headless`             | `false`     | Headless (server-only) mode                            |
| `paused`               | `false`     | Start simulation paused                                |
| `namespace`            | `bluerov`   | Gazebo model name                                      |
| `x` / `y` / `z`       | `0.0`       | Initial position (metres)                              |
| `roll` / `pitch` / `yaw` | `0.0`     | Initial orientation (radians)                          |
| `use_sim_time`         | `true`      | Use Gazebo simulation clock                            |
| `verbose`              | `0`         | Gazebo console verbosity level                         |

### `bluerov_mission.launch.py`

| Argument  | Default | Description                                                              |
| --------- | ------- | ------------------------------------------------------------------------ |
| `use_dvl` | `false` | Use `dvl_to_mavros.py`; `false` uses `ground_truth_to_mavros.py`         |

---

## 7. Useful ROS 2 Commands

```bash
# Check MAVROS connection state
ros2 topic echo /mavros/state

# Monitor vehicle position
ros2 topic echo /mavros/local_position/pose

# Check Gazebo odometry is flowing
ros2 topic echo /bluerov/odom

# Monitor DVL (if use_dvl:=true)
ros2 topic echo /bluerov/dvl/velocity

# List all MAVROS topics
ros2 topic list | grep mavros

# Arm manually
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Switch to GUIDED mode manually
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
```
