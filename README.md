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

### 1.1 Simulation stack

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

### 1.2 Behaviour Tree mission architecture

```
                       ┌──────────────────────────────────────────────────┐
                       │        py_trees Behaviour Tree                   │
                       │        (bluerov_square_mission_tree.py)          │
                       │                                                  │
                       │  square_mission  (Sequence, memory=True)         │
                       │  ├── arm_and_set_mode                            │
                       │  ├── leg1_forward   x=+2, y= 0  (forward 2 m)   │
                       │  ├── leg2_left      x= 0, y=+2  (left 2 m)      │
                       │  ├── leg3_backward  x=-2, y= 0  (backward 2 m)  │
                       │  └── leg4_right     x= 0, y=-2  (right 2 m)     │
                       └──────────────────┬───────────────────────────────┘
                                          │  ROS 2 Action  /bluerov/move_rel
                                          │  Goal: PoseStamped (body frame) +
                                          │        anchor_frame_name
                                          ▼
                       ┌──────────────────────────────────────────────────┐
                       │  MoveRelativeActionServer                        │
                       │  (move_relative_action_server.py)                │
                       │                                                  │
                       │  1. TF lookup  anchor_frame → map               │
                       │     tf2_ros.Buffer + do_transform_pose_stamped   │
                       │  2. Anchor offset correction                     │
                       │     mirrors convert_to_controls_pose             │
                       │     recalculate_target() pattern                 │
                       │  3. Publish /mavros/setpoint_position/local      │
                       │     at 1 Hz (higher rates cause replanning)      │
                       │  4. Poll /mavros/local_position/pose             │
                       │     → succeed / abort on dist/yaw threshold      │
                       └──────────────────┬───────────────────────────────┘
                                          │ /mavros/setpoint_position/local
                                          ▼
                              [ MAVROS → ArduSub SITL → Thrusters ]
```

**Setpoint offsets** are expressed in the `base_link` (FLU) frame — `+x` forward, `+y` left, `+z` up. The action server transforms each offset to the `map` frame via TF2 at the time each leg starts, so legs are always relative to the robot's current position.

**Anchor frame** (`anchor_frame_name` goal field) allows goals to target any vehicle frame (camera, gripper, …) rather than `base_link`. When `anchor_frame_name = "base_link"` the correction is a no-op (pure body-relative movement). This mirrors the `recalculate_target()` pattern from `frames/scripts/convert_to_controls_pose.py`.

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
│   ├── foxglove-sdk/            # Foxglove bridge (built from source; see §3)
│   └── bluerov_sim/             # Main orchestrator package
│       ├── config/
│       │   ├── ardusub.parm                # EKF3 external nav parameters
│       │   ├── bluerov_gz_bridge.yaml      # Gazebo bridge: odom only
│       │   └── bluerov_gz_bridge_full.yaml # Gazebo bridge: odom + DVL
│       ├── launch/
│       │   ├── bluerov_sim.launch.py       # Gazebo + ArduSub SITL + MAVROS + Foxglove
│       │   └── bluerov_mission.launch.py   # Odometry node + waypoint mission
│       ├── mavros_params/
│       │   └── sim_mavros_params.yaml
│       ├── models/bluerov2/               # BlueROV2 SDF model with DVL + IMU
│       ├── urdf/
│       │   └── bluerov2.urdf              # Robot description for TF / Foxglove visualisation
│       ├── worlds/                        # World files owned by this package
│       │   └── robosub_2025_pool.world
│       ├── action/
│       │   └── MoveRelativePose.action    # ROS 2 action: body-frame setpoint
│       ├── bluerov_sim/                   # Python package (importable after build)
│       │   ├── arm_and_set_mode.py        # py_trees: arm + GUIDED mode behaviour
│       │   ├── move_relative_behaviour.py # py_trees: MoveRelativePose action client
│       │   └── move_relative_action_server.py  # ROS 2 action server class
│       └── scripts/
│           ├── ground_truth_to_mavros.py  # Gazebo odometry → MAVROS
│           ├── dvl_to_mavros.py           # DVL + GT pose → MAVROS
│           ├── bluerov_movement.py        # Square waypoint mission (state-machine)
│           ├── world_objects_to_markers.py # World SDF → RViz/Foxglove MarkerArray
│           ├── move_relative_action_server.py  # Entry point for action server
│           └── bluerov_square_mission_tree.py  # py_trees square mission entry point
├── docker/
│   └── Dockerfile               # ROS 2 Humble + Gazebo Harmonic + ArduSub SITL
├── build.bash                   # Docker image build wrapper
├── run.bash                     # Rocker launcher (--nvidia --x11 --network=host)
├── bluerov_mission.yaml         # tmuxp session: sim + mission + Foxglove in one command
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

[rocker](https://github.com/osrf/rocker) is required to run the container. It wraps `docker run` with GPU and X11 support.

Setting things up in a virtual environment is recommended for isolation. If you don't already have it, install Python 3's venv module:

```bash
sudo apt-get install python3-venv
```

Create a venv:

```bash
mkdir -p ~/rocker_venv
python3 -m venv ~/rocker_venv
```

Install rocker:

```bash
cd ~/rocker_venv
. ~/rocker_venv/bin/activate
pip install git+https://github.com/osrf/rocker.git
```

For any new terminal, re-activate the venv before trying to use it:

```bash
. ~/rocker_venv/bin/activate
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

### Foxglove Bridge

> **Note:** As of the [2026-03-20 ROS Humble Hawksbill release](https://discourse.openrobotics.org/t/new-packages-for-humble-hawksbill-2026-03-20/53413), `ros-humble-foxglove-bridge` has been **removed** from the official package index. It must be built from source.

Clone the SDK into `src/` so it is picked up by `colcon build`:

```bash
git clone https://github.com/foxglove/foxglove-sdk src/foxglove-sdk
```

Then build it as part of the normal workspace build — no separate `make` step is needed. The bridge is launched automatically by `bluerov_sim.launch.py` and listens on port **8765**.

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

# Tier 5: Foxglove bridge
colcon build --packages-select foxglove_bridge

# Tier 6: All remaining packages
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

### Quick-start with tmuxp

`bluerov_mission.yaml` launches the full stack (simulation + mission + Foxglove) in a single tmux session with three panes:

```bash
tmuxp load bluerov_mission.yaml
```

| Pane | Contents |
| ---- | -------- |
| sim | `bluerov_sim.launch.py` with `world_name:=robosub_2025_pool`, ArduSub + MAVROS + GUI |
| movement | `bluerov_mission.launch.py use_dvl:=false` |
| monitor | Foxglove connection info — open `ws://localhost:8765` in Foxglove Studio |

---

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

### Behaviour tree square mission

Uses a py_trees behaviour tree with a ROS 2 action server. Setpoints are expressed in the `base_link` body frame and transformed to `map` frame at runtime.

```bash
# Terminal 1 — Simulation
source install/setup.bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  world_name:=empty.sdf \
  ardusub:=true \
  mavros:=true \
  gui:=true

# Terminal 2 — Action server + BT mission
source install/setup.bash
ros2 launch bluerov_sim bluerov_square_bt.launch.py
```

The mission arms the vehicle, switches to GUIDED mode, then drives a 2 m × 2 m square:
`leg1 forward (+x=2)` → `leg2 left (+y=2)` → `leg3 backward (x=-2)` → `leg4 right (y=-2)`

Monitor the action server:
```bash
ros2 action list          # /bluerov/move_rel should appear
ros2 action status /bluerov/move_rel
```

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

### Custom robot model

Two arguments control the robot model. **Both must be changed together for a complete swap:**

| Argument | Controls |
|----------|----------|
| `urdf_path` | Foxglove visualization and ROS TF frames |
| `model_sdf_path` | Gazebo physics: collisions, mass, sensors, hydrodynamics, thruster plugins |

Changing only `urdf_path` results in a mismatch — Foxglove shows the custom model but Gazebo continues to simulate BlueROV2 physics and collisions.

```bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  urdf_path:=/path/to/your/robot.urdf \
  model_sdf_path:=/path/to/your/model.sdf
```

When writing a custom URDF, mirror the joint names and TF frame names from your SDF to keep TF consistent with Gazebo.

### Alternative: DAVE Underwater Camera plugin

The DAVE ecosystem (`src/dave/`) includes `dave_gz_sensor_plugins::UnderwaterCamera` — a Gazebo Harmonic sensor plugin that applies physically-based underwater optics on top of a standard depth camera render. It was evaluated as an alternative to the standard Gazebo camera implemented above.

**How it works:**
- Uses `type="rgbd_camera"` sensor (depth + RGB) in SDF
- Applies per-pixel exponential light attenuation (separate R/G/B coefficients) and background scattering using the depth buffer
- Publishes directly to ROS 2 as `sensor_msgs/msg/Image` on `{topic}/simulated_image` — no `ros_gz_bridge` entry needed for the image
- Reference implementation: `src/dave/models/dave_robot_models/description/bluerov2/model.sdf`

**Example SDF sensor block:**
```xml
<sensor name="front_cam" type="rgbd_camera">
  <update_rate>10</update_rate>
  <always_on>1</always_on>
  <topic>/bluerov/front_cam</topic>
  <camera>
    <horizontal_fov>1.05</horizontal_fov>
    <image><width>1920</width><height>1080</height></image>
    <clip><near>0.1</near><far>10.0</far></clip>
  </camera>
  <plugin filename="UnderwaterCamera" name="dave_gz_sensor_plugins::UnderwaterCamera">
    <attenuationR>0.8</attenuationR>  <!-- murky coastal water -->
    <attenuationG>0.5</attenuationG>
    <attenuationB>0.2</attenuationB>
    <backgroundR>85</backgroundR>
    <backgroundG>107</backgroundG>
    <backgroundB>47</backgroundB>
  </plugin>
</sensor>
```

**Standard Gazebo Camera (implemented) vs DAVE Underwater Camera:**

| Criterion | Standard Gazebo Camera | DAVE Underwater Camera |
|-----------|------------------------|------------------------|
| Underwater visual realism | None — standard RGB render | Yes — per-pixel attenuation + scatter |
| New dependencies | None | `dave_gz_sensor_plugins` + plugin path setup |
| Plugin path risk | None | `dave_ws` install must be on `GZ_SIM_SYSTEM_PLUGIN_PATH` |
| Rendering overhead | Low (RGB only) | Higher (~20–30%; forced depth buffer) |
| `CameraInfo` topic | Yes (via bridge) | No |
| Image encoding | RGB8 | BGR8 (non-standard for ROS) |
| Topic suffix | configurable | hardcoded `{base}/simulated_image` |
| World SDF requirement | None | `gz-sim-rgbd-camera-system` must be loaded |

**Prerequisites to enable the DAVE plugin:**
1. `dave_gz_sensor_plugins` must be built and its install lib directory added to `GZ_SIM_SYSTEM_PLUGIN_PATH`
2. The world SDF must load the `gz-sim-rgbd-camera-system` Gazebo system plugin
3. Any downstream ROS 2 node receiving images must handle BGR8 encoding explicitly
4. No `CameraInfo` is published — intrinsics must be supplied separately if needed

---

> **SDF → URDF conversion:** No zero-setup conversion tool is available in this environment. [`sdf_to_urdf`](https://github.com/andreasBihlmaier/sdf_to_urdf) is a ROS 2 C++ package that wraps `sdformat_urdf` and provides a CLI converter. It requires cloning and `colcon build`, and is early-stage (v0.0.0, created Aug 2025). Confirmed working on Humble/Jazzy; broken on Rolling as of June 2025.

---

### QGroundControl

QGroundControl runs on the **host machine** (not inside the container). Because the container uses `--network=host`, MAVROS broadcasts MAVLink telemetry on UDP 14550, which QGC picks up automatically on the same machine.

Download QGC for your OS from [qgroundcontrol.com](https://qgroundcontrol.com).

If the vehicle does not appear automatically after QGC starts, add a link manually:

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
| `debug`                | `false`     | Enable GDB debug mode for ArduSub SITL                 |
| `namespace`            | `bluerov`   | Gazebo model name                                      |
| `x` / `y` / `z`       | `0.0`       | Initial position (metres)                              |
| `roll` / `pitch` / `yaw` | `0.0`     | Initial orientation (radians)                          |
| `use_sim_time`         | `true`      | Use Gazebo simulation clock                            |
| `verbose`              | `0`         | Gazebo console verbosity level                         |
| `urdf_path`            | `<package>/urdf/bluerov2.urdf`        | Path to URDF for robot_state_publisher and Foxglove visualization |
| `model_sdf_path`       | `<package>/models/bluerov2/model.sdf` | Path to SDF model for Gazebo spawning (physics, collisions, sensors) |

> `bluerov_sim.launch.py` also unconditionally starts **robot_state_publisher**, **joint_state_publisher**, **foxglove_bridge** (WebSocket port 8765), and **world_objects_to_markers** (publishes world SDF objects as a `MarkerArray` on `/world_objects/markers`).

### `bluerov_mission.launch.py`

| Argument  | Default | Description                                                              |
| --------- | ------- | ------------------------------------------------------------------------ |
| `use_dvl` | `false` | Use `dvl_to_mavros.py`; `false` uses `ground_truth_to_mavros.py`         |

### `bluerov_square_bt.launch.py`

No launch arguments. Starts `ground_truth_to_mavros`, `move_relative_action_server`, and `bluerov_square_mission_tree` together. Requires the simulation to be running first.

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

# Monitor world object markers
ros2 topic echo /world_objects/markers
```

### Foxglove Studio

Connect Foxglove Studio to the running simulation:

```
Open Foxglove Studio → Open Connection → Rosbridge (WebSocket)
URL: ws://localhost:8765
```

Key topics to add as panels: `/mavros/local_position/pose`, `/bluerov/odom`, `/tf`, `/world_objects/markers`.
