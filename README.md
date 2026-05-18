# bluerov_ws

> **Work in progress**

ROS 2 Humble + Gazebo Harmonic workspace for **BlueROV2** simulation with **ArduSub SITL** and **MAVROS**. The full stack — Gazebo, ArduSub firmware, MAVROS, and a py_trees behaviour-tree mission — runs from a single `tmuxp` command.

For non-default worlds, odometry sources, custom robots, headless runs, and the full launch-argument reference, see [`docs/customization.md`](docs/customization.md).

---

## Table of Contents

1. [Architecture](#1-architecture)
2. [Docker Setup](#2-docker-setup)
3. [Building the Workspace](#3-building-the-workspace)
4. [Running the Demo](#4-running-the-demo)
5. [Diagnostics](#5-diagnostics)

---

## 1. Architecture

### 1.1 Simulation stack

```
Gazebo Harmonic
  ├─ JSON over UDP ──────► ArduPilot Gazebo Plugin ─── MAVLink TCP :5760 ──► ArduSub SITL
  └─ Gazebo topics ──────► ros_gz_bridge                                           │
                                 │                                                  │ MAVLink TCP
                                 │ ROS 2 topics                                     │
                                 ▼                                                  ▼
                           ground_truth_to_mavros.py ── /mavros/odometry/out ───► MAVROS
                                                                              ├─► BT mission
                                                                              └─► QGroundControl (UDP 14550)
```

| Component                   | Role                                                                                                |
| --------------------------- | --------------------------------------------------------------------------------------------------- |
| **Gazebo Harmonic**         | Simulates BlueROV2 — 6 thrusters, BuoyancyPlugin, HydrodynamicsPlugin, IMU                          |
| **ArduPilot Gazebo Plugin** | Bridges Gazebo physics to ArduSub via JSON over UDP (`libArduPilotPlugin.so`)                       |
| **ArduSub SITL**            | Flight controller firmware (same binary as the real BlueROV2); EKF3 fuses external nav + barometer |
| **ros_gz_bridge**           | Forwards Gazebo topics (`/bluerov/odom`, …) to ROS 2                                                |
| **ground_truth_to_mavros**  | Publishes Gazebo odom to `/mavros/odometry/out` and broadcasts `map → base_link` TF                 |
| **MAVROS**                  | ROS 2 ↔ MAVLink bridge; exposes `/mavros/setpoint_position/local` (in) and `/mavros/local_position/pose` (out) |
| **QGroundControl**          | Ground control station on the host — connects automatically via UDP 14550                          |

Because ArduSub SITL is the same firmware that runs on real BlueROV2 hardware and MAVROS provides the ROS 2 abstraction, mission code is identical between sim and vehicle.

### 1.2 Mission stack

```
py_trees Behaviour Tree  (bluerov_square_mission_tree.py)
  square_mission — Sequence, memory=True
  ├── arm_and_set_mode
  ├── leg1_forward   x=+2, y= 0  (forward 2 m)
  ├── leg2_left      x= 0, y=+2  (left   2 m)
  ├── leg3_backward  x=−2, y= 0  (back   2 m)
  └── leg4_right     x= 0, y=−2  (right  2 m)
         │
         │ GetPoseToControlsFrame service
         │ /bluerov/convert_to_controls_pose  (base_link → map)
         ▼
  ConvertToControlsPose  (frames/scripts/convert_to_controls_pose.py)
         │
         │ ROS 2 Action  /bluerov/controls
         │ Goal: Locomotion (map-frame pose, move_rel=False)
         ▼
  LocomotionActionServer  (locomotion_action_server.py)
    1. Accept map-frame setpoint
    2. Publish /mavros/setpoint_position/local at 1 Hz
    3. Poll /mavros/local_position/pose → succeed / abort on threshold
         │
         ▼
  MAVROS → ArduSub SITL → Thrusters
```

**Body-frame setpoints.** BT legs are expressed in `base_link` FLU — `+x` forward, `+y` left, `+z` up. `ConvertToControlsPose` (from the `frames` package) converts each body-frame pose to the `map` frame using current odometry so legs are always relative to the robot's current position.

**Anchor frame.** Goals carry an `anchor_frame_name` (default `base_link`). When the anchor is something else (camera, gripper, …) the conversion subtracts the anchor's offset from `base_link`, so "move forward 2 m" can mean "until the camera is 2 m ahead of where it is now" instead of "until the body is."

---

## 2. Docker Setup

Everything runs inside the Docker container. The image (`ros:humble-ros-base-jammy`) ships ROS 2 Humble, Gazebo Harmonic, ArduSub SITL, and all build dependencies.

### Install rocker

[rocker](https://github.com/osrf/rocker) wraps `docker run` with GPU and X11 support.

```bash
sudo apt-get install python3-venv
python3 -m venv ~/rocker_venv
. ~/rocker_venv/bin/activate
pip install git+https://github.com/osrf/rocker.git
```

Re-activate the venv in any new terminal: `. ~/rocker_venv/bin/activate`.

### Build the image

```bash
./build.bash
# Tags as bluerov_ws:humble and bluerov_ws:YYYY_MM_DD_HHMM
```

### Run the container

```bash
./run.bash bluerov_ws:humble
```

This launches with:

- NVIDIA GPU passthrough (`--nvidia`)
- X11 display forwarding (`--x11`)
- Host network (`--network=host`) — required for MAVROS / QGC UDP 14550 and Foxglove on 8765
- `$HOME` mounted at `/root/HOST` — workspace persists across restarts

The workspace is available at `/root/HOST/bluerov_ws` inside the container.

### QGroundControl (host)

QGC is required — it is the ground control station used to monitor and (when needed) override the vehicle. It runs on the **host machine**, not in the container. Because the container uses `--network=host`, MAVROS broadcasts MAVLink telemetry on UDP 14550 and QGC picks it up automatically.

Download QGC for your OS from [qgroundcontrol.com](https://qgroundcontrol.com) and start it before launching the demo. If the vehicle does not appear automatically, add a UDP link manually on port `14550` (`Application Settings → Comm Links → Add`).

---

## 3. Building the Workspace

Inside the container, on first checkout, pull external packages via `vcs`:

```bash
cd /root/HOST/bluerov_ws
vcs import src < bluerov_ws.repos
```

The Foxglove bridge must be cloned manually — `ros-humble-foxglove-bridge` was [removed from the Humble package index on 2026-03-20](https://discourse.openrobotics.org/t/new-packages-for-humble-hawksbill-2026-03-20/53413):

```bash
git clone https://github.com/foxglove/foxglove-sdk src/foxglove-sdk
```

Build with `colcon` (always use `--symlink-install` so Python edits take effect without a rebuild):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 4. Running the Demo

Make sure QGroundControl is running on the host (see [§2 QGroundControl](#qgroundcontrol-host)), then from inside the container:

```bash
tmuxp load bluerov_mission.yaml
```

That's the whole demo. It opens one tmux window with two panes:

| Pane         | Contents                                                                                          |
| ------------ | ------------------------------------------------------------------------------------------------- |
| `sim`        | `bluerov_sim.launch.py` — Gazebo (`robosub_2025_pool`) + ArduSub SITL + MAVROS + Foxglove bridge  |
| `bt_mission` | `bluerov_square_bt.launch.py` — locomotion action server + frames service + BT square mission    |

The Foxglove bridge is reachable at `ws://localhost:8765` — connect with [Foxglove Studio](https://foxglove.dev) on the host.

The behaviour tree arms the vehicle, switches it to GUIDED, then drives a 2 m × 2 m square:
`leg1 forward (+x=2)` → `leg2 left (+y=2)` → `leg3 backward (x=−2)` → `leg4 right (y=−2)`.

Useful Foxglove panels: `/tf`, `/mavros/local_position/pose`, `/bluerov/odom`, `/world_objects/markers`.

---

## 5. Diagnostics

```bash
ros2 topic echo /mavros/state                       # FCU/MAVROS connection state
ros2 topic echo /mavros/local_position/pose         # vehicle pose (feedback path)
ros2 topic echo /bluerov/odom                       # raw Gazebo odom
ros2 action list   | grep controls                  # /bluerov/controls present?
ros2 service list  | grep convert_to_controls_pose  # /bluerov/convert_to_controls_pose present?

# Manual arming / mode switch (bypass the BT)
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode   mavros_msgs/srv/SetMode    "{custom_mode: 'GUIDED'}"
```
