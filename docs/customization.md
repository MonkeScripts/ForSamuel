# Customization

Alternatives to the default `tmuxp load bluerov_mission.yaml` flow described in the [README](../README.md). Use this when you need a different world, a different odometry source, a custom robot model, or a headless run.

---

## Manual two-terminal launch

```bash
# Terminal 1 — simulation + ArduSub SITL + MAVROS + Foxglove bridge
source install/setup.bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  world_name:=robosub_2025_pool \
  ardusub:=true \
  mavros:=true \
  gui:=true

# Terminal 2 — behaviour-tree square mission
source install/setup.bash
ros2 launch bluerov_sim bluerov_square_bt.launch.py
```

## Waypoint mission without behaviour trees

State-machine mission flying a 5 m × 5 m square at –2 m depth instead of the behaviour-tree square:

```bash
ros2 launch bluerov_sim bluerov_mission.launch.py use_dvl:=false
```

`(0,0,-2)` → `(5,0,-2)` → `(5,5,-2)` → `(0,5,-2)` → `(0,0,-2)`

## DVL-based odometry

The default is ground truth via `ground_truth_to_mavros.py`. To swap in the Nortek DVL500-300 twist (with GT pose), use the `use_dvl` flag on `bluerov_mission.launch.py`:

```bash
ros2 launch bluerov_sim bluerov_mission.launch.py use_dvl:=true
```

`dvl_to_mavros.py` fuses ground truth pose with DVL twist into `/mavros/odometry/out`.

## Headless / CI mode

```bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  headless:=true ardusub:=true mavros:=true
```

## Competition worlds

Pass the world filename (without extension) as `world_name`. World files are resolved via `GZ_SIM_RESOURCE_PATH` once the workspace is sourced.

| `world_name`            | Package       | Description                     |
| ----------------------- | ------------- | ------------------------------- |
| `robosub_2025_pool`     | `bluerov_sim` | RoboSub 2025 competition pool   |
| `robosub_2023_pool`     | `bb_worlds`   | RoboSub 2023 competition pool   |
| `robosub_2025_prequali` | `bb_worlds`   | RoboSub 2025 pre-qualification  |
| `sauvc_2024`            | `bb_worlds`   | SAUVC 2024 competition pool     |
| `dave_ocean_waves`      | `bb_worlds`   | Open ocean with wave simulation |
| `robotx_2026_sg_river`  | `bb_worlds`   | RobotX 2026 Singapore river     |

`bluerov_sim.launch.py` parses `spherical_coordinates` from the world SDF to set ArduSub's home location. If the world is not on `GZ_SIM_RESOURCE_PATH`, it falls back to a default lat/lon (33.810313, -118.393867).

## Custom robot model

Two arguments control the robot model. **Both must be changed together for a complete swap:**

| Argument         | Controls                                                                |
| ---------------- | ----------------------------------------------------------------------- |
| `urdf_path`      | Foxglove visualisation + ROS TF frames                                  |
| `model_sdf_path` | Gazebo physics: collisions, mass, sensors, hydrodynamics, thrusters     |

Changing only `urdf_path` produces a silent mismatch — Foxglove shows the custom model but Gazebo continues to simulate BlueROV2 physics and collisions.

```bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  urdf_path:=/path/to/your/robot.urdf \
  model_sdf_path:=/path/to/your/model.sdf
```

When writing a custom URDF, mirror joint names and TF frame names from your SDF to keep TF consistent with Gazebo.

## QGroundControl

QGC runs on the **host machine**, not in the container. Because the container uses `--network=host`, MAVROS broadcasts MAVLink telemetry on UDP 14550 and QGC picks it up automatically.

If the vehicle does not appear after QGC starts, add a link manually:

```
Application Settings → Comm Links → Add
Type: UDP
Port: 14550
```

---

## Launch argument reference

### `bluerov_sim.launch.py`

| Argument                 | Default                               | Description                                                          |
| ------------------------ | ------------------------------------- | -------------------------------------------------------------------- |
| `world_name`             | `empty.sdf`                           | Gazebo world file (resolved via `GZ_SIM_RESOURCE_PATH`)              |
| `ardusub`                | `true`                                | Launch ArduSub SITL                                                  |
| `mavros`                 | `true`                                | Launch MAVROS node                                                   |
| `gui`                    | `true`                                | Show Gazebo GUI                                                      |
| `headless`               | `false`                               | Headless (server-only) mode                                          |
| `paused`                 | `false`                               | Start simulation paused                                              |
| `debug`                  | `false`                               | Enable GDB debug mode for ArduSub SITL                               |
| `namespace`              | `bluerov`                             | Gazebo model name                                                    |
| `x` / `y` / `z`          | `0.0`                                 | Initial position (m)                                                 |
| `roll` / `pitch` / `yaw` | `0.0`                                 | Initial orientation (rad)                                            |
| `use_sim_time`           | `true`                                | Use Gazebo simulation clock                                          |
| `verbose`                | `0`                                   | Gazebo console verbosity                                             |
| `urdf_path`              | `<package>/urdf/bluerov2.urdf`        | URDF for robot_state_publisher and Foxglove visualisation            |
| `model_sdf_path`         | `<package>/models/bluerov2/model.sdf` | SDF model for Gazebo spawning (physics, collisions, sensors)         |

`bluerov_sim.launch.py` also unconditionally starts **robot_state_publisher**, **joint_state_publisher**, **foxglove_bridge** (WebSocket port 8765), and **world_objects_to_markers** (publishes world SDF objects as a `MarkerArray` on `/world_objects/markers`).

### `bluerov_mission.launch.py`

| Argument  | Default | Description                                                       |
| --------- | ------- | ----------------------------------------------------------------- |
| `use_dvl` | `false` | Use `dvl_to_mavros.py`; `false` uses `ground_truth_to_mavros.py`  |

### `bluerov_square_bt.launch.py`

No launch arguments. Starts `locomotion_action_server`, `convert_to_controls_pose` (frames package, configured for `map` / `base_link`), and `bluerov_square_mission_tree` together. The simulation must already be running.

---

## Docker run flags

`./run.bash <image>` accepts these flags:

| Flag   | Effect                                          |
| ------ | ----------------------------------------------- |
| (none) | Default: NVIDIA + X11 + joystick + host network |
| `-c`   | Add CUDA library support                        |
| `-s`   | noVNC / TurboVNC image for CloudSim             |
| `-t`   | CI/headless mode (no X11)                       |
| `-x`   | VRX competition server base image               |
