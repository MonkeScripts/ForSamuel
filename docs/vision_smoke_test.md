# Vision pipeline smoke test

Verifies the YOLO + pose-estimator chain end-to-end without involving the
behaviour tree, locomotion server, or actuators. The success criterion is
"detection messages flow when the camera sees the target."

All commands run **inside the container** (`./run.bash bluerov_ws:humble`).

## 0. One-time: rebuild bluerov_sim so the two new launch files install

```bash
cd /root/HOST/bluerov_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bluerov_sim --symlink-install
source install/setup.bash
```

## 1. Pane A — simulation + ArduSub + MAVROS + ros_gz bridge

```bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
    world_name:=robosub_2025_pool ardusub:=true mavros:=true gui:=true
```

Wait until the Gazebo GUI shows the pool, bin, and two torpedo panels.

## 2. Pane B — sanity-check camera topics flow

```bash
ros2 topic hz /bluerov/bottom_cam/image          # ~30 Hz expected
ros2 topic hz /bluerov/front_cam/image           # ~30 Hz expected
ros2 topic echo /bluerov/bottom_cam/camera_info --once   # K matrix populated
```

If any of these don't publish, the gz_bridge or the SDF cameras are the
problem — fix that first, the rest of the test is downstream.

## 3a. Pane C — bin vision pipeline

```bash
ros2 launch bluerov_sim bluerov_bin_vision.launch.py
```

Expected log lines:
- `bin_yolo_node ... lifecycle state: unconfigured` (waiting for activation)
- `lifecycle_manager_node` ready

## 3b. Pane D — walk YOLO through configure -> activate

`lifecycle_manager_node` exposes `/bluerov/bin/manage_nodes`. Transition 1 =
configure (subscribers/publishers come up), transition 3 = activate (model
loads onto CUDA, image subscription begins).

```bash
ros2 service call /bluerov/bin/manage_nodes \
    lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /bluerov/bin/manage_nodes \
    lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
```

In Pane C you should now see:
- `[bin_yolo_node] Model loaded: /root/HOST/.../yolov11s_bin_20250813_1.pt`
- `[bin_yolo_node] Activated`

## 4. Verify detections topic

```bash
ros2 topic list | grep yolo                  # /bluerov/bin/bin/yolo/{detections,annotations}
ros2 topic hz   /bluerov/bin/bin/yolo/detections
ros2 topic echo /bluerov/bin/bin/yolo/detections --once
```

- ~10–30 Hz of DetectionArray messages — `detections` likely empty until the
  bottom camera actually sees the bin (BlueROV spawns at the origin; bin is
  at `(-6, 4.5, -2)`).

## 5. Fly BlueROV over the bin via QGC and re-check

In QGroundControl (host):
1. Connect to the BlueROV (auto via UDP 14550).
2. Switch to GUIDED, ARM, fly to roughly `lat/lon` of `(-6, 4.5)` in the local
   pool frame — or just use the joystick / "Fly" view to drag a waypoint over
   the bin.
3. Descend to `z ≈ -0.5` (i.e. ~1.5 m above the bin at -2 m depth) so the
   bottom camera looks down at it.

Then in Pane D:

```bash
ros2 topic echo /bluerov/bin/bin/yolo/detections --once
```

Expect at least one `Detection` entry with a class label matching the bin
classes the model was trained on.

## 6. Torpedo pipeline — same flow, front camera

```bash
# Pane C
ros2 launch bluerov_sim bluerov_torpedo_vision.launch.py

# Pane D
ros2 service call /bluerov/torpedo/manage_nodes \
    lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /bluerov/torpedo/manage_nodes \
    lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"

# Verify
ros2 topic hz /bluerov/torpedo/torpedo/yolo/detections
ros2 topic hz /bluerov/torpedo/torpedo/hole/yolo/detections
```

Fly the vehicle to roughly `(1, 0, -2)` facing +x — the front camera then
points at `torpedo_panel_v2` at `(3, 0, -2)`.

## Failure-mode triage

| Symptom | Likely cause |
| --- | --- |
| `ros2 topic hz /bluerov/bottom_cam/image` returns nothing | gz_bridge not running, or the camera plugin isn't loaded in the SDF for `robosub_2025_pool.world` |
| `manage_nodes` returns `success: false` on configure | YOLO node never came up — check the Pane C log |
| Model-load ERROR on activate | `.pt` path wrong, or CUDA/torch couldn't import — `python3 -c "import torch; torch.cuda.is_available()"` |
| `/bluerov/bin/bin/yolo/detections` stays at 0 Hz | YOLO subscribed to wrong image topic — verify `input_image_topic` in `bin.yaml` matches the bridge output |
| Detections topic publishes empty arrays only | Camera not looking at target, or model trained on visually different footage (expected risk — see plan §Risks #6) |
