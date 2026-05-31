# Vision Smoke Test

Use this to check the bin and torpedo perception launch files without running a
full behavior tree.

## Build

```bash
cd /root/HOST/isaac_ros-dev
source /opt/ros/humble/setup.bash
colcon build --packages-select ardusub_interface bluerov_sim bluerov_tasks --symlink-install
source install/setup.bash
```

## Start Sim

```bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  world_name:=robosub_2025_pool \
  ardusub:=true \
  mavros:=true \
  gui:=true
```

Check cameras:

```bash
ros2 topic hz /bluerov/bottom_cam/image
ros2 topic hz /bluerov/front_cam/image
ros2 topic echo /bluerov/bottom_cam/camera_info --once
```

## Bin Vision

```bash
ros2 launch bluerov_tasks bluerov_bin_vision.launch.py
```

Activate YOLO:

```bash
ros2 service call /bluerov/bin/manage_nodes \
  lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /bluerov/bin/manage_nodes \
  lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
```

Check output:

```bash
ros2 topic hz /bluerov/bin/bin/yolo/detections
ros2 topic echo /bluerov/bin/bin/yolo/detections --once
```

## Torpedo Vision

```bash
ros2 launch bluerov_tasks bluerov_torpedo_vision.launch.py
```

Activate YOLO:

```bash
ros2 service call /bluerov/torpedo/manage_nodes \
  lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /bluerov/torpedo/manage_nodes \
  lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
```

Check output:

```bash
ros2 topic hz /bluerov/torpedo/torpedo/yolo/detections
ros2 topic hz /bluerov/torpedo/torpedo/hole/yolo/detections
```

## Quick Triage

```bash
ros2 topic list | grep yolo
ros2 topic list | grep image
ros2 service list | grep manage_nodes
```
