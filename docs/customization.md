# Extra Demo Commands

Use these after the workspace is built and sourced:

```bash
cd /root/HOST/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Manual Square Mission

Terminal 1:

```bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  world_name:=robosub_2025_pool \
  ardusub:=true \
  mavros:=true \
  gui:=true
```

Terminal 2:

```bash
ros2 launch bluerov_tasks bluerov_square_bt.launch.py
```

## Waypoint Mission Without A Behavior Tree

```bash
ros2 launch bluerov_tasks bluerov_mission.launch.py
```

This flies `(0,0,-2)` to `(5,0,-2)` to `(5,5,-2)` to `(0,5,-2)` and back.

## DVL Odometry Adapter

```bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  world_name:=robosub_2025_pool \
  ardusub:=true \
  mavros:=true \
  odom_source:=dvl
```

## Headless Sim

```bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  world_name:=robosub_2025_pool \
  headless:=true \
  gui:=false \
  ardusub:=true \
  mavros:=true
```

## Manual MAVROS Commands

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
```

## QGroundControl

QGroundControl runs on the host. If the vehicle does not appear automatically,
add a UDP link on port `14550`.
