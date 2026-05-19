"""BlueROV2 torpedo-task: controls + TFs + actuators stack.

Same shape as bluerov_bin_controls.launch.py — the locomotion + service +
TF plane the torpedo BT needs:
  - bluerov_tfs.launch.py — static + grouped TFs for torpedo task frames
    (torpedo/centre, torpedo_{1,2}/{fish,shark}/view, etc.)
  - locomotion_action_server.py — /bluerov/controls Locomotion action server
  - convert_to_controls_pose.py — /bluerov/convert_to_controls_pose service
    (handles anchor_frame_name=torpedo_shooter_{left,right}_link by subtracting
    the URDF shooter offset before returning the goal)
  - actuators_stub.py — /bluerov/actuation/torpedo/{left,right} Trigger services

Cluster_tf, vision pipeline, and the BT each run in their own panes (see
bluerov_torpedo_cluster.launch.py, bluerov_torpedo_vision.launch.py, and
bluerov_torpedo_bt.launch.py respectively) so each layer can be restarted /
log-isolated on its own pane.

Prereq: launch bluerov_sim.launch.py world_name:=robosub_2025_pool in
another terminal first (the panel models live in that world).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bluerov_share = get_package_share_directory("bluerov_sim")

    tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bluerov_share, "launch", "bluerov_tfs.launch.py")
        )
    )

    locomotion_server = Node(
        package="bluerov_sim",
        executable="locomotion_action_server.py",
        name="locomotion_action_server",
        output="screen",
    )

    convert_to_controls_pose = Node(
        package="frames",
        executable="convert_to_controls_pose.py",
        name="convert_to_controls_pose",
        output="screen",
        parameters=[{
            "controls_frame": "map",
            "base_frame": "base_link",
            "odom_topic": "/mavros/odometry/out",
        }],
        remappings=[("convert_to_controls_pose", "/bluerov/convert_to_controls_pose")],
    )

    actuators = Node(
        package="bluerov_sim",
        executable="actuators_stub.py",
        name="actuators_stub",
        output="screen",
    )

    return LaunchDescription([
        tfs_launch,
        locomotion_server,
        convert_to_controls_pose,
        actuators,
    ])
