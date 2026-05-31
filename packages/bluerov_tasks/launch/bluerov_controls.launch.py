"""BlueROV2 controls + TFs + actuators stack (shared by bin and torpedo).

The locomotion / service / TF plane that every task BT needs:
  - bluerov_tfs.launch.py — static + grouped TFs for task frames
    (bin/centre, torpedo/centre, torpedo_{1,2}/{fish,shark}/view, …;
    convert_to_controls_pose handles anchor_frame_name=torpedo_shooter_*_link
    by subtracting the URDF shooter offset before returning the goal)
  - locomotion_action_server.py — /bluerov/controls Locomotion action server
  - convert_to_controls_pose.py — /bluerov/convert_to_controls_pose service
  - actuators_stub.py — /bluerov/actuation/{dropper,torpedo/left,torpedo/right}
    Trigger services (task-agnostic; the BT picks the one it needs)

Cluster_tf, vision pipeline, and the BT each run in their own panes (see
bluerov_cluster.launch.py and the per-task vision/bt launches) so each
layer can be restarted / log-isolated on its own pane.

Prereq: launch bluerov_sim.launch.py with the matching world in another
terminal first (robosub_2025_pool for bin/torpedo).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bluerov_share = get_package_share_directory("bluerov_tasks")

    tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bluerov_share, "launch", "bluerov_tfs.launch.py")
        )
    )

    locomotion_server = Node(
        package="bluerov_tasks",
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
        package="bluerov_tasks",
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
