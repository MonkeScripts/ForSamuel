"""BlueROV2 bin-task: controls + TFs + actuators stack.

The locomotion/control plane the bin BT needs:
  - bluerov_tfs.launch.py — static TFs for task frames
  - locomotion_action_server.py — /bluerov/controls Locomotion action server
  - convert_to_controls_pose.py — /bluerov/convert_to_controls_pose service
  - actuators_stub.py — /bluerov/actuation/dropper Trigger service

Cluster_tf, vision pipeline, and the BT each run in their own panes (see
bluerov_bin_cluster.launch.py, bluerov_bin_vision.launch.py, and
bluerov_bin_bt.launch.py respectively) so each layer can be restarted /
log-isolated on its own pane.

Prereq: launch bluerov_sim.launch.py in another terminal first.
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
