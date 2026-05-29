"""BlueROV2 torpedo-task: behaviour tree only.

Just the py_trees mission tree (bluerov_torpedo_mission_tree.py). Run after
bluerov_sim.launch.py, bluerov_controls.launch.py,
bluerov_cluster.launch.py, and bluerov_torpedo_vision.launch.py
are up.

Split out of the old monolithic bluerov_torpedo.launch.py so the BT pane
shows only py_trees console output — easier to read tick-by-tick.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bt = Node(
        package="bluerov_sim",
        executable="bluerov_torpedo_mission_tree.py",
        name="bluerov_torpedo_mission_tree",
        output="screen",
    )

    return LaunchDescription([bt])
