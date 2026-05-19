"""BlueROV2 bin-task: behaviour tree only.

Just the py_trees mission tree (bluerov_bin_mission_tree.py). Run after
bluerov_sim.launch.py, bluerov_bin_controls.launch.py,
bluerov_bin_cluster.launch.py, and bluerov_bin_vision.launch.py are up.

Split out of bluerov_bin.launch.py so the BT pane shows only py_trees
console output — easier to read tick-by-tick.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bt = Node(
        package="bluerov_sim",
        executable="bluerov_bin_mission_tree.py",
        name="bluerov_bin_mission_tree",
        output="screen",
    )

    return LaunchDescription([bt])
