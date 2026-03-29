from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the BlueROV2 square-mission behaviour tree.

    Starts three nodes:
      1. ground_truth_to_mavros  — broadcasts the map→base_link TF transform
      2. locomotion_action_server — bb_controls_msgs/Locomotion server on /bluerov/controls
      3. bluerov_square_mission_tree — py_trees behaviour tree (mission driver)

    Requires the simulation to already be running:
        ros2 launch bluerov_sim bluerov_sim.launch.py
    """
    ground_truth_node = Node(
        package="bluerov_sim",
        executable="ground_truth_to_mavros.py",
        name="ground_truth_to_mavros",
        output="screen",
    )

    action_server_node = Node(
        package="bluerov_sim",
        executable="locomotion_action_server.py",
        name="locomotion_action_server",
        output="screen",
    )

    mission_tree_node = Node(
        package="bluerov_sim",
        executable="bluerov_square_mission_tree.py",
        name="bluerov_square_mission_tree",
        output="screen",
    )

    return LaunchDescription(
        [
            action_server_node,
            mission_tree_node,
        ]
    )
