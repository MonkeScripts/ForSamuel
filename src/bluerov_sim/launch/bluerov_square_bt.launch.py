from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the BlueROV2 square-mission behaviour tree.

    Starts four nodes:
      1. ground_truth_to_mavros  — broadcasts the map→base_link TF transform
      2. locomotion_action_server — bb_controls_msgs/Locomotion server on /bluerov/controls
      3. convert_to_controls_pose — frames package service for base_link→map conversion
      4. bluerov_square_mission_tree — py_trees behaviour tree (mission driver)

    Requires the simulation to already be running:
        ros2 launch bluerov_sim bluerov_sim.launch.py
    """
    action_server_node = Node(
        package="bluerov_sim",
        executable="locomotion_action_server.py",
        name="locomotion_action_server",
        output="screen",
    )

    convert_to_controls_pose_node = Node(
        package="frames",
        executable="convert_to_controls_pose.py",
        name="convert_to_controls_pose",
        output="screen",
        parameters=[{
            "controls_frame": "map",
            "base_frame": "base_link",
            "odom_topic": "/mavros/odometry/out",
        }],
        remappings=[
            ("convert_to_controls_pose", "/bluerov/convert_to_controls_pose"),
        ],
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
            convert_to_controls_pose_node,
            mission_tree_node,
        ]
    )
