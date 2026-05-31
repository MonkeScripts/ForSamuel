from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    movement_node = Node(
        package="bluerov_tasks",
        executable="bluerov_movement.py",
        name="bluerov_movement",
        output="screen",
    )

    return LaunchDescription(
        [
            movement_node,
        ]
    )
