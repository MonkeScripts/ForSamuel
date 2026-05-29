"""BlueROV2 bin vision-only smoke test.

Spins up just the perception side: YOLO lifecycle node + bin pose estimator +
lifecycle_manager + image_brighten + simple_matcher + points_pose_estimator.
No BT, no locomotion, no actuators — so this can be exercised against a
plain `bluerov_sim.launch.py` while flying via QGC.

Once everything is up, walk the YOLO node through configure -> activate:

  ros2 service call /bluerov/bin/manage_nodes \
      lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"   # configure
  ros2 service call /bluerov/bin/manage_nodes \
      lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"   # activate

Then verify:

  ros2 topic hz /bluerov/bin/bin/yolo/detections
  ros2 topic echo /bluerov/bin/bin/yolo/annotations --once
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    cfg = os.path.join(
        get_package_share_directory("bluerov_sim"),
        "config", "vision_pipeline", "bin.yaml",
    )

    vision = GroupAction([
        PushRosNamespace("/bluerov/bin"),
        LifecycleNode(
            package="yolo_ros_trt",
            executable="yolo_node",
            name="bin_yolo_node",
            namespace="",
            parameters=[cfg],
        ),
        Node(
            package="pose_estimator",
            executable="bin_pose_estimator_node",
            name="bin_pose_estimator_node",
            parameters=[cfg],
        ),
        Node(
            package="vision_pipeline",
            executable="lifecycle_manager_node",
            name="lifecycle_manager_node",
            parameters=[cfg],
        ),
        Node(
            package="image_processing",
            executable="image_brighten_node",
            name="image_brighten_node",
            parameters=[cfg],
        ),
        Node(
            package="image_matching",
            executable="simple_matcher_node",
            name="simple_matcher_node",
            parameters=[cfg],
        ),
        Node(
            package="pose_estimator",
            executable="points_pose_estimator_node",
            name="points_pose_estimator_node",
            parameters=[cfg],
        ),
    ])

    return LaunchDescription([vision])
