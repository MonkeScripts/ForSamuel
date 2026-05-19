"""BlueROV2 torpedo-task: cluster_tf action + service servers (under /bluerov).

Split out so cluster_tf can be restarted / log-isolated on its own pane (it
is the noisiest layer — it polls TFs at ~12 Hz and warns loudly while a
target frame like torpedo/yolo hasn't been published yet).

  - cluster_tf_action_server.py — async ClusterTfAction goal handler
  - cluster_tf_service_server.py — synchronous ClusterTfSrv handler

The BT (bluerov_torpedo_mission_tree) and bluerov_sim/torpedo/move_and_shoot_seq.py
both submit cluster goals/requests with out_parents="map" (see
bluerov_sim/shared_trees/search.py constants).
"""

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    # use_sim_time=True is critical: torpedo_pose_estimator stamps the
    # torpedo/yolo TF with the image header (Gazebo sim time, sec≈7000).
    # Without this flag the cluster_tf server captures `self.get_clock().now()`
    # as wall time (sec≈1.78e9) at service-start, then classifies every
    # incoming TF as "old" because its sim-time stamp is decades before the
    # wall-time start_time. Result: every TF is dropped, cluster spread =
    # 10000, BT never advances past search. (Same fix as the bin pane.)
    sim_time_param = {"use_sim_time": True}

    cluster_servers = GroupAction([
        PushRosNamespace("bluerov"),
        Node(
            package="bb_filters",
            executable="cluster_tf_action_server.py",
            name="cluster_tf_action_server",
            output="screen",
            parameters=[sim_time_param],
        ),
        Node(
            package="bb_filters",
            executable="cluster_tf_service_server.py",
            name="cluster_tf_service_server",
            output="screen",
            parameters=[sim_time_param],
        ),
    ])

    return LaunchDescription([cluster_servers])
