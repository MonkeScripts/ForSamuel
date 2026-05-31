"""BlueROV2 cluster_tf action + service servers (under /bluerov).

Shared by every task BT (bin, torpedo, …). Split out from the controls
launch so cluster_tf can be restarted / log-isolated on its own pane —
it is the noisiest layer (polls TFs at ~12 Hz and warns loudly until a
target frame like bin/yolo or torpedo/yolo is published).

  - cluster_tf_action_server.py — async ClusterTfAction goal handler
  - cluster_tf_service_server.py — synchronous ClusterTfSrv handler

Task BTs (and bluerov_tasks/torpedo/move_and_shoot_seq.py) submit cluster
goals/requests with out_parents="map" — see
bluerov_tasks/shared_trees/search.py constants.
"""

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    # use_sim_time=True is critical: per-task pose estimators (bin, torpedo)
    # stamp their /yolo TF with the image header (Gazebo sim time, sec≈7000).
    # Without this flag the cluster_tf server captures `self.get_clock().now()`
    # as wall time (sec≈1.78e9) at service-start, then classifies every
    # incoming TF as "old" because its sim-time stamp is decades before the
    # wall-time start_time. Result: every TF is dropped, cluster spread =
    # 10000, BT never advances past search.
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
