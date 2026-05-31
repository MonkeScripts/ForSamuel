"""Forked from mission_planner_2.vehicles.shared.trees.search for BlueROV2.

Original kept hardcoded `auv4/base_link_ned` (module constant) and
`/auv4/cluster_tfs_srv` (service path literal) embedded in every search
factory. Both are now keyword-arg-parameterised with BlueROV defaults
so the bin/torpedo trees can use them as-is.

The goto.* objects come from scripts/goto.py (BlueROV-routed wrappers over
the upstream auv goto), so `anchor_frame_name=base_link_frame` is the
BlueROV default and would no longer need an explicit override — kept for
clarity at call sites.
"""

import os
import sys
from typing import List

import numpy as np
import py_trees
import py_trees_ros
from ament_index_python.packages import get_package_prefix
from bb_perception_msgs.srv import ClusterTfSrv
from geometry_msgs.msg import PoseStamped
from py_trees_ros.subscribers import operator
from rclpy.qos import qos_profile_sensor_data

from mission_planner_2.common.util.pose_utils import (
    create_clustering_request,
    create_stamped_pose,
)
from mission_planner_2.vehicles.shared.trees.blackboard import DynamicSetBlackboard

# scripts/goto.py is installed to lib/bluerov_tasks/ by ament_cmake `install(PROGRAMS)`,
# not inside this Python package. Reach it via the ament prefix instead of a
# brittle relative path. Same idiom as bluerov_tasks/bins/bins.py.
_BLUEROV_SCRIPTS_DIR = os.path.join(
    get_package_prefix("bluerov_tasks"), "lib", "bluerov_tasks"
)
if _BLUEROV_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _BLUEROV_SCRIPTS_DIR)
import goto  # noqa: E402  (bluerov_tasks/scripts/goto.py)

DEFAULT_BASE_LINK_FRAME = "base_link"
DEFAULT_CLUSTER_SRV_NAME = "/bluerov/cluster_tfs_srv"
# Upstream defaults out_parents="world_ned" (AUV4 NED world). Our BlueROV stack
# uses map/ENU (see ardusub_interface/scripts/ground_truth_to_mavros.py).
# Threaded into every create_clustering_request call below.
DEFAULT_OUT_PARENTS_FRAME = "map"


def _to_top_left(f: float, l: float, start_xy: np.ndarray) -> np.ndarray:
    return np.array(
        [
            -start_xy[0] + f,
            -start_xy[1] - l,
        ],
        dtype=float,
    )


def _gen_square(
    f: float, b: float, l: float, r: float, start_xy: np.ndarray
) -> np.ndarray:
    top_left = _to_top_left(f, l, start_xy)
    btm_left = np.array((-f - b, 0))
    btm_right = np.array((0, l + r))
    top_right = np.array((f + b, 0))
    return np.array(
        [
            top_left,
            btm_left,
            btm_right,
            top_right,
        ],
        dtype=float,
    )


def _generate_layered_square_search_bot_pattern(
    fwd: float,
    back: float,
    left: float,
    right: float,
    num_squares: int,
    base_link_frame: str,
    offset_coeff: float = 0.2,
) -> list:
    output_points = []
    end = np.zeros_like((2, 1), dtype=float)
    for i in range(0, num_squares):
        offset = i * offset_coeff
        points = _gen_square(
            fwd + offset, back + offset, left + offset, right + offset, end
        )
        output_points.append(points)
        end = np.sum(points, axis=0) + end

    output_points = np.concatenate(output_points, axis=0)
    return [
        create_stamped_pose(base_link_frame, position_x=point[0], position_y=point[1])
        for point in output_points
    ]


def _generate_square(fwd, back, left, right, base_link_frame: str):
    return [
        create_stamped_pose(base_link_frame, position_x=fwd, position_y=-left),
        create_stamped_pose(base_link_frame, position_x=-(fwd + back)),
        create_stamped_pose(base_link_frame, position_y=left + right),
        create_stamped_pose(base_link_frame, position_x=fwd + back),
    ]


def _create_search_bot_root(
    poses: List[PoseStamped],
    cluster_node_start,
    cluster_node_end,
    base_link_frame: str,
    wait_between_moves_sec: float = 5.0,
    search_depth: float = 0.3,
):
    root = py_trees.composites.Sequence(
        name="Search seq (bot cam)",
        memory=True,
    )

    goto_search_pattern = goto.NFromConstant(
        name="Goto search pattern",
        poses=poses,
        wait_between_moves_sec=wait_between_moves_sec,
        specified_heading=True,
        depth_override_value=search_depth,
        anchor_frame_name=base_link_frame,
    )

    root.add_children(
        [
            cluster_node_start,
            goto_search_pattern,
            cluster_node_end,
        ]
    )

    return root


def create_search_bot_constant_root(
    fwd: float,
    back: float,
    left: float,
    right: float,
    object_frame: str,
    object_frame_clustered: str,
    wait_between_moves: float = 5.0,
    search_depth: float = 0.3,
    base_link_frame: str = DEFAULT_BASE_LINK_FRAME,
    cluster_srv_name: str = DEFAULT_CLUSTER_SRV_NAME,
):
    poses = _generate_square(fwd, back, left, right, base_link_frame)

    cluster_node_start = py_trees_ros.service_clients.FromConstant(
        name="Cluster search",
        service_type=ClusterTfSrv,
        service_name=cluster_srv_name,
        service_request=create_clustering_request(
            enabled=True,
            in_children=object_frame,
            out_children=object_frame_clustered,
            out_parents=DEFAULT_OUT_PARENTS_FRAME,
            persistent=False,
        ),
    )

    cluster_node_stop = py_trees_ros.service_clients.FromConstant(
        name="Cluster search stop",
        service_type=ClusterTfSrv,
        service_name=cluster_srv_name,
        service_request=create_clustering_request(
            enabled=False,
            persistent=False,
            in_children=object_frame,
            out_children=object_frame_clustered,
            out_parents=DEFAULT_OUT_PARENTS_FRAME,
        ),
    )

    return _create_search_bot_root(
        poses,
        cluster_node_start=cluster_node_start,
        cluster_node_end=cluster_node_stop,
        wait_between_moves_sec=wait_between_moves,
        search_depth=search_depth,
        base_link_frame=base_link_frame,
    )


def create_homing_search_bot_layered_square_root(
    fwd: float,
    back: float,
    left: float,
    right: float,
    num_squares: int,
    object_frame: str,
    cluster_dist_threshold: float,
    object_frame_clustered: str,
    check_topic: str,
    check_topic_type,
    offset_coeff: float = 0.2,
    wait_between_moves: float = 5.0,
    search_depth: float = 0.3,
    min_cluster_size: int = 2,
    base_link_frame: str = DEFAULT_BASE_LINK_FRAME,
    cluster_srv_name: str = DEFAULT_CLUSTER_SRV_NAME,
):
    cluster_key = "homing_bot_layered_cluster_resp"
    check_topic_sub_key = f"{check_topic}_message"
    poses = _generate_layered_square_search_bot_pattern(
        fwd, back, left, right, num_squares, base_link_frame, offset_coeff
    )

    root = py_trees.composites.Sequence(
        name="Seq homing search with early stop",
        memory=True,
    )

    def cluster_node_start_func(persistent: bool):
        return py_trees_ros.service_clients.FromConstant(
            name="Cluster search",
            service_type=ClusterTfSrv,
            service_name=cluster_srv_name,
            service_request=create_clustering_request(
                enabled=True,
                in_children=object_frame,
                out_children=object_frame_clustered,
                out_parents=DEFAULT_OUT_PARENTS_FRAME,
                persistent=persistent,
                min_cluster_size=min_cluster_size,
            ),
        )

    def cluster_node_stop_func(persistent: bool):
        return py_trees_ros.service_clients.FromConstant(
            name="Cluster search stop",
            service_type=ClusterTfSrv,
            service_name=cluster_srv_name,
            service_request=create_clustering_request(
                enabled=False,
                persistent=persistent,
                in_children=object_frame,
                out_children=object_frame_clustered,
                out_parents=DEFAULT_OUT_PARENTS_FRAME,
            ),
            key_response=cluster_key,
        )

    srv_start_cluster = cluster_node_start_func(persistent=False)

    par_search_check = py_trees.composites.Parallel(
        name="Search layered - check par",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )

    goto_search_pattern = goto.NFromConstant(
        name="Goto search pattern",
        poses=poses,
        wait_between_moves_sec=wait_between_moves,
        specified_heading=True,
        depth_override_value=search_depth,
        anchor_frame_name=base_link_frame,
    )

    seq_check_seen = py_trees.composites.Sequence(
        name="Seq sub and check seen something",
        memory=False,
    )

    sub_check_topic = py_trees_ros.subscribers.ToBlackboard(
        name=f"Sub {check_topic}",
        topic_name=check_topic,
        topic_type=check_topic_type,
        qos_profile=qos_profile_sensor_data,
        blackboard_variables={check_topic_sub_key: "data"},
    )

    check_check_ok = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check early stopping",
        check=py_trees.common.ComparisonExpression(
            variable=check_topic_sub_key,
            value=True,
            operator=operator.eq,
        ),
    )

    goto_stationkeep = goto.FromConstant(
        name="goto stationkeep",
        pose=create_stamped_pose(frame_id=base_link_frame),
        depth_override_value=search_depth,
        anchor_frame_name=base_link_frame,
    )

    seq_check_seen.add_children([sub_check_topic, check_check_ok])

    always_running = py_trees.decorators.FailureIsRunning(
        name="Keep running on failure check",
        child=seq_check_seen,
    )

    par_search_check.add_children([goto_search_pattern, always_running])

    seq_stop_search = py_trees.composites.Sequence(
        name="Seq check stop search",
        memory=True,
    )
    srv_stop_cluster = cluster_node_stop_func(persistent=False)
    seq_stop_search.add_children([srv_stop_cluster])

    root.add_children([par_search_check, goto_stationkeep])

    return py_trees.decorators.Retry(
        name="Retry homing search",
        child=root,
        num_failures=10000,
    )


def create_search_bot_layered_square_root(
    fwd: float,
    back: float,
    left: float,
    right: float,
    num_squares: int,
    object_frame: str,
    cluster_dist_threshold: float,
    object_frame_clustered: str,
    offset_coeff: float = 0.2,
    wait_between_moves: float = 5.0,
    search_depth: float = 0.3,
    min_cluster_size: int = 2,
    base_link_frame: str = DEFAULT_BASE_LINK_FRAME,
    cluster_srv_name: str = DEFAULT_CLUSTER_SRV_NAME,
):
    cluster_key = "bot_layered_cluster_resp"

    poses = _generate_layered_square_search_bot_pattern(
        fwd, back, left, right, num_squares, base_link_frame, offset_coeff
    )

    def cluster_node_start_func(persistent: bool):
        return py_trees_ros.service_clients.FromConstant(
            name="Cluster search",
            service_type=ClusterTfSrv,
            service_name=cluster_srv_name,
            service_request=create_clustering_request(
                enabled=True,
                in_children=object_frame,
                out_children=object_frame_clustered,
                out_parents=DEFAULT_OUT_PARENTS_FRAME,
                persistent=persistent,
                min_cluster_size=min_cluster_size,
            ),
        )

    def cluster_node_stop_func(persistent: bool):
        return py_trees_ros.service_clients.FromConstant(
            name="Cluster search stop",
            service_type=ClusterTfSrv,
            service_name=cluster_srv_name,
            service_request=create_clustering_request(
                enabled=False,
                persistent=persistent,
                in_children=object_frame,
                out_children=object_frame_clustered,
                out_parents=DEFAULT_OUT_PARENTS_FRAME,
            ),
            key_response=cluster_key,
        )

    def cluster_validate_seq(
        poses: List[PoseStamped],
        cluster_dist_threshold: float,
    ):
        root = py_trees.composites.Sequence(
            name="Seq bot search and validate",
            memory=True,
        )

        seq_search_bot = _create_search_bot_root(
            poses=poses,
            cluster_node_start=cluster_node_start_func(True),
            cluster_node_end=cluster_node_stop_func(True),
            wait_between_moves_sec=wait_between_moves,
            search_depth=search_depth,
            base_link_frame=base_link_frame,
        )

        check_valid_cluster = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Check valid cluster",
            check=py_trees.common.ComparisonExpression(
                variable=cluster_key,
                value=cluster_dist_threshold,
                operator=lambda x, y: x.cluster_spread < y or num_squares == 1,
            ),
        )

        root.add_children([seq_search_bot, check_valid_cluster])

        return root

    root = py_trees.composites.Selector(
        name="Search seq (bot cam) with layers",
        memory=True,
    )

    root.add_children(
        [
            cluster_validate_seq(
                poses=poses[layer_num * 4 : layer_num * 4 + 4],
                cluster_dist_threshold=cluster_dist_threshold,
            )
            for layer_num in range(len(poses) // 4)
        ]
    )

    return root


def create_search_bot_bb_root(
    fwd: float,
    back: float,
    left: float,
    right: float,
    object_frame_key: str,
    object_frame_clustered_key: str,
    wait_between_moves: float = 5.0,
    search_depth: float = 0.3,
    base_link_frame: str = DEFAULT_BASE_LINK_FRAME,
    cluster_srv_name: str = DEFAULT_CLUSTER_SRV_NAME,
):
    poses = _generate_square(fwd, back, left, right, base_link_frame)
    enable_request_key = "/bot_search/service_request"
    disable_request_key = "/bot_search/service_request_stop"

    root = py_trees.composites.Sequence(
        name="Search seq (bot cam)",
        memory=True,
    )

    dynamic_set_start_req = DynamicSetBlackboard(
        name="Dynamic set cluster goal",
        key=[object_frame_key, object_frame_clustered_key],
        update_key=enable_request_key,
        overwrite=True,
        func=lambda frame, frame_clustered: create_clustering_request(
            enabled=True,
            in_children=frame,
            out_children=frame_clustered,
            out_parents=DEFAULT_OUT_PARENTS_FRAME,
            persistent=False,
        ),
    )

    dynamic_set_end_req = DynamicSetBlackboard(
        name="Dynamic set cluster stop",
        key=[object_frame_key, object_frame_clustered_key],
        update_key=disable_request_key,
        overwrite=True,
        func=lambda frame, frame_clustered: create_clustering_request(
            enabled=False,
            persistent=False,
            in_children=frame,
            out_children=frame_clustered,
            out_parents=DEFAULT_OUT_PARENTS_FRAME,
        ),
    )

    cluster_node_start = py_trees_ros.service_clients.FromBlackboard(
        name="Cluster search",
        service_type=ClusterTfSrv,
        service_name=cluster_srv_name,
        key_request=enable_request_key,
    )

    cluster_node_end = py_trees_ros.service_clients.FromBlackboard(
        name="Cluster search stop",
        service_type=ClusterTfSrv,
        service_name=cluster_srv_name,
        key_request=disable_request_key,
    )

    seq_search = _create_search_bot_root(
        poses,
        cluster_node_start=cluster_node_start,
        cluster_node_end=cluster_node_end,
        wait_between_moves_sec=wait_between_moves,
        search_depth=search_depth,
        base_link_frame=base_link_frame,
    )

    root.add_children([dynamic_set_start_req, dynamic_set_end_req, seq_search])

    return root


def _gen_yaw_points(
    max_left: float,
    max_right: float,
    s: float,
    base_link_frame: str,
) -> List[PoseStamped]:
    num_points_l = int(max_left // s)
    num_points_r = int(max_right // s)
    points = [
        create_stamped_pose(base_link_frame, yaw=s) for _ in range(num_points_r)
    ]

    offset = (num_points_r + 1) * s
    points.append(create_stamped_pose(base_link_frame, yaw=-offset - s))
    for _ in range(num_points_l - 1):
        points.append(create_stamped_pose(base_link_frame, yaw=-s))

    return points


def create_search_front_root(
    object_frame: str,
    object_frame_clustered: str,
    yaw_left_deg: float = 30.0,
    yaw_right_deg: float = 30.0,
    step: float = 15.0,
    wait_between_moves: float = 5.0,
    search_depth: float = 0.3,
    base_link_frame: str = DEFAULT_BASE_LINK_FRAME,
    cluster_srv_name: str = DEFAULT_CLUSTER_SRV_NAME,
):
    root = py_trees.composites.Sequence(
        name="Search seq (front cam)",
        memory=True,
    )

    points = _gen_yaw_points(
        max_left=yaw_left_deg,
        max_right=yaw_right_deg,
        s=step,
        base_link_frame=base_link_frame,
    )

    goto_yaw = goto.NFromConstant(
        name="Goto search pattern",
        poses=points,
        wait_between_moves_sec=wait_between_moves,
        is_relative_movement=True,
        depth_override_value=search_depth,
        anchor_frame_name=base_link_frame,
    )

    cluster_node_start = py_trees_ros.service_clients.FromConstant(
        name="Cluster search",
        service_type=ClusterTfSrv,
        service_name=cluster_srv_name,
        service_request=create_clustering_request(
            enabled=True,
            in_children=object_frame,
            out_children=object_frame_clustered,
            out_parents=DEFAULT_OUT_PARENTS_FRAME,
            persistent=False,
        ),
    )

    cluster_node_stop = py_trees_ros.service_clients.FromConstant(
        name="Cluster search stop",
        service_type=ClusterTfSrv,
        service_name=cluster_srv_name,
        service_request=create_clustering_request(
            enabled=False,
            persistent=False,
            in_children=object_frame,
            out_children=object_frame_clustered,
            out_parents=DEFAULT_OUT_PARENTS_FRAME,
        ),
    )

    root.add_children([cluster_node_start, goto_yaw, cluster_node_stop])

    return root
