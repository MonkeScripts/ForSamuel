import os
import sys

import numpy as np
import py_trees
import py_trees_ros
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from tf_transformations import euler_from_quaternion

from mission_planner_2.common.core import shared_action_client
from mission_planner_2.common.util.namespace_utils import full_key_generator
from mission_planner_2.common.util.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
    within_threshold_rpy,
    within_threshold_xyz,
)
from bluerov_sim.node_registry import BlueROVSharedAction
from mission_planner_2.vehicles.shared.trees.blackboard import DynamicSetBlackboard
from mission_planner_2.vehicles.shared.trees.cluster_goto import (
    create_goto_cluster_from_bb_root,
)
from mission_planner_2.vehicles.shared.trees.tf_checker import (
    create_tf_checker_from_bb_root,
    create_tf_checker_from_constant_root,
)

# Pull the BlueROV-routed goto wrappers from scripts/goto.py — they live in
# the installed `lib/bluerov_sim/` directory (ament_cmake `install(PROGRAMS …)`),
# not in this package, so import via the ament-resolved prefix. Same idiom as
# bluerov_sim/bins/bins.py and bluerov_sim/shared_trees/search.py.
_BLUEROV_SCRIPTS_DIR = os.path.join(
    get_package_prefix("bluerov_sim"), "lib", "bluerov_sim"
)
if _BLUEROV_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _BLUEROV_SCRIPTS_DIR)
import goto  # noqa: E402  (bluerov_sim/scripts/goto.py)

# Upstream uses generate_namespace() which walks the caller path for `/trees/`
# — this file isn't under any `/trees/`, so hardcode the namespace the path
# WOULD have produced in mission_planner_2. Same workaround as bins.py.
NAMESPACE = "/bluerov/torpedo/move_and_shoot"
fk = full_key_generator(NAMESPACE)

_CLUSTERING_GOAL_KEY = fk("clustering_goal")
_CLUSTERING_GOAL_CHECK_KEY = fk("clustering_goal_check")

BASE_LINK_FRAME = "base_link"
# Upstream defaults clustering's out_parents to "world_ned" (AUV4 NED world).
# Our BlueROV stack uses map/ENU — see bluerov_sim/shared_trees/search.py.
OUT_PARENTS_FRAME = "map"


def create_firing_root(
    actuation_topic: str,
    torp_string: str,
    shoot_repeats: int,
    wait_after_fire_duration: float,
):
    root = py_trees.composites.Sequence(
        name="Repeated firing sequence",
        memory=True,
    )

    fire = py_trees_ros.service_clients.FromConstant(
        name=f"Fire {torp_string} torpedo",
        service_type=Trigger,
        service_name=actuation_topic,
        service_request=Trigger.Request(),
    )

    repeat_firing = py_trees.decorators.Repeat(
        name=f"Fire {torp_string} repeats: {shoot_repeats}",
        child=fire,
        num_success=shoot_repeats,
    )

    wait_after_fire = py_trees.timers.Timer(
        name="Wait after fire",
        duration=wait_after_fire_duration,
    )

    root.add_children(
        [
            repeat_firing,
            wait_after_fire,
        ]
    )

    return root


def create_move_and_shoot_generator(
    anchor_frame_key: str,
    torpedo_shooter_left_frame: str,
    torpedo_shooter_right_frame: str,
    choice_key: str,
    pose_key: str,
    pose_frame_key: str,
    fish_shoot_frame_key: str,
    shark_shoot_frame_key: str,
    template_frame_optical_key: str,
    template_frame_optical_clustered_key: str,
    cluster_duration: int,
    realign_cluster_duration: int,
    actuation_topic_left: str,
    actuation_topic_right: str,
    world_to_torp_yaw: float,
    zero_yaw_key: str,
    distance_threshold=0.05,
    yaw_threshold=3.0,
    retries=3,
    stabilization_duration=2.5,
    num_retries_clustering=3,
    wait_after_fire_duration: float = 3.0,
    shoot_repeats: int = 2,
):
    def f(first=True):
        if first:
            anchor_frame = torpedo_shooter_left_frame
            shoot_pose_sel = (
                lambda choice, fish_shoot_frame, shark_shoot_frame: create_stamped_pose(
                    fish_shoot_frame if choice.success else shark_shoot_frame
                )
            )
            shoot_frame_sel = lambda choice, fish_shoot_frame, shark_shoot_frame: (
                fish_shoot_frame if choice.success else shark_shoot_frame
            )
            actuation_topic = actuation_topic_left
            torp_string = "first"
        else:
            anchor_frame = torpedo_shooter_right_frame
            shoot_pose_sel = (
                lambda choice, fish_shoot_frame, shark_shoot_frame: create_stamped_pose(
                    shark_shoot_frame if choice.success else fish_shoot_frame
                )
            )
            shoot_frame_sel = lambda choice, fish_shoot_frame, shark_shoot_frame: (
                shark_shoot_frame if choice.success else fish_shoot_frame
            )
            actuation_topic = actuation_topic_right
            torp_string = "second"

        set_anchor_frame = py_trees.behaviours.SetBlackboardVariable(
            name="Set anchor frame",
            variable_name=anchor_frame_key,
            variable_value=anchor_frame,
            overwrite=True,
        )

        dynamic_set_pose = DynamicSetBlackboard(
            name="select torpedo pose",
            key=[choice_key, fish_shoot_frame_key, shark_shoot_frame_key],
            update_key=pose_key,
            overwrite=True,
            func=shoot_pose_sel,
        )

        dynamic_set_frame = DynamicSetBlackboard(
            name="select torpedo frame",
            key=[choice_key, fish_shoot_frame_key, shark_shoot_frame_key],
            update_key=pose_frame_key,
            overwrite=True,
            func=shoot_frame_sel,
        )

        dynamic_set_cluster_goal = DynamicSetBlackboard(
            name="set clustering goal",
            key=[template_frame_optical_key, template_frame_optical_clustered_key],
            update_key=_CLUSTERING_GOAL_KEY,
            overwrite=True,
            func=lambda frame, clustered: create_clustering_goal(
                in_children=frame,
                out_children=clustered,
                out_parents=OUT_PARENTS_FRAME,
                duration=cluster_duration,
                use_cache=False,
            ),
        )

        dynamic_set_cluster_goal_check = DynamicSetBlackboard(
            name="set clustering goal check",
            key=[template_frame_optical_key, template_frame_optical_clustered_key],
            update_key=_CLUSTERING_GOAL_CHECK_KEY,
            overwrite=True,
            func=lambda frame, clustered: create_clustering_goal(
                in_children=frame,
                out_children=clustered,
                out_parents=OUT_PARENTS_FRAME,
                duration=realign_cluster_duration,
                use_cache=False,
            ),
        )

        cluster_node = shared_action_client.FromBlackboard(
            name=f"Cluster the transforms before {torp_string} shot",
            shared_action=BlueROVSharedAction.CLUSTER,
            key=_CLUSTERING_GOAL_KEY,
        )

        retry_cluster_node = py_trees.decorators.Retry(
            name="Retry cluster node",
            child=cluster_node,
            num_failures=num_retries_clustering,
        )

        cluster_node_check = shared_action_client.FromBlackboard(
            name=f"Cluster the transforms before {torp_string} shot",
            shared_action=BlueROVSharedAction.CLUSTER,
            key=_CLUSTERING_GOAL_CHECK_KEY,
        )

        retry_cluster_node_check = py_trees.decorators.Retry(
            name="Retry cluster node check",
            child=cluster_node_check,
            num_failures=num_retries_clustering,
        )

        goto_target = goto.FromBlackboard(
            name=f"Go to {torp_string} target",
            pose_key=pose_key,
            anchor_frame_name=anchor_frame,
        )

        goto_cluster = py_trees.decorators.FailureIsSuccess(
            name=f"Cluster and goto {torp_string}",
            child=create_goto_cluster_from_bb_root(
                cluster_node=retry_cluster_node,
                cluster_node_check=retry_cluster_node_check,
                goto_node=goto_target,
                start_frame_keys=[anchor_frame_key, "/global/base_link"],
                retries=retries,
                goto_pose_frame_key=pose_frame_key,
                within_threshold_list=[
                    within_threshold_xyz(distance_threshold),
                    within_threshold_rpy(yaw_threshold),
                ],
                stabilization_duration=stabilization_duration,
            ),
            # child=create_goto_cluster_from_bb_tf_tf_root(
            #     cluster_node=cluster_node_first,
            #     cluster_node_check=cluster_node_check_first,
            #     goto_node=goto_target_first,
            #     distance_threshold=0.05,
            #     retries=3,
            #     tf_frame_key=_POSE_FRAME_KEY,
            #     within_threshold=within_threshold_dist,
            #     stabilization_duration=2.5,
            # ),
        )

        seq_repeated_fire = create_firing_root(
            actuation_topic=actuation_topic,
            torp_string=torp_string,
            shoot_repeats=shoot_repeats,
            wait_after_fire_duration=wait_after_fire_duration,
        )

        seq_relocalise = py_trees.composites.Sequence(
            name="Relocalize to torpedo yaw",
            memory=True,
        )

        _odom_tf_key = fk("odom_tf")
        _base_link_to_torp_view_frame_key = fk("bl_to_torp_view_frame")

        get_odom = create_tf_checker_from_constant_root(
            start_frames=[OUT_PARENTS_FRAME],
            end_frames=[BASE_LINK_FRAME],
            update_keys=[_odom_tf_key],
            fallback_val=[None],
        )

        lookup_base_link_to_torp_view = create_tf_checker_from_bb_root(
            start_frame_keys=["/global/base_link"],
            end_frame_keys=[pose_frame_key],
            update_keys=[_base_link_to_torp_view_frame_key],
            fallback_val=[None],
        )

        save_zeroed_yaw = DynamicSetBlackboard(
            name="Create zeroed yaw pose",
            key=[
                _odom_tf_key,
                _base_link_to_torp_view_frame_key,
            ],
            update_key=zero_yaw_key,
            overwrite=True,
            func=lambda odom_tf, torp_view_tf: get_torp_relocalised_yaw(
                odom_tf,
                torp_view_tf,
                world_to_torp_yaw,
            ),
        )

        seq_relocalise.add_children(
            [
                get_odom,
                lookup_base_link_to_torp_view,
                save_zeroed_yaw,
            ]
        )

        force_success_seq_relocalise = py_trees.decorators.FailureIsSuccess(
            name="Force succeed relocalise",
            child=seq_relocalise,
        )

        children = [
            set_anchor_frame,
            dynamic_set_pose,
            dynamic_set_frame,
            dynamic_set_cluster_goal,
            dynamic_set_cluster_goal_check,
            goto_cluster,
            seq_repeated_fire,
        ]

        if not first:
            children.append(force_success_seq_relocalise)

        root = py_trees.composites.Sequence(
            f"Move and shoot {torp_string} torpedo",
            memory=True,
            children=children,
        )

        return root

    return f


def get_torp_relocalised_yaw(
    odom_tf: TransformStamped | None,
    torp_view_tf: TransformStamped | None,
    torp_offset: float,
):
    if odom_tf is None or torp_view_tf is None:
        return 0.0

    torp_offset_rad = np.deg2rad(torp_offset)

    _, _, odom_y = euler_from_quaternion(
        [
            odom_tf.transform.rotation.x,
            odom_tf.transform.rotation.y,
            odom_tf.transform.rotation.z,
            odom_tf.transform.rotation.w,
        ]
    )

    _, _, torp_y = euler_from_quaternion(
        [
            torp_view_tf.transform.rotation.x,
            torp_view_tf.transform.rotation.y,
            torp_view_tf.transform.rotation.z,
            torp_view_tf.transform.rotation.w,
        ]
    )

    final = (odom_y + torp_y - torp_offset_rad) % (2 * np.pi)

    return final
