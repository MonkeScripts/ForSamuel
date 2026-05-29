import operator

import py_trees

from mission_planner_2.common.util.detection_utils import create_img_matching_request
from mission_planner_2.common.util.pose_utils import create_clustering_goal
from mission_planner_2.vehicles.auv.trees.robosub24.bins.choice_selector import (
    create_choice_selector_root,
)
from mission_planner_2.vehicles.shared.trees.blackboard import DynamicSetBlackboard


def create_template_selector_root(
    points_1_key: str,
    points_2_key: str,
    is_rotated_key: str,
    bin_correct_detections_req_key: str,
    camera_frame,
    template_name,
    rotated_template_name,
    clustering_goal_key,
    template_frame_optical,
    rotated_template_frame_optical,
    template_frame_optical_clustered,
    clustering_duration,
    choice_key,
    pose_key,
    goto_frame_key,
    fish_bin_view_frame,
    fish_bin_view_rotated_frame,
    shark_bin_view_frame,
    shark_bin_view_rotated_frame,
) -> py_trees.composites.Selector:
    sel_template_selector_root = py_trees.composites.Selector(
        name="Template selector root",
        memory=True,
    )

    seq_unrotated = py_trees.composites.Sequence(
        name="Seq unrotated set up",
        memory=True,
    )

    seq_rotate = py_trees.composites.Sequence(
        name="Seq rotated set up",
        memory=True,
    )

    # Step 1: Check if need to rotate, returns true if need to rotate
    set_if_not_rotate = DynamicSetBlackboard(
        name="Set is_rotated",
        key=[points_1_key, points_2_key],
        update_key=is_rotated_key,
        func=lambda p1, p2: len(p1.data) > len(p2.data),
    )

    check_if_rotated = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check if is_rotated",
        check=py_trees.common.ComparisonExpression(
            variable=is_rotated_key,
            value=True,
            operator=operator.eq,
        ),
    )

    set_enable_detections_req = py_trees.behaviours.SetBlackboardVariable(
        name="Set enable detections request",
        variable_name=bin_correct_detections_req_key,
        variable_value=create_img_matching_request(
            enable=True,
            camera_frame_id=camera_frame,
            template_name=template_name,
        ),
        overwrite=True,
    )

    set_clustering_goal = py_trees.behaviours.SetBlackboardVariable(
        name="Set clustering goal",
        variable_name=clustering_goal_key,
        variable_value=create_clustering_goal(
            in_children=template_frame_optical,
            out_children=template_frame_optical_clustered,
            duration=clustering_duration,
            use_cache=False,
        ),
        overwrite=True,
    )

    sel_update_selection = create_choice_selector_root(
        choice_key=choice_key,
        pose_key=pose_key,
        goto_frame_key=goto_frame_key,
        fish_bin_frame=fish_bin_view_frame,
        shark_bin_frame=shark_bin_view_frame,
    )

    seq_unrotated.add_children(
        [
            set_if_not_rotate,
            check_if_rotated,
            set_enable_detections_req,
            set_clustering_goal,
            sel_update_selection,
        ]
    )

    set_rotated_enable_detections_req = py_trees.behaviours.SetBlackboardVariable(
        name="Set enable detections request (rotated)",
        variable_name=bin_correct_detections_req_key,
        variable_value=create_img_matching_request(
            enable=True,
            camera_frame_id=camera_frame,
            template_name=rotated_template_name,
        ),
        overwrite=True,
    )

    set_rotated_clustering_goal = py_trees.behaviours.SetBlackboardVariable(
        name="Set clustering goal (rotated)",
        variable_name=clustering_goal_key,
        variable_value=create_clustering_goal(
            in_children=rotated_template_frame_optical,
            out_children=template_frame_optical_clustered,
            duration=clustering_duration,
            use_cache=False,
        ),
        overwrite=True,
    )

    sel_rotated_update_selection = create_choice_selector_root(
        choice_key=choice_key,
        pose_key=pose_key,
        goto_frame_key=goto_frame_key,
        fish_bin_frame=fish_bin_view_rotated_frame,
        shark_bin_frame=shark_bin_view_rotated_frame,
    )

    seq_rotate.add_children(
        [
            set_rotated_clustering_goal,
            set_rotated_enable_detections_req,
            sel_rotated_update_selection,
        ]
    )

    sel_template_selector_root.add_children(
        children=[
            seq_unrotated,
            seq_rotate,
        ],
    )
    return sel_template_selector_root
