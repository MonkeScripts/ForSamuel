import operator

import py_trees
import py_trees_ros
from bb_perception_msgs.msg import PointCorrespondencesStamped
from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
from rclpy.qos import qos_profile_sensor_data

from mission_planner_2.common.core import checked_service
from mission_planner_2.vehicles.shared.trees.blackboard import DynamicSetBlackboard
from mission_planner_2.common.util.detection_utils import (
    create_img_matching_request,
)
from mission_planner_2.common.util.namespace_utils import full_key_generator

# Hardcoded namespace — generate_namespace() walks the caller path for
# `/trees/` but this file isn't under one. Same workaround as bins.py.
NAMESPACE = "/bluerov/torpedo/correspondences"
fk = full_key_generator(NAMESPACE)

_POINTS_1_KEY = fk("points_1")
_POINTS_2_KEY = fk("points_2")


def _create_point_correspondences_check_root(
    toggle_template_topic: str,
    camera_frame,
    template_frame,
    template_frame_optical,
    num_retries,
    points_correspondences_topic,
    template_num,
    points_key,
):
    srv_enable_detections_frame = checked_service.FromConstant(
        name="Enable detections",
        service_name=toggle_template_topic,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=create_img_matching_request(
            enable=True,
            camera_frame_id=camera_frame,
            template_name=template_frame,
        ),
        key_response=fk("torpedo_enable_detections"),
        check_func=lambda x: x.new_state,  # check if the service call was successful
    )

    retry_enable_detections = py_trees.decorators.Retry(
        name="Retry enable detections",
        child=srv_enable_detections_frame,
        num_failures=num_retries,
    )

    sub_get_points_first = py_trees_ros.subscribers.ToBlackboard(
        name="Get points",
        topic_name=points_correspondences_topic,
        topic_type=PointCorrespondencesStamped,
        qos_profile=qos_profile_sensor_data,
        blackboard_variables={
            points_key: "object_points",
            fk("object_frame_id"): "object_frame_id",
        },
    )

    # Check whether the point correspondences are for the newly
    # set template and not from a previous task.
    check_point_correspondences_first = (
        py_trees.behaviours.CheckBlackboardVariableValue(
            name="Check point correspondences",
            check=py_trees.common.ComparisonExpression(
                variable=fk("object_frame_id"),
                value=template_frame_optical,
                operator=operator.eq,
            ),
        )
    )

    seq_get_points_first = py_trees.composites.Sequence(
        name=f"Seq template {template_num}",
        memory=True,
    )

    seq_get_points_first.add_children(
        children=[
            retry_enable_detections,
            sub_get_points_first,
            check_point_correspondences_first,
        ],
    )

    sub_get_points_first_sequence_retry = py_trees.decorators.Retry(
        name=f"Try template {template_num}",
        child=seq_get_points_first,
        num_failures=100,
    )

    return sub_get_points_first_sequence_retry


def create_point_correspondences_check_root(
    toggle_template_topic: str,
    camera_frame,
    template_frame_1,
    template_frame_2,
    template_frame_optical_1,
    template_frame_optical_2,
    num_retries,
    points_correspondences_topic,
    correct_template_key,
):
    root = py_trees.composites.Sequence(
        name="Check PointCorresponces seq",
        memory=True,
    )

    seq_check_template_1 = _create_point_correspondences_check_root(
        toggle_template_topic=toggle_template_topic,
        camera_frame=camera_frame,
        template_frame=template_frame_1,
        template_frame_optical=template_frame_optical_1,
        num_retries=num_retries,
        points_correspondences_topic=points_correspondences_topic,
        template_num=1,
        points_key=_POINTS_1_KEY,
    )

    seq_check_template_2 = _create_point_correspondences_check_root(
        toggle_template_topic=toggle_template_topic,
        camera_frame=camera_frame,
        template_frame=template_frame_2,
        template_frame_optical=template_frame_optical_2,
        num_retries=num_retries,
        points_correspondences_topic=points_correspondences_topic,
        template_num=2,
        points_key=_POINTS_2_KEY,
    )

    dynamic_set_correct_template = DynamicSetBlackboard(
        name="Dynamic set correct template",
        key=[_POINTS_1_KEY, _POINTS_2_KEY],
        update_key=correct_template_key,
        overwrite=True,
        func=lambda first, second: template_frame_optical_1
        if len(first.data) > len(second.data)
        else template_frame_optical_2,
    )

    root.add_children(
        [
            seq_check_template_1,
            seq_check_template_2,
            dynamic_set_correct_template,
        ]
    )

    return root
