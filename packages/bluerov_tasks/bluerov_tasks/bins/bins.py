import operator
import os
import sys

import py_trees
import py_trees_ros
from ament_index_python.packages import get_package_prefix
from bb_perception_msgs.msg import PointCorrespondencesStamped
from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
from lifecycle_msgs.srv import ChangeState
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

from mission_planner_2.common.core import checked_service, shared_action_client
from mission_planner_2.common.util.detection_utils import (
    create_end_vision_req,
    create_img_matching_request,
    create_start_vision_req,
)
from mission_planner_2.common.util.namespace_utils import full_key_generator
from mission_planner_2.common.util.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
    tf_to_stamped_pose,
    within_threshold_xyz,
)
from bluerov_tasks.node_registry import BlueROVSharedAction
from mission_planner_2.vehicles.auv.trees.robosub24.bins.helpers import find_acute_angle
from mission_planner_2.vehicles.auv.trees.robosub24.bins.template_selector import (
    create_template_selector_root,
)
from mission_planner_2.vehicles.shared.trees.blackboard import DynamicSetBlackboard
from mission_planner_2.vehicles.shared.trees.cluster_goto import (
    create_goto_cluster_from_bb_root,
)
from bluerov_tasks.shared_trees.search import (
    create_search_bot_layered_square_root,
)
from mission_planner_2.vehicles.shared.trees.tf_checker import (
    create_tf_checker_from_constant_root,
)

# Pull the BlueROV-routed goto wrappers from scripts/goto.py — they live in
# the installed `lib/bluerov_tasks/` directory (ament_cmake `install(PROGRAMS …)`),
# not in this package, so import via the ament-resolved prefix.
_BLUEROV_SCRIPTS_DIR = os.path.join(
    get_package_prefix("bluerov_tasks"), "lib", "bluerov_tasks"
)
if _BLUEROV_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _BLUEROV_SCRIPTS_DIR)
import goto  # noqa: E402  (bluerov_tasks/scripts/goto.py)
from arm_and_set_mode import ArmAndSetMode  # noqa: E402  (bluerov_tasks/scripts/arm_and_set_mode.py)

# Upstream uses generate_namespace(), which walks the caller path for
# `/trees/` — bluerov_tasks/bins/bins.py isn't under any `/trees/`, so we
# hardcode the namespace that path WOULD have produced in mission_planner_2
# (`/auv/robosub24/bins/bins`). Keeps blackboard keys collision-free without
# moving the file or forking namespace_utils.
NAMESPACE = "/bluerov/bins"
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/bluerov/bin/manage_nodes"

TOGGLE_TEMPLATE_TOPIC = "/bluerov/bin/image_matching/toggle_template"
TEMPLATE_NAME = "Task03_DropBRUVS.png"
ROTATED_TEMPLATE_NAME = "Task03_DropBRUVS_Rotated.png"
POINT_CORRESPONDENCES_TOPIC = "/bluerov/bin/image_matching/point_correspondences"

BASE_LINK_FRAME = "base_link"
CAMERA_FRAME = "bottom_cam_optical"
TEMPLATE_FRAME_OPTICAL = "Task03_DropBRUVS_optical"
ROTATED_TEMPLATE_FRAME_OPTICAL = "Task03_DropBRUVS_Rotated_optical"
TEMPLATE_FRAME_OPTICAL_CLUSTERED = "bin/clustered"
TEMPLATE_FRAME_YOLO = "bin/yolo"
TEMPLATE_FRAME_YOLO_CLUSTERED = "bin/yolo/clustered"

ACTUATION_TOPIC = "/bluerov/actuation/dropper"
ACTUATION_UINT = UInt8(data=6)

CLUSTERING_DURATION = 2
STABILIZE_CONTROLS_DURATION = 5.0
RETRIES = 4
NUM_RETRIES = 3

# Negative z = 1.3 m below surface in map/ENU (see SEARCH_DEPTH note).
BIN_DEPTH_OVERRIDE_VALUE = -1.3

BIN_CENTRE_VIEW_FRAME = "bin/centre/view"
FISH_BIN_VIEW_FRAME = "bin/fish/view"
SHARK_BIN_VIEW_FRAME = "bin/shark/view"
FISH_BIN_VIEW_ROTATED_FRAME = "bin/fish/rotated/view"
SHARK_BIN_VIEW_ROTATED_FRAME = "bin/shark/rotated/view"

SEARCH_PATTERN = [
    {"x": 0.0, "y": -0.25, "z": 0.0},
    {"x": 0.5, "y": 0.0, "z": 0.0},
    {"x": 0.0, "y": 0.5, "z": 0.0},
    {"x": -0.5, "y": 0.0, "z": 0.0},
]

# Bin pose in the robosub_2025_pool world (bb_worlds/worlds/robosub_2025_pool.world:
# `<include><uri>model://robosub25/bin</uri><pose>-6 4.5 -2 0 0 0</pose>`). Hardcoded
# to give the BT a coarse approach point before its layered-square search refines
# the position via YOLO detections.
BIN_WORLD_X = -6.0
BIN_WORLD_Y = 4.5

SEARCH_FWD = 1.0
SEARCH_BACK = 0.3
SEARCH_LEFT = 0.5
SEARCH_RIGHT = 0.5
# NUM_SQUARES=1 triggers a special-case in shared_trees/search.py's
# cluster_validate_seq: `operator=lambda x, y: x.cluster_spread < y or
# num_squares == 1`. So with one layer, the cluster check auto-passes and
# the BT advances after a single 4-waypoint sweep instead of trying
# escalating squares. Bump this back up (3+) once detections are stable
# enough that we want strict spread validation.
NUM_SQUARES = 1
OFFSET_COEFF = 1.0
# Map/ENU convention: negative z = below surface (see scripts/bluerov_movement.py
# self.depth = -2.0). Upstream AUV uses NED where positive = down — flipped here.
SEARCH_DEPTH = -0.3
BETWEEN_DROPS_WAIT = 3.5
EXTRA_DROP_WAIT = 0.5
WAIT_BETWEEN_MOVES = 1.0
CLUSTER_DIST_THRESHOLD = 0.5
MIN_CLUSTER_SIZE = 4
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_POSE_KEY = fk("pose")
_GOTO_FRAME_KEY = fk("goto_frame")
_ANCHOR_FRAME_KEY = fk("anchor_frame")
_POINTS_1_KEY = fk("points_1")
_POINTS_2_KEY = fk("points_2")
_IS_ROTATED_KEY = fk("is_rotated")
_CLUSTERING_GOAL_KEY = fk("clustering_goal")
_BIN_CORRECT_DETECTIONS_REQ_KEY = fk("enable_correct_detections_req")
_BIN_CORRECT_ENABLE_DETECTIONS_KEY = fk("bin_correct_enable_detections")
_BIN_CENTRE_TF_KEY = fk("bin_centre_tf")
_BIN_CENTRE_ACUTE_POSE_KEY = fk("bin_centre_acute_pose")


def create_bin_root():
    """
    Create the root of the bin tree.
    """

    seq_bin_root = py_trees.composites.Sequence(
        name="Bin root",
        memory=True,
    )

    seq_drop_into_bin = py_trees.composites.Sequence(
        name="Drop into bin",
        memory=True,
    )

    # Step 0: Enable vision pipeline
    srv_start_vision = checked_service.FromConstant(
        name="Start vision",
        service_type=ChangeState,
        service_name=VISION_SERVER_TOPIC,
        service_request=create_start_vision_req(),
        check_func=lambda x: x.success,
    )

    retry_start_vision = py_trees.decorators.Retry(
        name="Retry start vision",
        child=srv_start_vision,
        num_failures=NUM_RETRIES,
    )

    # Coarse approach: fly to the bin's known world (x, y) at search depth
    # before the search square fires. Pose is in map frame, so anchor_frame
    # stays at base_link (no offset subtraction needed) and depth_override
    # forces SEARCH_DEPTH so we don't dive to the bin floor on approach.
    # specified_heading=False — we don't care which way we're facing yet.
    goto_bin_vicinity = goto.FromConstant(
        name="Goto bin vicinity",
        pose=create_stamped_pose(
            frame_id="map",
            position_x=BIN_WORLD_X,
            position_y=BIN_WORLD_Y,
        ),
        anchor_frame_name=BASE_LINK_FRAME,
        specified_heading=False,
        depth_override_value=SEARCH_DEPTH,
    )

    seq_search = create_search_bot_layered_square_root(
        fwd=SEARCH_FWD,
        back=SEARCH_BACK,
        left=SEARCH_LEFT,
        right=SEARCH_RIGHT,
        num_squares=NUM_SQUARES,
        object_frame=TEMPLATE_FRAME_YOLO,
        object_frame_clustered=TEMPLATE_FRAME_YOLO_CLUSTERED,
        offset_coeff=OFFSET_COEFF,
        wait_between_moves=WAIT_BETWEEN_MOVES,
        search_depth=SEARCH_DEPTH,
        cluster_dist_threshold=CLUSTER_DIST_THRESHOLD,
        min_cluster_size=MIN_CLUSTER_SIZE,
    )

    cluster_bin_centre = shared_action_client.FromConstant(
        name="Cluster bin centre (debug)",
        shared_action=BlueROVSharedAction.CLUSTER,
        action_goal=create_clustering_goal(
            in_children=TEMPLATE_FRAME_YOLO,
            out_children=TEMPLATE_FRAME_YOLO_CLUSTERED,
            out_parents="map",  # upstream default is "world_ned" (AUV) — we use map
            duration=7,
        ),
    )

    extract_tf = create_tf_checker_from_constant_root(
        start_frames=[
            BASE_LINK_FRAME,
        ],
        end_frames=[
            BIN_CENTRE_VIEW_FRAME,
        ],
        update_keys=[_BIN_CENTRE_TF_KEY],
        fallback_val=[
            None,
        ],
    )

    calculate_acute_pose = DynamicSetBlackboard(
        name="Calculate acute pose to bin centre",
        key=_BIN_CENTRE_TF_KEY,
        update_key=_BIN_CENTRE_ACUTE_POSE_KEY,
        func=lambda tf: find_acute_angle(tf_to_stamped_pose(BASE_LINK_FRAME, tf)),
    )

    # Step 2: Navigate to bin centre
    goto_bin_centre = goto.FromBlackboard(
        name="Goto bin centre",
        pose_key=_BIN_CENTRE_ACUTE_POSE_KEY,
        anchor_frame_name=BASE_LINK_FRAME,
        depth_override_value=BIN_DEPTH_OVERRIDE_VALUE,
    )

    stabilise = py_trees.timers.Timer("Stabilise", duration=STABILIZE_CONTROLS_DURATION)

    # Step 3: Enable image matching detections
    srv_enable_detections = checked_service.FromConstant(
        name="Enable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=create_img_matching_request(
            enable=True,
            camera_frame_id=CAMERA_FRAME,
            template_name=TEMPLATE_NAME,
        ),
        check_func=lambda x: x.new_state == True,
    )

    retry_enable_detections = py_trees.decorators.Retry(
        name="Retry enable detections",
        child=srv_enable_detections,
        num_failures=NUM_RETRIES,
    )

    # Step 4a: Get first set of point correspondences
    sub_get_points_first = py_trees_ros.subscribers.ToBlackboard(
        name="Get points",
        topic_name=POINT_CORRESPONDENCES_TOPIC,
        topic_type=PointCorrespondencesStamped,
        qos_profile=qos_profile_sensor_data,
        blackboard_variables={
            _POINTS_1_KEY: "object_points",
            fk("object_frame_id_1"): "object_frame_id",
        },
    )

    # Check whether the point correspondences are for the newly
    # set template and not from a previous task.
    check_point_correspondences_first = (
        py_trees.behaviours.CheckBlackboardVariableValue(
            name="Check point correspondences",
            check=py_trees.common.ComparisonExpression(
                variable=fk("object_frame_id_1"),
                value=TEMPLATE_FRAME_OPTICAL,
                operator=operator.eq,
            ),
        )
    )

    seq_get_points_first = py_trees.composites.Sequence(
        name="Seq unrotated template",
        memory=True,
    )

    seq_get_points_first.add_children(
        children=[
            sub_get_points_first,
            check_point_correspondences_first,
        ],
    )

    sub_get_points_first_sequence_retry = py_trees.decorators.Retry(
        name="Try unrotated template",
        child=seq_get_points_first,
        num_failures=100,
    )

    srv_enable_detections_rotated = checked_service.FromConstant(
        name="Enable detections (rotated)",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=create_img_matching_request(
            enable=True,
            camera_frame_id=CAMERA_FRAME,
            template_name=ROTATED_TEMPLATE_NAME,
        ),
        check_func=lambda x: x.new_state
        == True,  # check if the service call was successful
    )

    retry_enable_rotated_detections = py_trees.decorators.Retry(
        name="Retry enable rotated detections",
        child=srv_enable_detections_rotated,
        num_failures=NUM_RETRIES,
    )

    # Step 4b: Get second set of point correspondences
    sub_get_points_second = py_trees_ros.subscribers.ToBlackboard(
        name="Get points (rotated)",
        topic_name=POINT_CORRESPONDENCES_TOPIC,
        topic_type=PointCorrespondencesStamped,
        qos_profile=qos_profile_sensor_data,
        blackboard_variables={
            _POINTS_2_KEY: "object_points",
            fk("object_frame_id_2"): "object_frame_id",
        },
    )

    # Check whether the point correspondences are for the newly
    # set template and not from a previous task.
    check_point_correspondences_second = (
        py_trees.behaviours.CheckBlackboardVariableValue(
            name="Check point correspondences (rotated)",
            check=py_trees.common.ComparisonExpression(
                variable=fk("object_frame_id_2"),
                value=ROTATED_TEMPLATE_FRAME_OPTICAL,
                operator=operator.eq,
            ),
        )
    )

    seq_get_points_rotated = py_trees.composites.Sequence(
        name="Seq rotated template",
        memory=True,
    )

    seq_get_points_rotated.add_children(
        children=[
            sub_get_points_second,
            check_point_correspondences_second,
        ],
    )

    sub_get_points_second_sequence_retry = py_trees.decorators.Retry(
        name="Try rotated template",
        child=seq_get_points_rotated,
        num_failures=100,
    )

    sel_update_template = create_template_selector_root(
        points_1_key=_POINTS_1_KEY,
        points_2_key=_POINTS_2_KEY,
        is_rotated_key=_IS_ROTATED_KEY,
        bin_correct_detections_req_key=_BIN_CORRECT_DETECTIONS_REQ_KEY,
        camera_frame=CAMERA_FRAME,
        template_name=TEMPLATE_NAME,
        rotated_template_name=ROTATED_TEMPLATE_NAME,
        clustering_goal_key=_CLUSTERING_GOAL_KEY,
        template_frame_optical=TEMPLATE_FRAME_OPTICAL,
        rotated_template_frame_optical=ROTATED_TEMPLATE_FRAME_OPTICAL,
        template_frame_optical_clustered=TEMPLATE_FRAME_OPTICAL_CLUSTERED,
        clustering_duration=CLUSTERING_DURATION,
        choice_key="/global/choice_is_fish",
        pose_key=_POSE_KEY,
        goto_frame_key=_GOTO_FRAME_KEY,
        fish_bin_view_frame=FISH_BIN_VIEW_FRAME,
        fish_bin_view_rotated_frame=FISH_BIN_VIEW_ROTATED_FRAME,
        shark_bin_view_frame=SHARK_BIN_VIEW_FRAME,
        shark_bin_view_rotated_frame=SHARK_BIN_VIEW_ROTATED_FRAME,
    )

    srv_enable_correct_detections = checked_service.FromBlackboard(
        "Enable correct detection template",
        service_type=IMPoseEstimatorToggleTemplate,
        service_name=TOGGLE_TEMPLATE_TOPIC,
        key_request=_BIN_CORRECT_DETECTIONS_REQ_KEY,
        key_response=_BIN_CORRECT_ENABLE_DETECTIONS_KEY,
        check_func=lambda x: x.new_state
        == True,  # check if the service call was successful
    )

    retry_enable_correct_detections = py_trees.decorators.Retry(
        name="Retry enable correct detections",
        child=srv_enable_correct_detections,
        num_failures=NUM_RETRIES,
    )

    action_cluster_for_goto = shared_action_client.FromBlackboard(
        name="Cluster transforms for dropping",
        shared_action=BlueROVSharedAction.CLUSTER,
        key=_CLUSTERING_GOAL_KEY,
    )

    retry_action_cluster_for_goto = py_trees.decorators.Retry(
        name="Retry cluster transforms for dropping",
        child=action_cluster_for_goto,
        num_failures=NUM_RETRIES,
    )

    action_cluster_for_goto_check = shared_action_client.FromBlackboard(
        name="Cluster transforms for dropping",
        shared_action=BlueROVSharedAction.CLUSTER,
        key=_CLUSTERING_GOAL_KEY,
    )

    retry_action_cluster_for_goto_check = py_trees.decorators.Retry(
        name="Retry cluster transforms for dropping",
        child=action_cluster_for_goto_check,
        num_failures=NUM_RETRIES,
    )

    # Step 9: Align to precise target
    goto_align_to_target = goto.FromBlackboard(
        name="Align to target",
        pose_key=_POSE_KEY,
        anchor_frame_name="dropper_link",
        depth_override_value=BIN_DEPTH_OVERRIDE_VALUE,
    )

    set_anchor_frame = py_trees.behaviours.SetBlackboardVariable(
        name="Set anchor frame",
        variable_name=_ANCHOR_FRAME_KEY,
        variable_value="dropper_link",
        overwrite=True,
    )

    # distance_threshold=0.05
    # TODO: check if want to retry inside the create_goto_cluster root instead
    seq_goto_cluster = py_trees.decorators.FailureIsSuccess(
        name="Goto cluster",
        child=create_goto_cluster_from_bb_root(
            cluster_node=retry_action_cluster_for_goto,
            cluster_node_check=retry_action_cluster_for_goto_check,
            goto_node=goto_align_to_target,
            retries=RETRIES,
            start_frame_keys=[_ANCHOR_FRAME_KEY],
            goto_pose_frame_key=_GOTO_FRAME_KEY,
            within_threshold_list=[
                within_threshold_xyz(0.05),
            ],
        ),
    )

    # Uncomment the following lines if you want to use the TF-based goto cluster
    # seq_goto_cluster = create_goto_cluster_from_bb_tf_tf_root(
    #     cluster_node=action_cluster_for_goto,
    #     cluster_node_check=action_cluster_for_goto_check,
    #     goto_node=goto_align_to_target,
    #     distance_threshold=0.05,
    #     retries=3,
    #     tf_frame_key=_GOTO_FRAME_KEY,
    #     within_threshold=within_threshold_dist,
    # )

    stabilise_before_dropping = py_trees.timers.Timer(
        "Stabilise before dropping", duration=STABILIZE_CONTROLS_DURATION
    )

    # Step 10: Set dropper actuation value
    set_dropper_actuation = py_trees.behaviours.SetBlackboardVariable(
        name="Set dropper actuation",
        variable_name=fk("bin_actuation"),
        variable_value=ACTUATION_UINT,
        overwrite=True,
    )

    # Step 11: Fire first dropper
    pub_fire_dropper_first = py_trees_ros.service_clients.FromConstant(
        name="Fire dropper first",
        service_name=ACTUATION_TOPIC,
        service_type=Trigger,
        service_request=Trigger.Request(),
    )

    # Step 12: Fire second dropper
    pub_fire_dropper_second = py_trees_ros.service_clients.FromConstant(
        name="Fire dropper second",
        service_name=ACTUATION_TOPIC,
        service_type=Trigger,
        service_request=Trigger.Request(),
    )

    pub_fire_dropper_third = py_trees_ros.service_clients.FromConstant(
        name="Fire dropper third",
        service_name=ACTUATION_TOPIC,
        service_type=Trigger,
        service_request=Trigger.Request(),
    )

    seq_stop_vision = py_trees.composites.Sequence(
        name="Stop vision",
        memory=True,
    )

    # Step 13: Disable detections
    srv_disable_detections = checked_service.FromConstant(
        name="Disable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=create_img_matching_request(
            enable=False,
            camera_frame_id=CAMERA_FRAME,
            template_name=TEMPLATE_NAME,
        ),
        check_func=lambda x: x is not None and x.new_state == False,
    )

    retry_disable_detections = py_trees.decorators.Retry(
        name="Retry Disable Detections",
        child=srv_disable_detections,
        num_failures=NUM_RETRIES,
    )

    force_success_disable_detections = py_trees.decorators.FailureIsSuccess(
        name="Force success disable detections",
        child=retry_disable_detections,
    )

    # Step 14: End vision pipeline
    srv_end_vision = checked_service.FromConstant(
        name="End vision pipeline",
        service_type=ChangeState,
        service_name=VISION_SERVER_TOPIC,
        service_request=create_end_vision_req(),
        check_func=lambda x: x.success,
    )

    retry_end_vision = py_trees.decorators.Retry(
        name="Retry end vision",
        child=srv_end_vision,
        num_failures=NUM_RETRIES,
    )

    force_success_end_vision = py_trees.decorators.FailureIsSuccess(
        name="Force success end vision",
        child=retry_end_vision,
    )

    seq_stop_vision.add_children(
        children=[
            force_success_disable_detections,
            force_success_end_vision,
        ],
    )

    # Build main drop sequence
    seq_drop_into_bin.add_children(
        children=[
            retry_start_vision,
            goto_bin_vicinity,
            seq_search,
            # cluster_bin_centre,  # TODO: remove this if use search
            extract_tf,
            calculate_acute_pose,
            goto_bin_centre,
            # stabilise,
            retry_enable_detections,
            sub_get_points_first_sequence_retry,
            retry_enable_rotated_detections,
            sub_get_points_second_sequence_retry,
            sel_update_template,
            retry_enable_correct_detections,
            set_anchor_frame,
            seq_goto_cluster,
            set_dropper_actuation,
            pub_fire_dropper_first,
            py_trees.timers.Timer(
                name="Wait between drops",
                duration=BETWEEN_DROPS_WAIT,
            ),
            pub_fire_dropper_second,
            py_trees.timers.Timer(
                name="Wait between drops",
                duration=EXTRA_DROP_WAIT,
            ),
            pub_fire_dropper_third,
            seq_stop_vision,
        ],
    )

    # Build root sequence
    seq_bin_root.add_children(
        children=[
            seq_drop_into_bin,
        ]
    )

    sel_bin_failure_fallback = py_trees.composites.Selector(
        name="Fallback for bin failure",
        memory=True,
    )

    seq_always_drop = py_trees.composites.Sequence(
        name="Always bin drop sequence as fallback",
        memory=True,
    )

    pub_fire_dropper_fallback_first = py_trees_ros.service_clients.FromConstant(
        name="Fire dropper first fallback",
        service_name=ACTUATION_TOPIC,
        service_type=Trigger,
        service_request=Trigger.Request(),
    )

    pub_fire_dropper_fallback_second = py_trees_ros.service_clients.FromConstant(
        name="Fire dropper second fallback",
        service_name=ACTUATION_TOPIC,
        service_type=Trigger,
        service_request=Trigger.Request(),
    )

    pub_fire_dropper_fallback_third = py_trees_ros.service_clients.FromConstant(
        name="Fire dropper third fallback",
        service_name=ACTUATION_TOPIC,
        service_type=Trigger,
        service_request=Trigger.Request(),
    )

    seq_always_drop.add_children(
        [
            pub_fire_dropper_fallback_first,
            py_trees.timers.Timer(
                name="Wait between drops",
                duration=BETWEEN_DROPS_WAIT,
            ),
            pub_fire_dropper_fallback_second,
            py_trees.timers.Timer(
                name="Wait between drops",
                duration=EXTRA_DROP_WAIT,
            ),
            pub_fire_dropper_fallback_third,
        ]
    )

    sel_bin_failure_fallback.add_children(
        [
            seq_bin_root,
            seq_always_drop,
        ]
    )

    # Top-level sequence: arm + GUIDED first, then run the bin mission
    # (with its own success/fallback selector). Mirrors the square BT root
    # in scripts/bluerov_square_mission_tree.py so the bin mission no
    # longer relies on the operator arming via QGroundControl first.
    root = py_trees.composites.Sequence(
        name="Bin mission root",
        memory=True,
    )
    root.add_children(
        [
            ArmAndSetMode(name="arm_and_set_mode"),
            sel_bin_failure_fallback,
        ]
    )

    return root
