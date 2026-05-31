"""BlueROV2 torpedo mission BT — full upstream parity, BlueROV-rewired.

Mirrored from mission_planner_2.vehicles.auv.trees.robosub24.torpedo.torpedo
with the same patches we applied for bins:
- `/auv4/...` topics replaced with `/bluerov/...`
- frames switched to BlueROV link names (front_cam_optical,
  torpedo_shooter_{left,right}_link)
- goto pulled from scripts/goto.py via sys.path so Locomotion goals land on
  /bluerov/controls instead of /auv4/controls
- move_and_shoot_seq and point_correspondences_check imported from this
  package (the upstream versions hardcode auv4/base_link_ned, world_ned, and
  AUVSharedAction.CLUSTER)
- NAMESPACE hardcoded — generate_namespace() needs `/trees/` in the file path
- SEARCH_DEPTH negated for map/ENU (upstream NED uses positive = below)
- ArmAndSetMode + goto_torpedo_vicinity wrapped around the upstream tree
"""
import operator
import os
import sys

import py_trees
from ament_index_python.packages import get_package_prefix
from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
from lifecycle_msgs.srv import ChangeState
from std_srvs.srv import Trigger

from mission_planner_2.common.core import checked_service
from mission_planner_2.common.util.detection_utils import (
    create_end_vision_req,
    create_img_matching_request,
    create_start_vision_req,
)
from mission_planner_2.common.util.namespace_utils import full_key_generator
from mission_planner_2.common.util.pose_utils import create_stamped_pose
from mission_planner_2.vehicles.shared.trees.blackboard import MultiSetBlackboard

from bluerov_tasks.shared_trees.search import create_search_front_root
from bluerov_tasks.torpedo.move_and_shoot_seq import (
    create_firing_root,
    create_move_and_shoot_generator,
)
from bluerov_tasks.torpedo.point_correspondences_check import (
    create_point_correspondences_check_root,
)

# scripts/goto.py and arm_and_set_mode.py install to lib/bluerov_tasks/ via
# ament_cmake `install(PROGRAMS …)`, not into this Python package. Same idiom
# as bluerov_tasks/bins/bins.py and bluerov_tasks/shared_trees/search.py.
_BLUEROV_SCRIPTS_DIR = os.path.join(
    get_package_prefix("bluerov_tasks"), "lib", "bluerov_tasks"
)
if _BLUEROV_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _BLUEROV_SCRIPTS_DIR)
import goto  # noqa: E402  (bluerov_tasks/scripts/goto.py)
from arm_and_set_mode import ArmAndSetMode  # noqa: E402  (bluerov_tasks/scripts/arm_and_set_mode.py)

# Hardcoded namespace — upstream uses generate_namespace() which walks the
# caller path for `/trees/`. Same workaround as bins.py.
NAMESPACE = "/bluerov/torpedo"
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
SELECTED_TEMPLATE = 2  # MUST be 1 or 2 (unused — selector picks via image match)

VISION_SERVER_TOPIC = "/bluerov/torpedo/manage_nodes"
TOGGLE_TEMPLATE_TOPIC = "/bluerov/torpedo/image_matching/toggle_template"
POINT_CORRESPONDENCES_TOPIC = "/bluerov/torpedo/image_matching/point_correspondences"

BASE_LINK_FRAME = "base_link"
CAMERA_FRAME = "front_cam_optical"
TORPEDO_SHOOTER_LEFT_FRAME = "torpedo_shooter_left_link"
TORPEDO_SHOOTER_RIGHT_FRAME = "torpedo_shooter_right_link"
TEMPLATE_FRAME_YOLO = "torpedo/yolo"
TEMPLATE_FRAME_YOLO_CLUSTERED = "torpedo/yolo/clustered"

CENTRE_VIEW_FRAME = "torpedo/centre/view"

ACTUATION_TOPIC_LEFT = "/bluerov/actuation/torpedo/left"
ACTUATION_TOPIC_RIGHT = "/bluerov/actuation/torpedo/right"

DISTANCE_THRESHOLD = 0.025
YAW_THRESHOLD = 1.0

CLUSTER_DURATION = 4
REALIGN_CLUSTER_DURATION = 2
STABILIZE_DURATION = 3
WAIT_AFTER_FIRE_DURATION = 3.0
NUM_RETRIES = 3
# Map/ENU convention: negative z = below surface. Upstream AUV uses NED
# where positive = down — flipped here. Torpedo panels sit at world z=-2
# (RoboSub-2025 pool, bb_worlds/worlds/robosub_2025_pool.world:168,174);
# -0.5 keeps the front camera looking down at the panels without diving so
# aggressively that the position controller overshoots and can't settle
# inside the 0.2m goto threshold (first-pass tuning — make this more
# negative once we confirm the panels are framed and the controller is happy).
SEARCH_DEPTH = -0.5

# Coarse approach point in map/ENU before the front-yaw search fires.
# Panels straddle the origin at x=±3, y=0. We target panel v1 (the negative-x
# one) directly: park ~1.5 m south of it and yaw to face north so the front
# camera looks straight at the panel. The yaw-scan then refines.
TORPEDO_VICINITY_X = -3.0
TORPEDO_VICINITY_Y = -1.5
# Yaw=90° in ENU map frame = facing +y (north) toward the panel.
TORPEDO_VICINITY_YAW_DEG = 90.0

TORPEDO_TEMPLATE_1 = "Task04_Tagging_01.png"
TORPEDO_TEMPLATE_2 = "Task04_Tagging_02.png"

TEMPLATE_FRAME_OPTICAL_1 = "Task04_Tagging_01_optical"
TEMPLATE_FRAME_OPTICAL_2 = "Task04_Tagging_02_optical"

TEMPLATE_FRAME_CLUSTERED_1 = "torpedo_1"
TEMPLATE_FRAME_CLUSTERED_2 = "torpedo_2"

FISH_SHOOT_FRAME_1 = "torpedo_1/fish/view"
FISH_SHOOT_FRAME_2 = "torpedo_2/fish/view"

SHARK_SHOOT_FRAME_1 = "torpedo_1/shark/view"
SHARK_SHOOT_FRAME_2 = "torpedo_2/shark/view"

SHOOT_REPEATS = 2
MAX_ALIGN_FAILURE = 5
#########################################################################

# Internal keys — should not collide as long as NAMESPACE above is unique.
_POSE_KEY = fk("pose")
_POSE_FRAME_KEY = fk("pose_frame")
_ANCHOR_FRAME_KEY = fk("anchor_frame")
_CORRECT_TEMPLATE_KEY = fk("correct_template")
_CORRECT_REQ_KEY = fk("correct_req")
_CORRECT_FISH_KEY = fk("correct_fish")
_CORRECT_SHARK_KEY = fk("correct_shark")
_CORRECT_CLUSTERED_KEY = fk("correct_clustered")


def create_torpedo_root(
    world_to_torp_yaw: float,
    zero_yaw_key: str,
):
    """
    Create the root of the torpedo tree.

    1 - arm + GUIDED + dive to search depth
    2 - move to torpedo vicinity (between the two panels)
    3 - front-yaw search to populate torpedo/yolo/clustered
    4 - check point correspondences for both templates
    5 - select correct template + enable detections
    6 - move and shoot first torpedo
    7 - return to centre, move and shoot second torpedo
    8 - stop vision

    Fallback: if any of the above fails, fire both torpedoes blindly so the
    competition score doesn't go to zero on a perception miss.
    """
    move_and_shoot_gen = create_move_and_shoot_generator(
        anchor_frame_key=_ANCHOR_FRAME_KEY,
        torpedo_shooter_left_frame=TORPEDO_SHOOTER_LEFT_FRAME,
        torpedo_shooter_right_frame=TORPEDO_SHOOTER_RIGHT_FRAME,
        choice_key="/global/choice_is_fish",
        pose_key=_POSE_KEY,
        pose_frame_key=_POSE_FRAME_KEY,
        fish_shoot_frame_key=_CORRECT_FISH_KEY,
        shark_shoot_frame_key=_CORRECT_SHARK_KEY,
        template_frame_optical_key=_CORRECT_TEMPLATE_KEY,
        template_frame_optical_clustered_key=_CORRECT_CLUSTERED_KEY,
        cluster_duration=CLUSTER_DURATION,
        realign_cluster_duration=REALIGN_CLUSTER_DURATION,
        actuation_topic_left=ACTUATION_TOPIC_LEFT,
        actuation_topic_right=ACTUATION_TOPIC_RIGHT,
        distance_threshold=DISTANCE_THRESHOLD,
        yaw_threshold=YAW_THRESHOLD,
        retries=MAX_ALIGN_FAILURE,
        stabilization_duration=2.5,
        num_retries_clustering=NUM_RETRIES,
        wait_after_fire_duration=WAIT_AFTER_FIRE_DURATION,
        shoot_repeats=SHOOT_REPEATS,
        world_to_torp_yaw=world_to_torp_yaw,
        zero_yaw_key=zero_yaw_key,
    )

    seq_launch_torpedo = py_trees.composites.Sequence(
        name="Launch torpedo",
        memory=True,
    )

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

    # Coarse approach: fly directly to panel v1's vicinity at search depth
    # and yaw to face the panel before the front-yaw search refines. Same
    # pattern as the bin tree's goto_bin_vicinity, but with specified_heading
    # so the front camera is pointed roughly at the panel after the move.
    goto_torpedo_vicinity = goto.FromConstant(
        name="Goto torpedo vicinity",
        pose=create_stamped_pose(
            frame_id="map",
            position_x=TORPEDO_VICINITY_X,
            position_y=TORPEDO_VICINITY_Y,
            yaw=TORPEDO_VICINITY_YAW_DEG,
        ),
        anchor_frame_name=BASE_LINK_FRAME,
        specified_heading=True,
        depth_override_value=SEARCH_DEPTH,
    )

    seq_search = create_search_front_root(
        object_frame=TEMPLATE_FRAME_YOLO,
        object_frame_clustered=TEMPLATE_FRAME_YOLO_CLUSTERED,
        wait_between_moves=2.0,
        search_depth=SEARCH_DEPTH,
    )

    goto_torp_centre = goto.FromConstant(
        name="Goto torp centre",
        pose=create_stamped_pose(CENTRE_VIEW_FRAME),
        anchor_frame_name=CAMERA_FRAME,
    )

    seq_check_point_correspondences = create_point_correspondences_check_root(
        toggle_template_topic=TOGGLE_TEMPLATE_TOPIC,
        camera_frame=CAMERA_FRAME,
        template_frame_1=TORPEDO_TEMPLATE_1,
        template_frame_2=TORPEDO_TEMPLATE_2,
        template_frame_optical_1=TEMPLATE_FRAME_OPTICAL_1,
        template_frame_optical_2=TEMPLATE_FRAME_OPTICAL_2,
        num_retries=NUM_RETRIES,
        points_correspondences_topic=POINT_CORRESPONDENCES_TOPIC,
        correct_template_key=_CORRECT_TEMPLATE_KEY,
    )

    sel_correct_stuff = py_trees.composites.Selector(
        name="Select correct frames to set",
        memory=True,
    )

    seq_set_template_1 = py_trees.composites.Sequence(
        name="Set for template 1",
        memory=True,
    )

    check_template = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check template",
        check=py_trees.common.ComparisonExpression(
            variable=_CORRECT_TEMPLATE_KEY,
            value=TEMPLATE_FRAME_OPTICAL_1,
            operator=operator.eq,
        ),
    )

    set_multi_template_1 = MultiSetBlackboard(
        name="Set template 1 stuff",
        keys=[
            _CORRECT_REQ_KEY,
            _CORRECT_FISH_KEY,
            _CORRECT_SHARK_KEY,
            _CORRECT_CLUSTERED_KEY,
        ],
        values=[
            create_img_matching_request(
                enable=True,
                camera_frame_id=CAMERA_FRAME,
                template_name=TORPEDO_TEMPLATE_1,
            ),
            FISH_SHOOT_FRAME_1,
            SHARK_SHOOT_FRAME_1,
            TEMPLATE_FRAME_CLUSTERED_1,
        ],
        overwrite=True,
    )

    seq_set_template_1.add_children(
        [
            check_template,
            set_multi_template_1,
        ]
    )

    set_multi_template_2 = MultiSetBlackboard(
        name="Set template 2 stuff",
        keys=[
            _CORRECT_REQ_KEY,
            _CORRECT_FISH_KEY,
            _CORRECT_SHARK_KEY,
            _CORRECT_CLUSTERED_KEY,
        ],
        values=[
            create_img_matching_request(
                enable=True,
                camera_frame_id=CAMERA_FRAME,
                template_name=TORPEDO_TEMPLATE_2,
            ),
            FISH_SHOOT_FRAME_2,
            SHARK_SHOOT_FRAME_2,
            TEMPLATE_FRAME_CLUSTERED_2,
        ],
        overwrite=True,
    )

    sel_correct_stuff.add_children(
        [
            seq_set_template_1,
            set_multi_template_2,
        ]
    )

    srv_enable_correct_detections_frame = checked_service.FromBlackboard(
        name="Enable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        key_request=_CORRECT_REQ_KEY,
        check_func=lambda x: x.new_state,
    )

    retry_enable_detections = py_trees.decorators.Retry(
        name="retry enable correct srv",
        child=srv_enable_correct_detections_frame,
        num_failures=NUM_RETRIES,
    )

    move_and_shoot_first = move_and_shoot_gen(first=True)

    goto_back_centre = goto.FromConstant(
        name="Go back to centre",
        pose=create_stamped_pose(CENTRE_VIEW_FRAME),
        anchor_frame_name=CAMERA_FRAME,
    )

    move_and_shoot_second = move_and_shoot_gen(first=False)

    seq_stop_vision = py_trees.composites.Sequence(
        name="Stop vision",
        memory=True,
    )

    srv_disable_detections = checked_service.FromConstant(
        name="Disable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=create_img_matching_request(
            enable=False,
            camera_frame_id=CAMERA_FRAME,
            template_name="",
        ),
        check_func=lambda x: x is not None and x.new_state is False,
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

    srv_end_vision = checked_service.FromConstant(
        name="End vision",
        service_type=ChangeState,
        service_name=VISION_SERVER_TOPIC,
        service_request=create_end_vision_req(),
        check_func=lambda x: x.success,
    )

    retry_end_vision = py_trees.decorators.Retry(
        name="Retry End Vision",
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

    seq_launch_torpedo.add_children(
        children=[
            retry_start_vision,
            goto_torpedo_vicinity,
            seq_search,
            goto_torp_centre,
            seq_check_point_correspondences,
            sel_correct_stuff,
            retry_enable_detections,
            move_and_shoot_first,
            goto_back_centre,
            move_and_shoot_second,
            seq_stop_vision,
        ],
    )

    sel_always_fire_fallback = py_trees.composites.Selector(
        name="Always fire torpedo fallback",
        memory=True,
    )

    seq_repeated_firing_fallback = py_trees.composites.Sequence(
        name="Always fire torpedoes fallback sequence",
        memory=True,
    )

    seq_repeated_firing_fallback.add_children(
        [
            create_firing_root(
                actuation_topic=ACTUATION_TOPIC_LEFT,
                torp_string="first",
                shoot_repeats=SHOOT_REPEATS,
                wait_after_fire_duration=WAIT_AFTER_FIRE_DURATION,
            ),
            create_firing_root(
                actuation_topic=ACTUATION_TOPIC_RIGHT,
                torp_string="second",
                shoot_repeats=SHOOT_REPEATS,
                wait_after_fire_duration=WAIT_AFTER_FIRE_DURATION,
            ),
        ]
    )

    sel_always_fire_fallback.add_children(
        [
            seq_launch_torpedo,
            seq_repeated_firing_fallback,
        ]
    )

    # Hardcode the fish/shark blackboard value so move_and_shoot's lambda
    # (which reads choice.success to pick fish_shoot_frame vs shark_shoot_frame)
    # doesn't KeyError on `/global/choice_is_fish`. Upstream populates this via
    # the choice_server service; we just default to fish for now. Trigger.Response
    # matches the type the lambda expects (.success bool field).
    set_choice_fish = py_trees.behaviours.SetBlackboardVariable(
        name="Set choice is fish (default)",
        variable_name="/global/choice_is_fish",
        variable_value=Trigger.Response(success=True, message="hardcoded fish"),
        overwrite=True,
    )

    # Pre-set the global base_link frame name string that move_and_shoot's
    # create_goto_cluster_from_bb_root reads via start_frame_keys=[..., "/global/base_link"].
    # Upstream lazily populates this from some bootstrap node; the BT crashes
    # at the first TF-checker tick otherwise. Same workaround as choice_is_fish.
    set_global_base_link = py_trees.behaviours.SetBlackboardVariable(
        name="Set /global/base_link",
        variable_name="/global/base_link",
        variable_value=BASE_LINK_FRAME,
        overwrite=True,
    )

    # Top-level sequence: arm + GUIDED, prime globals, then run the torpedo
    # mission (with its own success/fallback selector). Mirrors the bin BT
    # root in bluerov_tasks/bins/bins.py so the torpedo mission doesn't rely
    # on the operator arming via QGroundControl first.
    root = py_trees.composites.Sequence(
        name="Torpedo mission root",
        memory=True,
    )
    root.add_children(
        [
            ArmAndSetMode(name="arm_and_set_mode"),
            set_choice_fish,
            set_global_base_link,
            sel_always_fire_fallback,
        ]
    )

    return root
