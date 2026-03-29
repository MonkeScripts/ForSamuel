#!/usr/bin/env python3
"""
BlueROV2 goto behaviours — thin wrappers over the AUV Locomotion action pattern.

Overrides the action registry to point at /bluerov/controls and defaults
is_relative_movement=True so the GetPoseToControlsFrame service is bypassed.
The anchor frame defaults to 'base_link' (FLU body frame).
"""
import math
import uuid
from typing import Any, Callable

import py_trees
from geometry_msgs.msg import PoseStamped

from bluerov_sim.node_registry import BlueROVSharedAction
from mission_planner_2.vehicles.auv.trees.goto import goto as auv_goto
from mission_planner_2.vehicles.shared.trees.blackboard import convert_to_safe_name
from mission_planner_2.vehicles.shared.trees.goto import goto_base


class FromBlackboard(auv_goto.FromBlackboard):
    """
    Locomotion action client for BlueROV2, reading a PoseStamped from the blackboard.

    Identical to auv.goto.FromBlackboard except:
    - Connects to /bluerov/controls (BlueROVSharedAction.LOCOMOTION)
    - is_relative_movement=True by default (bypasses GetPoseToControlsFrame)
    - anchor_frame_name defaults to 'base_link'
    - Relaxed tolerances for MAVROS-based control
    """

    def __init__(
        self,
        name: str,
        pose_key: str,
        anchor_frame_name: str = 'base_link',
        specified_heading: bool = True,
        ignore_depth: bool = False,
        x_threshold: float = 0.2,
        y_threshold: float = 0.2,
        z_threshold: float = 0.2,
        yaw_threshold: float = 5.0,
        stabilize_duration: int = 60,
        generate_feedback_message: Callable[[Any], str] = None,
        wait_for_server_timeout_sec: int = -3,
        wait_for_service_timeout_sec: int = -3,
        is_relative_movement: bool = True,
        depth_override_value: float | None = None,
    ):
        # Call goto_base.FromBlackboard.__init__ directly to inject
        # BlueROVSharedAction.LOCOMOTION instead of the hardcoded
        # AUVSharedAction.LOCOMOTION in auv.goto.FromBlackboard.__init__.
        # service_client_type=None is safe because is_relative_movement=True
        # causes setup() to return before the service client is accessed.
        goto_base.FromBlackboard.__init__(
            self,
            name=name,
            pose_key=pose_key,
            action_client_type=BlueROVSharedAction.LOCOMOTION,
            service_client_type=None,
            anchor_frame_name=anchor_frame_name,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            wait_for_service_timeout_sec=wait_for_service_timeout_sec,
        )
        self.specified_heading = specified_heading
        self.ignore_depth = ignore_depth
        self.x_threshold = x_threshold
        self.y_threshold = y_threshold
        self.z_threshold = z_threshold
        self.yaw_threshold = yaw_threshold
        self.stabilize_duration = stabilize_duration
        self.is_relative_movement = is_relative_movement
        self.depth_override_value = depth_override_value


class FromConstant(FromBlackboard):
    """
    Locomotion action client for BlueROV2, using a constant PoseStamped goal.

    Set pose.header.frame_id = 'base_link' for body-relative movement.
    Pose coordinates follow FLU convention: +x=forward, +y=left, +z=up.

    Example — move 2 m forward:
        Goto('leg1', pose=_make_pose(x=2.0, y=0.0))
    """

    def __init__(
        self,
        name: str,
        pose: PoseStamped | list[PoseStamped],
        anchor_frame_name: str = 'base_link',
        specified_heading: bool = True,
        ignore_depth: bool = False,
        x_threshold: float = 0.2,
        y_threshold: float = 0.2,
        z_threshold: float = 0.2,
        yaw_threshold: float = 5.0,
        stabilize_duration: int = 60,
        generate_feedback_message: Callable[[Any], str] = None,
        wait_for_server_timeout_sec: int = -3,
        wait_for_service_timeout_sec: int = -3,
        is_relative_movement: bool = True,
        depth_override_value: float | None = None,
    ):
        if not isinstance(pose, list):
            pose = [pose]

        namespace = py_trees.blackboard.Blackboard.absolute_name(
            '/', convert_to_safe_name(name)
        )
        pose_key = py_trees.blackboard.Blackboard.absolute_name(
            namespace, f'pose_{str(uuid.uuid4()).replace("-", "")}'
        )

        super().__init__(
            name=name,
            pose_key=pose_key,
            anchor_frame_name=anchor_frame_name,
            specified_heading=specified_heading,
            ignore_depth=ignore_depth,
            x_threshold=x_threshold,
            y_threshold=y_threshold,
            z_threshold=z_threshold,
            yaw_threshold=yaw_threshold,
            stabilize_duration=stabilize_duration,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            wait_for_service_timeout_sec=wait_for_service_timeout_sec,
            is_relative_movement=is_relative_movement,
            depth_override_value=depth_override_value,
        )

        self.blackboard.register_key(
            key='request',
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                namespace='/', key=pose_key
            ),
        )
        self.blackboard.set(name='request', value=pose)
