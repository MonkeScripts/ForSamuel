#!/usr/bin/env python3
"""
BlueROV2 action/service registry and TreeNode for mission_planner_2 integration.
"""
import rclpy.action
from bb_controls_msgs.action import Locomotion
from bb_planner_msgs.srv import GetPoseToControlsFrame
from mission_planner_2.common.config.generic_registry import (
    ActionRegistry,
    ServiceRegistry,
    SharedAction,
    SharedService,
)
from mission_planner_2.common.core.tree_node import TreeNode


class BlueROVSharedAction(SharedAction):
    LOCOMOTION = ActionRegistry("/bluerov/controls", Locomotion)


class BlueROVSharedService(SharedService):
    CONVERT_TO_CONTROLS_POSE = ServiceRegistry(
        "/bluerov/convert_to_controls_pose", GetPoseToControlsFrame
    )


class BlueROVTreeNode(TreeNode):
    """TreeNode that pre-registers the BlueROV action and service clients."""

    def __init__(self, node_name: str = "bluerov_tree_node"):
        super().__init__(node_name=node_name)
        self._register_action_clients()
        self._register_service_clients()

    def _register_action_clients(self):
        action_clients = dict()
        for action in BlueROVSharedAction:
            action_clients[action.name] = rclpy.action.ActionClient(
                node=self,
                action_type=action.value.type,
                action_name=action.value.topic,
            )
        self.set_action_clients(action_clients)

    def _register_service_clients(self):
        service_clients = dict()
        for service in BlueROVSharedService:
            service_clients[service.name] = self.create_client(
                srv_type=service.value.type,
                srv_name=service.value.topic,
            )
        self.set_service_clients(service_clients)
