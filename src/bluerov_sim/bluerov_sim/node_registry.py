#!/usr/bin/env python3
"""
BlueROV2 action registry and TreeNode for mission_planner_2 integration.
"""
import rclpy.action
from bb_controls_msgs.action import Locomotion
from mission_planner_2.common.config.generic_registry import (
    ActionRegistry,
    SharedAction,
)
from mission_planner_2.common.core.tree_node import TreeNode


class BlueROVSharedAction(SharedAction):
    LOCOMOTION = ActionRegistry("/bluerov/controls", Locomotion)


class BlueROVTreeNode(TreeNode):
    """TreeNode that pre-registers the BlueROV Locomotion action client."""

    def __init__(self, node_name: str = "bluerov_tree_node"):
        super().__init__(node_name=node_name)
        self._register_action_clients()

    def _register_action_clients(self):
        action_clients = dict()
        for action in BlueROVSharedAction:
            action_clients[action.name] = rclpy.action.ActionClient(
                node=self,
                action_type=action.value.type,
                action_name=action.value.topic,
            )
        self.set_action_clients(action_clients)
