#!/usr/bin/env python3
"""
py_trees behaviour: ArmAndSetMode

One-shot behaviour that arms the vehicle and sets GUIDED mode via MAVROS
service calls, using the async pattern from bluerov_movement.py.

Returns SUCCESS once both services confirm.
Returns FAILURE if either service call fails.
Returns RUNNING while waiting for service responses.
"""

import py_trees
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node


class ArmAndSetMode(py_trees.behaviour.Behaviour):
    """
    Send arm + set-GUIDED requests to MAVROS and wait for confirmation.

    Follows the TurtleMove.setup(**kwargs) pattern from
    mission_planner_2/vehicles/turtlesim/turtle_circle.py.
    """

    def __init__(self, name: str = "arm_and_set_mode") -> None:
        super().__init__(name)
        self.node: Node = None
        self._arm_client = None
        self._mode_client = None
        self._arm_future = None
        self._mode_future = None
        self._armed = False
        self._guided = False

    # ------------------------------------------------------------------ #
    # py_trees lifecycle                                                   #
    # ------------------------------------------------------------------ #

    def setup(self, **kwargs) -> None:
        self.logger.debug(f"{self.qualified_name}.setup()")
        try:
            self.node = kwargs["node"]
        except KeyError:
            raise KeyError(f"{self.qualified_name} did not find 'node' in kwargs")

        self._arm_client = self.node.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_client = self.node.create_client(SetMode, "/mavros/set_mode")

    def initialise(self) -> None:
        self.logger.debug(f"{self.qualified_name}.initialise()")
        self._arm_future = None
        self._mode_future = None
        self._armed = False
        self._guided = False
        self._send_requests()

    def update(self) -> py_trees.common.Status:
        self.logger.debug(f"{self.qualified_name}.update()")

        # Re-send if services not ready yet
        if self._arm_future is None or self._mode_future is None:
            self._send_requests()
            return py_trees.common.Status.RUNNING

        # Poll arm future
        if not self._armed:
            if self._arm_future.done():
                try:
                    result = self._arm_future.result()
                    if result.success:
                        self._armed = True
                        self.logger.info(f"{self.qualified_name}: vehicle armed")
                    else:
                        self.logger.warning(
                            f"{self.qualified_name}: arm request returned failure"
                        )
                        return py_trees.common.Status.FAILURE
                except Exception as e:
                    self.logger.error(f"{self.qualified_name}: arm service error: {e}")
                    return py_trees.common.Status.FAILURE

        # Poll mode future
        if not self._guided:
            if self._mode_future.done():
                try:
                    result = self._mode_future.result()
                    if result.mode_sent:
                        self._guided = True
                        self.logger.info(
                            f"{self.qualified_name}: GUIDED mode set successfully"
                        )
                    else:
                        self.logger.warning(
                            f"{self.qualified_name}: set_mode request returned failure"
                        )
                        return py_trees.common.Status.FAILURE
                except Exception as e:
                    self.logger.error(
                        f"{self.qualified_name}: set_mode service error: {e}"
                    )
                    return py_trees.common.Status.FAILURE

        if self._armed and self._guided:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.logger.debug(
            f"{self.qualified_name}.terminate({self.status} -> {new_status})"
        )

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def _send_requests(self) -> None:
        """Send arm and mode requests asynchronously (mirrors bluerov_movement.py)."""
        if not self._armed and self._arm_future is None:
            if self._arm_client.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                self._arm_future = self._arm_client.call_async(req)
                self.logger.info(f"{self.qualified_name}: arm request sent")
            else:
                self.logger.info(
                    f"{self.qualified_name}: waiting for /mavros/cmd/arming service"
                )

        if not self._guided and self._mode_future is None:
            if self._mode_client.service_is_ready():
                req = SetMode.Request()
                req.custom_mode = "GUIDED"
                self._mode_future = self._mode_client.call_async(req)
                self.logger.info(f"{self.qualified_name}: set GUIDED mode request sent")
            else:
                self.logger.info(
                    f"{self.qualified_name}: waiting for /mavros/set_mode service"
                )
