#!/usr/bin/env python3
"""
ROS 2 action server: bb_controls_msgs/Locomotion (/bluerov/controls)

Drives the BlueROV2 to a goal pose expressed as body-frame setpoints
(forward, sidemove, depth, heading) via MAVROS setpoint_position/local.

Implements the bb_controls_msgs/Locomotion interface used by
mission_planner_2's goto behaviours.  Only move_rel=True is needed for
the square mission but move_rel=False (absolute map-frame) is also handled.

Publish rate is capped at 1 Hz — ArduSub replans its trajectory on every
setpoint and will move very slowly at higher rates (see bluerov_movement.py).
"""

import math
import time as py_time
import traceback
from typing import Optional

import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from bb_controls_msgs.action import Locomotion
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_geometry_msgs import do_transform_pose_stamped

BASE_FRAME = "base_link"
SETPOINT_FRAME = "map"
PUBLISH_RATE_HZ = 1.0
DEFAULT_TIMEOUT_SEC = 60.0
DEFAULT_DIST_THRESHOLD = 0.2  # metres
DEFAULT_YAW_THRESHOLD_DEG = 5.0  # degrees


class LocomotionActionServer(Node):

    def __init__(self) -> None:
        super().__init__("locomotion_action_server")

        cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Locomotion,
            "/bluerov/controls",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cb_group,
        )

        mavros_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", mavros_qos
        )

        self._current_pose: Optional[PoseStamped] = None
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_cb,
            mavros_qos,
            callback_group=cb_group,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info("LocomotionActionServer ready on /bluerov/controls")

    # ------------------------------------------------------------------ #
    # ROS callbacks                                                        #
    # ------------------------------------------------------------------ #

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._current_pose = msg

    def _goal_cb(self, goal_request) -> GoalResponse:
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------ #
    # Execute callback                                                     #
    # ------------------------------------------------------------------ #

    def _execute_cb(self, goal_handle) -> Locomotion.Result:
        goal = goal_handle.request

        # Extract first waypoint (single-waypoint mode for square mission)
        forward = goal.forward_setpoints[0] if goal.forward_setpoints else 0.0
        sidemove = goal.sidemove_setpoints[0] if goal.sidemove_setpoints else 0.0
        depth = goal.depth_setpoints[0] if goal.depth_setpoints else 0.0
        heading_deg = goal.heading_setpoints[0] if goal.heading_setpoints else 0.0

        timeout_sec = (
            float(goal.max_correction_time)
            if goal.max_correction_time > 0
            else DEFAULT_TIMEOUT_SEC
        )
        dist_thresh = (
            goal.forward_tolerance
            if goal.forward_tolerance > 0.0
            else DEFAULT_DIST_THRESHOLD
        )
        yaw_thresh_deg = (
            goal.heading_tolerance
            if goal.heading_tolerance > 0.0
            else DEFAULT_YAW_THRESHOLD_DEG
        )

        self.get_logger().info(
            f"Executing: move_rel={goal.move_rel} "
            f"fwd={forward:.2f}m side={sidemove:.2f}m "
            f"depth={depth:.2f}m hdg={heading_deg:.1f}° "
            f"timeout={timeout_sec}s"
        )

        try:
            target_in_map = self._build_target(
                forward,
                sidemove,
                depth,
                heading_deg,
                goal.move_rel,
                goal.specified_heading,
            )
        except Exception as e:
            self.get_logger().error(
                f"TF transform failed: {e}\n{traceback.format_exc()}"
            )
            goal_handle.abort()
            return self._make_result(Locomotion.Result.STATUS_ABORTED)

        self.get_logger().info(
            f"Target in map: ({target_in_map.pose.position.x:.2f}, "
            f"{target_in_map.pose.position.y:.2f}, "
            f"{target_in_map.pose.position.z:.2f})"
        )

        start_time = self.get_clock().now()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_result(Locomotion.Result.STATUS_ABORTED)

            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            if elapsed > timeout_sec:
                goal_handle.abort()
                self.get_logger().warn(f"Goal timed out after {elapsed:.1f}s")
                return self._make_result(Locomotion.Result.STATUS_FAILURE)

            target_in_map.header.stamp = self.get_clock().now().to_msg()
            self._setpoint_pub.publish(target_in_map)

            if self._current_pose is not None:
                dist = self._distance(self._current_pose, target_in_map)
                yaw_err_deg = math.degrees(
                    self._yaw_error(self._current_pose, target_in_map)
                )

                feedback = Locomotion.Feedback()
                feedback.forward_error = float(
                    self._current_pose.pose.position.x - target_in_map.pose.position.x
                )
                feedback.sidemove_error = float(
                    self._current_pose.pose.position.y - target_in_map.pose.position.y
                )
                feedback.depth_error = float(
                    self._current_pose.pose.position.z - target_in_map.pose.position.z
                )
                feedback.heading_error = float(yaw_err_deg)
                total_dist = max(abs(forward) + abs(sidemove) + abs(depth), 0.01)
                feedback.overall_progress = float(max(0.0, 1.0 - dist / total_dist))
                goal_handle.publish_feedback(feedback)

                self.get_logger().debug(
                    f"dist={dist:.3f}m  yaw_err={yaw_err_deg:.1f}°  elapsed={elapsed:.1f}s"
                )

                yaw_ok = (not goal.specified_heading) or (yaw_err_deg < yaw_thresh_deg)
                if dist < dist_thresh and yaw_ok:
                    goal_handle.succeed()
                    self.get_logger().info(
                        f"Goal reached — dist={dist:.3f}m  yaw_err={yaw_err_deg:.1f}°"
                    )
                    return self._make_result(Locomotion.Result.STATUS_SUCCESS)
            else:
                self.get_logger().warn("No pose from /mavros/local_position/pose yet")

            py_time.sleep(1.0 / PUBLISH_RATE_HZ)

        goal_handle.abort()
        return self._make_result(Locomotion.Result.STATUS_ABORTED)

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def _build_target(
        self,
        forward: float,
        sidemove: float,
        depth: float,
        heading_deg: float,
        move_rel: bool,
        specified_heading: bool,
    ) -> PoseStamped:
        """Return the target PoseStamped in map frame."""
        yaw = math.radians(heading_deg) if specified_heading else 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        if move_rel:
            pose_in_base = PoseStamped()
            pose_in_base.header.frame_id = BASE_FRAME
            pose_in_base.pose.position.x = forward
            pose_in_base.pose.position.y = sidemove
            pose_in_base.pose.position.z = depth  # FLU: positive = up
            pose_in_base.pose.orientation.z = qz
            pose_in_base.pose.orientation.w = qw

            tf = self._tf_buffer.lookup_transform(
                SETPOINT_FRAME,
                BASE_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            target = do_transform_pose_stamped(pose_in_base, tf)
            target.header.frame_id = SETPOINT_FRAME
            return target
        else:
            target = PoseStamped()
            target.header.frame_id = SETPOINT_FRAME
            target.pose.position.x = forward
            target.pose.position.y = sidemove
            target.pose.position.z = depth
            target.pose.orientation.z = qz
            target.pose.orientation.w = qw
            return target

    @staticmethod
    def _make_result(status: int) -> Locomotion.Result:
        result = Locomotion.Result()
        result.status = status
        return result

    @staticmethod
    def _get_yaw(pose: PoseStamped) -> float:
        """Extract yaw from quaternion (same formula as bluerov_movement.py)."""
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _distance(self, current: PoseStamped, target: PoseStamped) -> float:
        dx = current.pose.position.x - target.pose.position.x
        dy = current.pose.position.y - target.pose.position.y
        dz = current.pose.position.z - target.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _yaw_error(self, current: PoseStamped, target: PoseStamped) -> float:
        return abs(
            self._normalize_angle(self._get_yaw(target) - self._get_yaw(current))
        )


# --------------------------------------------------------------------------- #
# Entry point                                                                  #
# --------------------------------------------------------------------------- #


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocomotionActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
