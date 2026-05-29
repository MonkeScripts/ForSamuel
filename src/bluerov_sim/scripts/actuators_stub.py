#!/usr/bin/env python3
"""Stub actuator node for BlueROV2 sim.

Exposes std_srvs/Trigger services that the bin and torpedo behaviour trees
call to fire the dropper and torpedoes. There is no physics here — each
Trigger logs the call and publishes a short-lived Marker at the relevant
actuator TF so Foxglove can visually confirm the firing.

Real DetachableJoint-based physics is deferred (Stage 4 in
.claude/plans/i-want-to-try-humming-crescent.md).
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


_MARKER_TOPIC = "/bluerov/actuation/markers"
_MARKER_LIFETIME_SEC = 5


def _make_marker(node: Node, marker_id: int, frame_id: str, color: tuple[float, float, float]) -> Marker:
    msg = Marker()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.ns = "bluerov_actuation"
    msg.id = marker_id
    msg.type = Marker.SPHERE
    msg.action = Marker.ADD
    msg.scale.x = msg.scale.y = msg.scale.z = 0.1
    msg.color.r, msg.color.g, msg.color.b = color
    msg.color.a = 1.0
    msg.lifetime.sec = _MARKER_LIFETIME_SEC
    return msg


class ActuatorsStub(Node):
    def __init__(self) -> None:
        super().__init__("actuators_stub")

        self._marker_pub = self.create_publisher(Marker, _MARKER_TOPIC, 10)

        self.create_service(
            Trigger, "/bluerov/actuation/dropper",
            lambda req, res: self._handle(req, res, "dropper_link", 0, (1.0, 0.8, 0.0)),
        )
        self.create_service(
            Trigger, "/bluerov/actuation/torpedo/left",
            lambda req, res: self._handle(req, res, "torpedo_shooter_left_link", 1, (1.0, 0.2, 0.2)),
        )
        self.create_service(
            Trigger, "/bluerov/actuation/torpedo/right",
            lambda req, res: self._handle(req, res, "torpedo_shooter_right_link", 2, (0.2, 0.4, 1.0)),
        )

        self.get_logger().info(
            "actuators_stub ready — /bluerov/actuation/{dropper,torpedo/left,torpedo/right} (Trigger)"
        )

    def _handle(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        frame_id: str,
        marker_id: int,
        color: tuple[float, float, float],
    ) -> Trigger.Response:
        self.get_logger().info(f"fired actuator at frame={frame_id}")
        self._marker_pub.publish(_make_marker(self, marker_id, frame_id, color))
        response.success = True
        response.message = f"actuator at {frame_id} fired (stub)"
        return response


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = ActuatorsStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
