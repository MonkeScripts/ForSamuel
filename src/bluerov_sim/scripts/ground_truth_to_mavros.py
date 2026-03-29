#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomRepublisher(Node):
    def __init__(self):
        super().__init__("odom_republisher")
        self.subscription = self.create_subscription(
            Odometry, "/bluerov/odom", self.odom_callback, 10
        )
        self.publisher = self.create_publisher(Odometry, "/mavros/odometry/out", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("Odometry republisher node started.")

    def odom_callback(self, msg: Odometry):
        # Republish for MAVROS/ArduSub EKF with correct timestamp
        repub_msg = Odometry()
        repub_msg.header.stamp = msg.header.stamp
        repub_msg.header.frame_id = "map"
        repub_msg.child_frame_id = "base_link"
        repub_msg.pose = msg.pose
        repub_msg.twist = msg.twist
        self.publisher.publish(repub_msg)
        self.get_logger().debug("Republished odometry message.")

        # Broadcast TF directly from Gazebo ground truth (bypasses ArduSub EKF)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
