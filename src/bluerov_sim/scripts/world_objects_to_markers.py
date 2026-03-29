#!/usr/bin/env python3
"""
Reads a Gazebo world SDF and publishes game elements as a live MarkerArray
for visualization in Foxglove's 3D panel.

Subscribes directly to gz-transport for dynamic pose updates (bypasses
ros_gz_bridge, which does not populate child_frame_id from Pose_V.name in
the ros-humble-ros-gzharmonic bridge).

Topics published:
  /world_objects/markers  (visualization_msgs/msg/MarkerArray, TRANSIENT_LOCAL)

Parameters:
  world_file (str): Absolute path to the .world SDF file.
  gz_world_name (str): Internal Gazebo world name (e.g. "sauvc_2024"). When set,
      the node subscribes to gz-transport dynamic_pose/info and republishes
      at up to 10 Hz whenever a tracked object's pose changes.
"""
import threading
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

try:
    import gz.transport13 as _gz_transport
    from gz.msgs10.pose_v_pb2 import Pose_V as _GzPoseV
except ImportError:
    _gz_transport = None
    _GzPoseV = None


# Maps SDF model:// URIs to primary package:// mesh URIs.
# Only models with a self-contained mesh file are listed; composite
# or shader-only models are intentionally absent.
_MODEL_MESHES = {
    "model://sp_swimming_pool":
        "package://bb_worlds/models/sp_swimming_pool/meshes/sppool.dae",
    "model://robosub25/slalom":
        "package://bb_worlds/models/robosub25/slalom/mesh.dae",
    "model://robosub25/torpedo_panel_v1":
        "package://bb_worlds/models/robosub25/torpedo_panel_v1/model.dae",
    "model://robosub25/torpedo_panel_v2":
        "package://bb_worlds/models/robosub25/torpedo_panel_v2/model.dae",
    "model://robosub25/gate":
        "package://bb_worlds/models/robosub_gate_base/gate.dae",
    "model://robosub25/ladle":
        "package://bb_worlds/models/robosub25/ladle/ladle.obj",
    "model://robosub25/bottle":
        "package://bb_worlds/models/robosub25/bottle/bottle.obj",
    "model://robosub25/octagon":
        "package://bb_worlds/models/octagon/octagon.dae",
    "model://robosub25/collection_table":
        "package://bb_worlds/models/robosub25/collection_table/collection_table.dae",
    # robosub25/bin  — composite model (task label box only, no mesh)
    # pool_waves     — GLSL water shader, no static mesh
}

_THROTTLE_SEC = 0.1  # max publish rate: 10 Hz


def _parse_pose(pose_text: str):
    """Convert an SDF <pose> string (x y z roll pitch yaw) to position + quaternion."""
    vals = [float(v) for v in pose_text.strip().split()]
    x, y, z = vals[0], vals[1], vals[2]
    qx, qy, qz, qw = quaternion_from_euler(vals[3], vals[4], vals[5])
    return x, y, z, qx, qy, qz, qw


class WorldObjectsPublisher(Node):
    def __init__(self):
        super().__init__("world_objects_publisher")

        world_file = (
            self.declare_parameter("world_file", "")
            .get_parameter_value()
            .string_value
        )
        self._gz_world_name = (
            self.declare_parameter("gz_world_name", "")
            .get_parameter_value()
            .string_value
        )

        # TRANSIENT_LOCAL (latched) so subscribers joining after startup
        # still receive the full marker array.
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(MarkerArray, "/world_objects/markers", latched_qos)
        self._markers: list = []
        self._name_to_id: dict = {}
        self._last_publish_time = None
        self._throttle_lock = threading.Lock()

        if not world_file:
            self.get_logger().warn("world_file parameter not set — no markers published.")
            return

        self._markers, self._name_to_id = self._build_markers(world_file)

        if self._markers:
            self._pub.publish(MarkerArray(markers=self._markers))
            self.get_logger().info(
                f"Published {len(self._markers)} world object marker(s) from {world_file}"
            )
        else:
            self.get_logger().warn("No mapped models found in world file.")

        if self._gz_world_name and self._name_to_id:
            if _gz_transport is None:
                self.get_logger().error(
                    "gz.transport13 not available — live pose updates disabled."
                )
                return

            pose_topic = f"/world/{self._gz_world_name}/dynamic_pose/info"
            # Node must be kept alive as an instance attribute or the
            # subscription is silently dropped when it goes out of scope.
            self._gz_node = _gz_transport.Node()
            self._gz_node.subscribe(_GzPoseV, pose_topic, self._gz_pose_callback)
            self.get_logger().info(f"Subscribed (gz-transport) to {pose_topic}")

    def _build_markers(self, world_file: str):
        try:
            tree = ET.parse(world_file)
        except Exception as e:
            self.get_logger().error(f"Failed to parse world file '{world_file}': {e}")
            return [], {}

        root = tree.getroot()
        world_elem = root.find("world") or root

        markers = []
        name_to_id: dict = {}
        for marker_id, include in enumerate(world_elem.findall("include")):
            uri_elem = include.find("uri")
            if uri_elem is None:
                continue

            uri = (uri_elem.text or "").strip()
            mesh_uri = _MODEL_MESHES.get(uri)
            if mesh_uri is None:
                continue

            # Determine the name Gazebo will use for this model instance.
            # Explicit <name> takes priority; otherwise fall back to the last
            # component of the URI (e.g. "model://sp_swimming_pool" → "sp_swimming_pool").
            name_elem = include.find("name")
            if name_elem is not None:
                model_name = (name_elem.text or "").strip()
            else:
                model_name = uri.split("/")[-1]

            pose_elem = include.find("pose")
            pose_text = (pose_elem.text if pose_elem is not None else None) or "0 0 0 0 0 0"
            x, y, z, qx, qy, qz, qw = _parse_pose(pose_text)

            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "world_objects"
            m.id = marker_id
            m.type = Marker.MESH_RESOURCE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.0
            # White with alpha=1 so embedded materials render correctly in Foxglove
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.mesh_resource = mesh_uri
            m.mesh_use_embedded_materials = True
            markers.append(m)
            name_to_id[model_name] = marker_id

        return markers, name_to_id

    def _gz_pose_callback(self, msg) -> None:
        """Called from gz-transport thread when dynamic poses are updated."""
        updated = False
        for pose in msg.pose:
            idx = self._name_to_id.get(pose.name)
            if idx is None:
                continue

            m = self._markers[idx]
            m.pose.position.x = pose.position.x
            m.pose.position.y = pose.position.y
            m.pose.position.z = pose.position.z
            m.pose.orientation.x = pose.orientation.x
            m.pose.orientation.y = pose.orientation.y
            m.pose.orientation.z = pose.orientation.z
            m.pose.orientation.w = pose.orientation.w
            updated = True

        if not updated:
            return

        with self._throttle_lock:
            now = self.get_clock().now()
            if self._last_publish_time is not None:
                elapsed = (now - self._last_publish_time).nanoseconds * 1e-9
                if elapsed < _THROTTLE_SEC:
                    return
            self._last_publish_time = now

        self._pub.publish(MarkerArray(markers=self._markers))


def main(args=None):
    rclpy.init(args=args)
    node = WorldObjectsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
