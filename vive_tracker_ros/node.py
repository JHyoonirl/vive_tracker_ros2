"""ROS 2 PoseStamped publisher for the standalone Tracker client."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from vive_tracker import ViveTracker


class ViveTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("vive_tracker_node")
        self.declare_parameter("serial_numbers", [""])
        self.declare_parameter("tracker_ids", [""])
        self.declare_parameter("namespaces", ["vive"])
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("translation", [0.0, 0.0, 0.0])
        self.declare_parameter("rpy_degrees", [0.0, 0.0, 0.0])
        self.declare_parameter("config_path", "")
        self.declare_parameter("lh_config", "")
        self.declare_parameter("survive_args", [])

        serials = [value for value in self._string_list("serial_numbers") if value]
        if not serials:
            raise ValueError("serial_numbers must contain at least one Tracker serial")
        ids = self._string_list("tracker_ids") or [f"tracker_{i}" for i in range(len(serials))]
        namespaces = self._string_list("namespaces") or ["vive"] * len(serials)
        if len(ids) != len(serials) or len(namespaces) != len(serials):
            raise ValueError("serial_numbers, tracker_ids, and namespaces must have equal lengths")

        self._serials = serials
        self._ids = ids
        self._namespaces = namespaces
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publishers = {}
        self._raw_publishers = {}
        for serial, tracker_id, namespace in zip(serials, ids, namespaces):
            prefix = f"/{namespace.strip('/')}/{tracker_id}"
            self._publishers[serial] = self.create_publisher(PoseStamped, f"{prefix}/pose", 10)
            self._raw_publishers[serial] = self.create_publisher(PoseStamped, f"{prefix}/pose_raw", 10)

        self._tracker = ViveTracker(
            serial_numbers=serials,
            translation=self.get_parameter("translation").value,
            rpy_degrees=self.get_parameter("rpy_degrees").value,
            config_path=self._optional_string("config_path"),
            lh_config=self._optional_string("lh_config"),
            args=self._string_list("survive_args"),
        )
        self._tracker.connect()
        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0:
            raise ValueError("publish_rate must be positive")
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f"tracking {len(serials)} Vive Tracker(s) at {rate:.1f} Hz")

    def _string_list(self, name: str) -> list[str]:
        value = self.get_parameter(name).value
        return [str(item) for item in value] if isinstance(value, (list, tuple)) else []

    def _optional_string(self, name: str):
        value = str(self.get_parameter(name).value)
        return value or None

    def _message(self, pose, position, orientation):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.pose.position.x, message.pose.position.y, message.pose.position.z = position
        message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = orientation
        return message

    def _publish(self) -> None:
        for serial in self._serials:
            pose = self._tracker.get_pose(serial)
            if pose is None:
                continue
            self._publishers[serial].publish(self._message(pose, pose.position, pose.orientation))
            self._raw_publishers[serial].publish(self._message(pose, pose.raw_position, pose.raw_orientation))

    def destroy_node(self):
        self._tracker.disconnect()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ViveTrackerNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
