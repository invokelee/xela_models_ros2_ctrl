#!/usr/bin/env python3
"""
Publish a URDF string once on a topic with latched QoS (transient local).
Used to expose full robot_description on /robot_description_full.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__("robot_description_publisher_full")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("topic_name", "/robot_description_full")

        urdf_str = self.get_parameter("robot_description").get_parameter_value().string_value
        topic = self.get_parameter("topic_name").get_parameter_value().string_value

        if not urdf_str:
            self.get_logger().error("Parameter 'robot_description' is empty; nothing to publish.")
            rclpy.shutdown()
            return

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(String, topic, qos)
        msg = String()
        msg.data = urdf_str
        self.pub.publish(msg)
        self.get_logger().info(f"Published robot_description to '{topic}' (latched)")


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
