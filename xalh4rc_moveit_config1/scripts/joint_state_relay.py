#!/usr/bin/env python3
"""
Relay /joint_states -> /joint_states_full with sensor_data QoS.
Keeps message content unchanged; only topic name changes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


class JointStateRelay(Node):
    def __init__(self):
        super().__init__("joint_state_relay")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.pub = self.create_publisher(JointState, "/joint_states_full", qos)
        self.sub = self.create_subscription(JointState, "/joint_states", self._cb, qos)
        self.get_logger().info("Relaying /joint_states -> /joint_states_full with sensor_data QoS")

    def _cb(self, msg: JointState):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
