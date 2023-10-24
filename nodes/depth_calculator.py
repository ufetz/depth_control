#!/usr/bin/env python3
import rclpy

from hippo_msgs.msg import DepthStamped
from sensor_msgs.msg import FluidPressure
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
"""
This node takes as input the pressure data and computes a resulting water depth.
"""


class DepthCalculator(Node):

    def __init__(self):
        super().__init__(node_name='my_second_node')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)

        self.depth_pub = self.create_publisher(DepthStamped, 'depth', 1)
        self.pressure_sub = self.create_subscription(FluidPressure, 'pressure',
                                                     self.on_pressure, qos)

    def on_pressure(self, pressure_msg):
        pressure = pressure_msg.fluid_pressure

        self.get_logger().info(
            f'Hello, I received a pressure of {pressure} Pa. ' +
            'I need to calculate the depth based on this measurement.',
            throttle_duration_sec=1)

        # TODO: Do something

        depth = pressure  # that doesn't seem right...

        depth_msg = DepthStamped()
        depth_msg.depth = depth

        # Let's add a time stamp
        now = self.get_clock().now()
        depth_msg.header.stamp = now.to_msg()

        self.depth_pub.publish(depth_msg)


def main():
    rclpy.init()
    node = DepthCalculator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
