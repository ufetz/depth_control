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

    def on_pressure(self, pressure_msg: FluidPressure) -> None:
        """Callback function of the pressure topic subscription, that, by
        definition, gets called each time a pressure message is received.

        Args:
            pressure_msg: A pressure measurement.
        """
        pressure = pressure_msg.fluid_pressure

        # TODO: you can remove this logging function
        self.get_logger().info(
            f'Hello, I received a pressure of {pressure} Pa. '
            'I need to calculate the depth based on this measurement.',
            throttle_duration_sec=1)

        depth = self.pressure_to_depth(pressure=pressure)
        now = self.get_clock().now()
        self.publish_depth_msg(depth=depth, now=now)

    def publish_depth_msg(self, depth: float, now: rclpy.time.Time) -> None:
        msg = DepthStamped()
        # Let's add a time stamp
        msg.header.stamp = now.to_msg()
        # and populate the depth field
        msg.depth = depth
        self.depth_pub.publish(msg)

    def pressure_to_depth(self, pressure: float) -> float:
        """Converts the measures pressure from the pressure sensor to vehicle's
        depth.

        Args:
            pressure: Measured pressure [Pa].

        Returns: Vehicle's depth [m].
            
        """
        # TODO: implement the required depth calculation
        return pressure  # this does not seem to be to correct calculation


def main():
    rclpy.init()
    node = DepthCalculator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
