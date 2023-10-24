#!/usr/bin/env python3
import rclpy

from hippo_msgs.msg import Float64Stamped
from rclpy.node import Node
"""
This node computes a square-wave setpoint for the depth controller, i.e.
the setpoint jumps between two different depth values with a set duration.
You can change this code to try out other setpoint functions, e.g. a sin wave.
"""


class DepthSetpointNode(Node):

    def __init__(self):
        super().__init__(node_name='depth_setpoint_publisher')

        self.start_time = self.get_clock().now()

        # change these parameters to adjust the setpoints
        # ... or change implementation details below
        # to achieve other setpoint functions.
        self.setpoint_1 = -0.4  # in m
        self.setpoint_2 = -0.6  # in m
        self.duration = 10.0  # in seconds

        self.depth_setpoint_pub = self.create_publisher(
            Float64Stamped, 'depth_setpoint', 1)
        self.timer = self.create_timer(1 / 50, self.on_timer)

    def on_timer(self):
        # change this for other setpoint functions
        now = self.get_clock().now()
        time = self.start_time - now
        i = time % (self.duration * 2)
        if i > (self.duration):
            setpoint = self.setpoint_1
        else:
            setpoint = self.setpoint_2

        # publish setpoint
        msg = Float64Stamped()
        msg.data = setpoint
        msg.header.stamp = self.get_clock().now()
        self.depth_setpoint_pub.publish(msg)


def main():
    rclpy.init()
    node = DepthSetpointNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
