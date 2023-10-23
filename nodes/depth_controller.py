#!/usr/bin/env python3
import rclpy

from std_msgs.msg import Float64
from hippo_msgs.msg import ActuatorSetpoint, DepthStamped
from rclpy.node import Node
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.

You 
"""


class DepthControlNode(Node):

    def __init__(self):
        super().__init__(node_name='depth_controller')

        self.current_setpoint = 0.0
        self.current_depth = None

        self.thrust_pub = self.create_publisher(ActuatorSetpoint,
                                                'thrust_setpoint', 1)

        self.setpoint_sub = self.create_subscription(Float64, 'depth_setpoint',
                                                     self.on_setpoint, 1)
        self.depth_sub = self.create_subscription(DepthStamped, 'depth',
                                                  self.on_depth, 1)

    def on_setpoint(self, setpoint_msg):
        # We received a new setpoint!
        # Let's save it, so that we can use it as soon as we receive
        # new depth data.
        self.current_setpoint = setpoint_msg.data

    def on_depth(self, depth_msg):
        # We received a new depth message!
        # Now we can get to action!
        current_depth = depth_msg.depth

        self.get_logger().info("Hi! I'm your controller running. " +
                               f"I received a depth of {current_depth} m.",
                               throttle_duration_sec=1)

        thrust_z = self.compute_control_output()

        # Publish the thrust command
        thrust_msg = ActuatorSetpoint()
        thrust_msg.z = thrust_z

        # Let's add a time stamp
        now = self.get_clock().now()
        thrust_msg.header.stamp = now.to_msg()

        self.thrust_pub.publish(thrust_msg)

    def compute_control_output(current_depth):
        # TODO: Do something...

        thrust_z = 1.0  # This doesn't seem right yet...

        return thrust_z


def main():
    rclpy.init()
    node = DepthControlNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
