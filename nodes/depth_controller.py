#!/usr/bin/env python3
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""

import rclpy
from hippo_msgs.msg import DepthStamped, Float64Stamped
from hippo_control_msgs.msg import ActuatorSetpoint
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


class DepthControlNode(Node):
    def __init__(self):
        super().__init__(node_name='depth_controller')

        self.current_setpoint = 0.0
        self.current_depth = 0.0

        self.last_error = 0.0
        self.error_sum = 0.0

        self.p_gain = 0.0
        self.i_gain = 0.0
        self.d_gain = 0.0
        self.g_offset = 0.0

        # ROS paramter for proportional gain
        self.declare_parameter(
            name='p_gain', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Proportional control gain.')
        )
        # ROS parameter for integral gain
        self.declare_parameter(
            name='i_gain', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Integral control gain.')
        )
        # ROS parameter for differential gain
        self.declare_parameter(
            name='d_gain', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Differential control gain.')
        )
        # ROS parameter for differential gain
        self.declare_parameter(
            name='g_offset', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Vertical thrust offset.')
        )
        self.add_on_set_parameters_callback(self.parameters_callback)

        # ROS publisher for thrust setpoint
        self.thrust_pub = self.create_publisher(
            msg_type=ActuatorSetpoint, topic='thrust_setpoint', qos_profile=1
        )

        # ROS subscriber for depth setpoint
        self.setpoint_sub = self.create_subscription(
            msg_type=Float64Stamped,
            topic='depth_setpoint',
            callback=self.on_setpoint,
            qos_profile=1,
        )
        # ROS subscriber for depth
        self.depth_sub = self.create_subscription(
            msg_type=DepthStamped,
            topic='depth',
            callback=self.on_depth,
            qos_profile=1,
        )

    def parameters_callback(self, params):
        success = False

        for param in params:
            if param.name == "p_gain":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.p_gain = param.value
                    success = True
            if param.name == "i_gain":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.i_gain = param.value
                    self.error_sum = 0.0
                    success = True
            if param.name == "d_gain":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.d_gain = param.value
                    success = True
            if param.name == "g_offset":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.g_offset = param.value
                    success = True

        self.get_logger().info(
            f"Set controller parameters.\n"
            f"Proportional gain: {self.p_gain}\n"
            f"Integral gain: {self.i_gain}\n"
            f"Differential gain: {self.d_gain}\n"
            f"Vertical thrust offset: {self.g_offset}"
        )

        return SetParametersResult(successful=success)

    def on_setpoint(self, setpoint_msg: Float64Stamped):
        # We received a new setpoint! Let's save it, so that we can use it as
        # soon as we receive new depth data.
        self.current_setpoint = setpoint_msg.data

    def on_depth(self, depth_msg: DepthStamped):
        # We received a new depth message! Now we can get to action!
        self.current_depth = depth_msg.depth

        self.get_logger().info(
            f'Received a depth of {self.current_depth:6.4f} m, depth setpoint at {self.current_setpoint:6.4f}',
            throttle_duration_sec=1,
        )

        thrust = self.compute_control_output()
        # either set the timestamp to the current time or set it to the
        # stamp of `depth_msg` because the control output corresponds to this
        # point in time. Both choices are meaningful.
        # option 1:
        # now = self.get_clock().now()
        # option 2:
        timestamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)

    def publish_vertical_thrust(
        self, thrust: float, timestamp: rclpy.time.Time
    ) -> None:
        msg = ActuatorSetpoint()
        # we want to set the vertical thrust exlusively. mask out xy-components.
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False

        msg.z = thrust

        # Let's add a time stamp
        msg.header.stamp = timestamp.to_msg()

        self.thrust_pub.publish(msg)

    def compute_control_output(self) -> float:
        control_error = self.current_setpoint - self.current_depth
        self.error_sum += control_error
        error_diff = control_error - self.last_error
        self.last_error = control_error

        # TODO: Apply the PID control
        thrust_z = self.p_gain * control_error + self.i_gain * self.error_sum + self.d_gain * error_diff + self.g_offset
        return thrust_z


def main():
    rclpy.init()
    node = DepthControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
