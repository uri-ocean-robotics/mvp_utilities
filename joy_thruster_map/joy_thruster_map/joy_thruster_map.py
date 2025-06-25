#!/usr/bin/env python3

# Author: Tony Jacob
# Part of RISE Project. 
# Takes in Joy msg and converts to thruster command.
# tony.jacob@uri.edu

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
import time


class JoyMapThruster(Node):
    def __init__(self):
        super().__init__('joy_thruster_map_node')
        self.get_logger().info("Started Joy-Thruster Node")

        # Parameters
        self.declare_parameter('joy_topic', Parameter.Type.STRING)
        self.declare_parameter('controller_get_state_srv', Parameter.Type.STRING)
        self.declare_parameter('thruster_config', Parameter.Type.INTEGER)
        self.declare_parameter('single_surge_map_coeff', Parameter.Type.DOUBLE)
        self.declare_parameter('single_surge_topic', Parameter.Type.STRING)
        self.declare_parameter('differential_surge_map_coeff', Parameter.Type.DOUBLE)
        self.declare_parameter('port_surge_topic', Parameter.Type.STRING)
        self.declare_parameter('starboard_surge_topic', Parameter.Type.STRING)

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.get_state_service = self.get_parameter('controller_get_state_srv').get_parameter_value().string_value
        self.thruster_config = self.get_parameter('thruster_config').get_parameter_value().integer_value
        
        self.single_surge_map_coeff = self.get_parameter('single_surge_map_coeff').get_parameter_value().double_value
        single_surge_topic = self.get_parameter('single_surge_topic').get_parameter_value().string_value
        
        self.differential_surge_map_coeff = self.get_parameter('differential_surge_map_coeff').get_parameter_value().double_value
        port_surge_topic = self.get_parameter('port_surge_topic').get_parameter_value().string_value
        starboard_surge_topic = self.get_parameter('starboard_surge_topic').get_parameter_value().string_value

        self.current_time = time.time()
        self.create_subscription(Joy, joy_topic, self.joy_callback, 10)

        # Single surge config
        if self.thruster_config == 0:
            self.single_surge = self.create_publisher(Float64, single_surge_topic, 10)
            self.get_logger().info(f"Started Thruster Mapping to Single Surge with topic name: {single_surge_topic}")
        
        # Differential surge config
        elif self.thruster_config == 1:
            self.port_thruster = self.create_publisher(Float64, port_surge_topic, 10)
            self.starboard_thruster = self.create_publisher(Float64, starboard_surge_topic, 10)
            self.get_logger().info(f"Started Thruster Mapping to Differential Surge Thrusters with topic name: {port_surge_topic} and {starboard_surge_topic}")


    def joy_callback(self, msg):
        if self.check_controller_state():
            if self.thruster_config == 0:
                self.send_single_surge_command(msg)
            elif self.thruster_config == 1:
                self.send_differential_surge_command(msg)
            
    def send_single_surge_command(self, msg):        
        surge_cmd = Float64()
        surge_cmd.data = msg.axes[1] * self.single_surge_map_coeff
        self.single_surge.publish(surge_cmd)
        # self.get_logger().info(f'Published surge command: {surge_cmd.data:.2f}')

    def send_differential_surge_command(self, msg):
        port_cmd, starboard_cmd = Float64(), Float64()
        port_cmd.data = msg.axes[1] * self.differential_surge_map_coeff
        starboard_cmd.data = msg.axes[3] * self.differential_surge_map_coeff

        self.port_thruster.publish(port_cmd)
        self.starboard_thruster.publish(starboard_cmd)

    def check_controller_state(self):
        elapsed_time = time.time() - self.current_time
        if elapsed_time > 0.01:
            self.current_time = time.time()
            client = self.create_client(Trigger, self.get_state_service)
            req = Trigger.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

            if future.result() is not None:
                if future.result().message == "enabled":
                    return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = JoyMapThruster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Joy-Thruster Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
