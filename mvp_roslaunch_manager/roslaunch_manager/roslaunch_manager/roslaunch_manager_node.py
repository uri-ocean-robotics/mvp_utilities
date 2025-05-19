#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from roslaunch_manager_interfaces.srv import GetLaunch, SetLaunch
import os

from roslaunch_manager.roslaunch_manager import ROSLaunchManager

class ROSLaunchNode(Node):
    def __init__(self):
        super().__init__('roslaunch_manager_node')
        self.launch_files = ''
        # Create an instance of ROSLaunchManager
        self.declare_parameter('udp_srv_ip', '')
        self.declare_parameter('udp_port', 5000)
        self.declare_parameter('udp_stream_enable', False)

        self.udp_srv_ip = self.get_parameter('udp_srv_ip').value
        self.udp_srv_port = self.get_parameter('udp_port').value
        self.udp_stream_enable = self.get_parameter('udp_stream_enable').value

     

        if not self.udp_srv_ip:
            try:
            # Connect to an external server (Google DNS)
                self.sock.connect(('8.8.8.8', 80))
                local_ip = self.sock.getsockname()[0]  # Get the local address used for the connection
            except Exception:
                local_ip = '127.0.0.1'  # Fallback to localhost if no connection can be made
            print(local_ip)


        self.launch_manager = ROSLaunchManager(self.udp_srv_ip, self.udp_srv_port, self.udp_stream_enable)

        # Advertise services
        self.set_roslaunch_srv = self.create_service(SetLaunch, 'set_launch', self.f_set_launch_cb)
        self.get_roslaunch_srv = self.create_service(GetLaunch, 'get_launch', self.f_get_launch_cb)

        current_path = os.getcwd()
        print(f"Current path: {current_path}")
        
    def f_set_launch_cb(self, request, response):
        if request.set is True:
            try:
                self.launch_manager.start_launch(request.pkg, request.file)
                response.success = True
                response.message = "launch file started"
            except Exception as e:
                self.get_logger().error(f"Failed to start node: {e}")
                response.success = False
                response.message = "launch file failed"
        else:
            try:
                self.launch_manager.stop_launch(request.pkg, request.file)
                response.success = True
                response.message = "launch file stopped"
            except Exception as e:
                self.get_logger().error(f"Failed to stop node: {e}")
                response.success = False
                response.message = "launch file failed"
        return response


    def f_get_launch_cb(self, request, response):
        try:
            response.list = self.launch_manager.list_running_launches()
        except Exception as e:
            self.get_logger().error(f"Failed to get active launch files: {e}")
            response.list = []  # Optional: return empty list on failure
        return response


    def shutdown(self):
        self.get_logger().info("Shutting down ROSLaunchManager...")
        try:
            self.launch_manager.shutdown()  # Ensure this method is implemented
            self.get_logger().info("All launch files stopped.")
        except Exception as e:
            self.get_logger().error(f"Error while shutting down: {e}")

def main():
    rclpy.init()
    node = ROSLaunchNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Shutting down.')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()