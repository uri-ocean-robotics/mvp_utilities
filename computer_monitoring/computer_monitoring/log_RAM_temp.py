#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Log RAM and Temp of computer boards and publish as ROS topics.
#tony.jacob@uri.edu

import os
import rclpy
import time
from builtin_interfaces.msg import Time as RosTime
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Temperature
import psutil


class Log_RAM_Temp(Node):
    def __init__(self):
        super().__init__('computer_monitoring')
        self.device = self.check_device()
        device_name = self.device.split()[1].lower()
        
        self.ram_pub = self.create_publisher(Float64, device_name + "/ram_utilized", 1)
        self.cpu_temp_pub = self.create_publisher(Temperature, device_name + "/cpu/temp", 1)
        self.cpu_usage_pub = self.create_publisher(Float64, device_name + "/cpu/utilized", 1)
        
        self.timer = self.create_timer(1, self.collect_and_publish)
    
    def collect_and_publish(self):
        self.get_ram_usage()
        self.get_cpu_temp()
        self.get_cpu_usage()

    def get_cpu_usage(self):
        mem_msg = Float64()
        mem_msg.data = psutil.cpu_percent()
        self.cpu_usage_pub.publish(mem_msg)

    def check_device(self):
        # Check for Raspberry Pi
        if os.path.exists('/proc/device-tree/model'):
            with open('/proc/device-tree/model', 'r') as f:
                model_info = f.read().strip()
                if 'Raspberry Pi' in model_info:
                    return 'Raspberry Pi'

        # Check for NVIDIA Jetson
        if os.path.exists('/proc/device-tree/compatible'):
            with open('/proc/device-tree/compatible', 'r') as f:
                compatible_info = f.read().strip()
                if 'nvidia' in compatible_info:
                    return 'NVIDIA Jetson'

        return 'Unknown Device'
    
    def get_ram_usage(self):
        meminfo = {}
        with open('/proc/meminfo') as f:
            for line in f:
                parts = line.split(':')
                meminfo[parts[0]] = int(parts[1].strip().split()[0])
        mem_total = meminfo['MemTotal']
        mem_available = meminfo['MemAvailable']
        mem_used = mem_total - mem_available
        mem = (mem_used/mem_total) *100
        mem_msg = Float64()
        mem_msg.data = mem
        self.ram_pub.publish(mem_msg)

    def get_cpu_temp(self):
        temp = psutil.sensors_temperatures()
        temp_msg = Temperature()
        
        now = time.time()
        secs = int(now)
        nsecs = int((now - secs) * 1e9)
        
        self.ros_time = RosTime(sec=secs, nanosec=nsecs)

        temp_msg.header.stamp = self.ros_time

        if self.device == 'Raspberry Pi':
            cpu_temp = float(temp['cpu_thermal'][0][1])
        
        elif self.device == 'NVIDIA Jetson':
            cpu_temp = float(temp['CPU-therm'][0][1])
        
        else:
            cpu_temp = float(4)
        
        temp_msg.temperature = cpu_temp
        self.cpu_temp_pub.publish(temp_msg)
    
def main():
    rclpy.init()
    node = Log_RAM_Temp()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()