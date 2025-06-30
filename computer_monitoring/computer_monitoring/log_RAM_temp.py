#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Log RAM and Temp of computer boards and publish as a single msg.
#tony.jacob@uri.edu

import os
import rclpy
from math import nan
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import psutil


class Log_RAM_Temp(Node):
    def __init__(self):
        super().__init__('computer_monitoring')
        self.device = self.check_device()
        device_name = self.device.split()[1].lower()
        
        self.computer_telemetry = self.create_publisher(Float32MultiArray, device_name+"/telemetry", 1)
        
        self.timer = self.create_timer(1, self.collect_and_publish)
    
    def collect_and_publish(self):
        ram_usage_percent = self.get_ram_usage()
        cpu_temp_C = self.get_cpu_temp()
        cpu_usage_percent = self.get_cpu_usage()

        msg = Float32MultiArray()
        msg.data = [ram_usage_percent, cpu_temp_C, cpu_usage_percent]
        self.computer_telemetry.publish(msg)

    def get_cpu_usage(self):
        return psutil.cpu_percent()


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
        return (mem_used/mem_total) *100


    def get_cpu_temp(self):
        temp = psutil.sensors_temperatures()

        if self.device == 'Raspberry Pi':
            cpu_temp = float(temp['cpu_thermal'][0][1])
        
        elif self.device == 'NVIDIA Jetson':
            cpu_temp = float(temp['CPU-therm'][0][1])
        
        else:
            cpu_temp = nan
        
        return cpu_temp
    
def main():
    rclpy.init()
    node = Log_RAM_Temp()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()