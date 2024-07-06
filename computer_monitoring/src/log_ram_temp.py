#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Log RAM and Temp of computer boards and publish as ROS topics.
#tony.jacob@uri.edu

import os
import rospy
import psutil
from std_msgs.msg import Float64
from sensor_msgs.msg import Temperature

class Log_RAM_Temp:
    def __init__(self) -> None:
        self.device = self.check_device()
        
        #Get device name to add to topic
        device_name = self.device.split()[1].lower()
        self.pub_ram_info = rospy.Publisher(device_name+"/ram_utilized", Float64, queue_size=1)
        self.pub_cpu_temp_info = rospy.Publisher(device_name+"/cpu/temp", Temperature, queue_size=1)
        self.pub_cpu_usage_info = rospy.Publisher(device_name+"/cpu/utilized", Float64, queue_size=1)
        
        self.rate = rospy.Rate(1)
        self.collect_and_publish()

    def get_cpu_temp(self):
        temp = psutil.sensors_temperatures()
        
        if self.device == 'Raspberry Pi':
            return float(temp['cpu_thermal'][0][1])
        
        elif self.device == 'NVIDIA Jetson':
            return float(temp['thermal_fan_est'][0][1])
    
    def get_ram_usage(self):    
        meminfo = {}
        with open('/proc/meminfo') as f:
            for line in f:
                parts = line.split(':')
                meminfo[parts[0]] = int(parts[1].strip().split()[0])
        mem_total = meminfo['MemTotal']
        mem_available = meminfo['MemAvailable']
        mem_used = mem_total - mem_available
        return mem_total, mem_used
    
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

    def collect_and_publish(self):
        temp_msg = Temperature()
        while not rospy.is_shutdown():
            #Temperature
            temp = self.get_cpu_temp()
            temp_msg.header.stamp = rospy.Time()
            temp_msg.temperature = temp
            #RAM
            ram_total, ram_used = self.get_ram_usage()
            #CPU
            cpu_used = self.get_cpu_usage()

            self.pub_ram_info.publish((ram_used/ram_total) *100)
            self.pub_cpu_temp_info.publish(temp_msg)
            self.pub_cpu_usage_info.publish(cpu_used)
            self.rate.sleep()
    
if __name__ == "__main__":
    rospy.init_node("computer_monitoring", anonymous=True)
    Log_RAM_Temp()
    rospy.spin()