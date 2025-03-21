#!/usr/bin/env python3

import rospy
from mvp_roslaunch_manager.srv import GetLaunch, SetLaunch, GetLaunchResponse, SetLaunchResponse
import os

from roslaunch_manager import *

class ROSLaunchNode:
    def __init__(self):
        rospy.init_node('roslaunch_manager')
        self.launch_files = ''
        # Create an instance of ROSLaunchManager
        self.udp_srv_ip = rospy.get_param('~udp_srv_ip', '')
        self.udp_srv_port = rospy.get_param('~udp_port', 5000)
        if not self.udp_srv_ip:
            try:
            # Connect to an external server (Google DNS)
                self.sock.connect(('8.8.8.8', 80))
                local_ip = self.sock.getsockname()[0]  # Get the local address used for the connection
            except Exception:
                local_ip = '127.0.0.1'  # Fallback to localhost if no connection can be made
            print(local_ip)



        self.launch_manager = ROSLaunchManager(self.udp_srv_ip, self.udp_srv_port)

        # Advertise services
        self.set_roslaunch_srv = rospy.Service('set_launch', SetLaunch, self.f_set_launch_cb)
        self.get_roslaunch_srv = rospy.Service('get_launch', GetLaunch, self.f_get_launch_cb)

        rospy.on_shutdown(self.shutdown)
        current_path = os.getcwd()
        print(f"Current path: {current_path}")
        
    def f_set_launch_cb(self, req):
        resp = SetLaunchResponse()

        if req.set == True:
            try:
                self.launch_manager.start_launch(req.file)
                resp.success = True
                resp.message = "launch file started"
                return resp
            except Exception as e:
                rospy.logerr(f"Failed to start node: {e}")
                resp.success = False
                resp.message = "launch file failed"
                return resp
        else:
            try:
                ##print launch files
                self.launch_manager.stop_launch(req.file)
                resp.success = True
                resp.message = "launch file stopped"
                return resp
            except Exception as e:
                rospy.logerr(f"Failed to start node: {e}")
                resp.success = False
                resp.message = "launch file failed"
                return resp


    def f_get_launch_cb(self, req):
        resp = GetLaunchResponse()
        try:
            resp.list = self.launch_manager.list_running_launches()

        except Exception as e:
                rospy.logerr(f"fialed to get active launc files: {e}")    

        return resp

    def shutdown(self):
        rospy.loginfo("Shutting down ROSLaunchManager...")
        try:
            self.launch_manager.shutdown()  # You need to implement this in ROSLaunchManager
            rospy.loginfo("All launch files stopped.")
        except Exception as e:
            rospy.logerr(f"Error while shutting down: {e}")

if __name__ == '__main__':
    try:
        node = ROSLaunchNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass