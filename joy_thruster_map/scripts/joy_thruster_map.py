#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Takes in Joy msg and convert to thruster command.
#tony.jacob@uri.edu

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_srvs.srv import TriggerRequest, TriggerResponse, Trigger
import time

class JoyMapThruster:
    def __init__(self):
        rospy.loginfo("Started Joy-Thruster Node")
        rospy.Subscriber("joy",Joy, self.joy_CB)

        self.pub = rospy.Publisher("surge_thruster", Float64, queue_size=1)
        self.current_time = time.time()

        self.get_state_service = rospy.get_param("joy_thruster_map/controller_get_state")
        self.surge_map_coeff = rospy.get_param("joy_thruster_map/surge_map_coeff")
        
    def joy_CB(self, msg):
        if self.check_controller_state():
            surge_cmd = Float64()
            surge_cmd.data = msg.axes[1]*self.surge_map_coeff
            self.pub.publish(surge_cmd)
            print(surge_cmd)

    def check_controller_state(self):
        """
        Function to check the state of the controller
        """
        elapsed_time = time.time() - self.current_time
        #Check state every 1s
        if elapsed_time > 0.01:
            self.current_time = time.time()
            service_client_get_state = rospy.ServiceProxy(self.get_state_service, Trigger)
            request = TriggerRequest()
            response = service_client_get_state(request)
            if response.message == "enabled":
                return True
            else:
                return False

if __name__ == "__main__":
    rospy.init_node("Joy_Thruster_Node")
    JoyMapThruster()
    rospy.spin()