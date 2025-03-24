#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Takes in Joy msg and convert to thruster command.
#tony.jacob@uri.edu

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class JoyMapThruster:
    def __init__(self):
        rospy.loginfo("Started Joy-Thruster Node")
        rospy.Subscriber("joy",Joy, self.joy_CB)

        self.pub = rospy.Publisher("surge_thruster", Float64, queue_size=1)

    def joy_CB(self, msg):
        surge_cmd = Float64()
        surge_cmd.data = msg.axes[1]
        self.pub.publish(surge_cmd)

if __name__ == "__main__":
    rospy.init_node("Joy_Thruster_Node")
    JoyMapThruster()
    rospy.spin()