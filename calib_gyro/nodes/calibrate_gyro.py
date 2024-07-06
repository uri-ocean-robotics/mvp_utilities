#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_srvs.srv import Empty, EmptyResponse


class calibrate_gyro:
    def __init__(self):
        self.sampling = False
        self.sample_n = 0
        self.gyro_bias = Vector3Stamped()
        imu_topic = rospy.get_param('~mag_topic', 'imu/data')
        rospy.Subscriber(imu_topic, Imu, self.imu_cb)

        rospy.Service('~start_sampling',Empty,self.start_sampling)
        rospy.Service('~stop_sampling',Empty,self.stop_sampling)
        
        self.calibrated_gyro_pub = rospy.Publisher('~gyro/bias', Vector3Stamped, queue_size=1)

    def imu_cb(self,data):

        if(self.sampling):
            x_total= self.gyro_bias.vector.x *self.sample_n + data.angular_velocity.x
            y_total = self.gyro_bias.vector.y *self.sample_n + data.angular_velocity.y
            z_total = self.gyro_bias.vector.z *self.sample_n + data.angular_velocity.z

            self.sample_n = self.sample_n + 1
            self.gyro_bias.vector.x = x_total/self.sample_n
            self.gyro_bias.vector.y = y_total/self.sample_n
            self.gyro_bias.vector.z = z_total/self.sample_n
            self.gyro_bias.header.seq = self.sample_n
            self.gyro_bias.header.stamp = rospy.Time.now()

            if self.sample_n%100==0:
                self.calibrated_gyro_pub.publish(self.gyro_bias)
                rospy.loginfo("Got %d gyro readings"%(self.sample_n))
                rospy.loginfo("current gyro bias x =" + str(self.gyro_bias.vector.x) )
                rospy.loginfo("current gyro bias y =" + str(self.gyro_bias.vector.y) )
                rospy.loginfo("current gyro bias z =" + str(self.gyro_bias.vector.z) )

                
                
    def start_sampling(self,req):
        rospy.loginfo('Started collecting gyro readings')
        self.sampling = True
        self.sample_n = 0
        self.gyro_bias.vector.x = 0
        self.gyro_bias.vector.y = 0
        self.gyro_bias.vector.z = 0
        self.gyro_bias.header.seq = 0
        self.gyro_bias.header.stamp = rospy.Time.now()
        self.gyro_bias.header.frame_id = 'imu'

        return EmptyResponse()

    def stop_sampling(self,req):
        rospy.loginfo('Stopped collecting gyro readings')
        self.sampling = False
        # rospy.loginfo('gyro bias = '+ self.gyro_bias)
        rospy.loginfo("final gyro bias x =" + str(self.gyro_bias.vector.x) )
        rospy.loginfo("final gyro bias y =" + str(self.gyro_bias.vector.y) )
        rospy.loginfo("final gyro bias z =" + str(self.gyro_bias.vector.z) )
        return EmptyResponse()

if __name__=='__main__':
    rospy.init_node('calibrate_gyro')
    cbgyro = calibrate_gyro()
    rospy.spin()
