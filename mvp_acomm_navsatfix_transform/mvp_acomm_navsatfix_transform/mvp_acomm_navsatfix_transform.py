import rclpy
import numpy as np
from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
from acomms_msgs.msg import UsblData
from sensor_msgs.msg import NavSatFix
from tf_transformations import euler_matrix, quaternion_matrix

class AcommNavsatfixTransform(Node):
    def __init__(self):
        super().__init__('mvp_acomm_geopoint_transform_node')
        self.get_logger().info('Started mvp_acomm_geopoint_transform_node Node')

        self.R = 6371000
        # Parameters
        self.declare_parameter('geopose_topic', '/wamv_rise/mvp_c2_commander/remote/id_4/geopose')
        self.declare_parameter('usbl_topic', '/wamv_rise/usbl/usbl_data')
        self.declare_parameter('modem_navsatfix_topic', '/wamv_rise/usbl/modem_navsatfix')
        self.declare_parameter('world_frame', 'wamv_rise/world')
        self.declare_parameter('sensor2baselink_tf', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.geopose_topic_name = self.get_parameter('geopose_topic').get_parameter_value().string_value
        self.usbl_topic_name = self.get_parameter('usbl_topic').get_parameter_value().string_value
        self.modem_navsatfix_topic_name = self.get_parameter('modem_navsatfix_topic').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.sensor2baselink_tf_array = self.get_parameter('sensor2baselink_tf').get_parameter_value().double_array_value

        self.current_geopose = None
        self.current_usbl_data = None

        # subscription
        self.create_subscription(GeoPoseStamped, self.geopose_topic_name, self.geopose_callback, 10)
        self.create_subscription(UsblData, self.usbl_topic_name, self.usbl_data_callback, 10)
        
        # publisher
        self.modem_navsatfix_pub = self.create_publisher(NavSatFix, self.modem_navsatfix_topic_name, 10)
    
    def geopose_callback(self, msg):
        self.current_geopose = msg

    def usbl_data_callback(self, msg):
        self.current_usbl_data = msg
        self.transform()
    
    def new_lat_lon(self, p_enu):
        x = p_enu[1]
        y = p_enu[0]
        dx = x
        dy = y
        ref_lat = self.current_geopose.pose.position.latitude
        ref_lon = self.current_geopose.pose.position.longitude

        new_lat = ref_lat + dx / self.R * 180 / np.pi
        R_lat = self.R * np.cos(ref_lat / 180.0 * np.pi)
        new_lon = ref_lon + dy / R_lat * 180 / np.pi

        return new_lat, new_lon

    def transform(self):
        p_usbl = np.array([self.current_usbl_data.xyz.x, self.current_usbl_data.xyz.y, self.current_usbl_data.xyz.z, 1.0])

        T_baselink_usbl = euler_matrix(self.sensor2baselink_tf_array[3], self.sensor2baselink_tf_array[4], self.sensor2baselink_tf_array[5], axes='sxyz')
        T_baselink_usbl[0, 3] = self.sensor2baselink_tf_array[0]
        T_baselink_usbl[1, 3] = self.sensor2baselink_tf_array[1]
        T_baselink_usbl[2, 3] = self.sensor2baselink_tf_array[2]

        p_baselink = np.matmul(T_baselink_usbl, p_usbl)

        T_enu_baselink = quaternion_matrix([self.current_geopose.pose.orientation.x,
                                            self.current_geopose.pose.orientation.y,
                                            self.current_geopose.pose.orientation.z,
                                            self.current_geopose.pose.orientation.w])

        p_enu = np.matmul(T_enu_baselink, p_baselink)

        new_lat, new_lon = self.new_lat_lon(p_enu)
    
        modem_navsat = NavSatFix()
        modem_navsat.header.frame_id = self.world_frame
        modem_navsat.header.stamp = self.get_clock().now().to_msg()
        modem_navsat.latitude = new_lat
        modem_navsat.longitude = new_lon
        modem_navsat.altitude = p_enu[2]
        self.modem_navsatfix_pub.publish(modem_navsat)


def main(args=None):
    rclpy.init(args=args)
    node = AcommNavsatfixTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down mvp_acomm_geopoint_transform_node Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
