#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import NavSatFix


class GeopointsToNavsatfix(Node):
    def __init__(self):
        super().__init__('geopoints_to_navsatfix_node')
        self.get_logger().info("Started Geopoints to NavSatFix Node")

        self.declare_parameter('subscribe_topic', 'local/feature_geo_points_sub')
        self.declare_parameter('publish_topic', 'navsatfix')
        self.declare_parameter('sensor_frame', 'sensor_frame')

        self.subscribe_topic = self.get_parameter('subscribe_topic').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.sensor_frame = self.get_parameter('sensor_frame').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.subscribe_topic,
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            NavSatFix,
            self.publish_topic,
            10
        )

        self.get_logger().info(f"Subscribing to {self.subscribe_topic}")
        self.get_logger().info(f"Publishing to {self.publish_topic}")

    def listener_callback(self, msg: Float32MultiArray):
        data = msg.data
        data_stamp = self.get_clock().now().to_msg()
        for i in range(0, len(data), 3):
            if i + 2 < len(data):
                navsatfix_msg = NavSatFix()
                navsatfix_msg.latitude = float(data[i])
                navsatfix_msg.longitude = float(data[i + 1])
                navsatfix_msg.altitude = float(data[i + 2])
                navsatfix_msg.header.stamp = data_stamp
                navsatfix_msg.header.frame_id = self.sensor_frame
                self.publisher.publish(navsatfix_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GeopointsToNavsatfix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Geopoints to NavSatFix Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
