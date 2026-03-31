#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from robot_localization.srv import ToLL
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from functools import partial
import threading
from message_filters import Subscriber, Cache


class PointCloudToGeoPoints(Node):
    def __init__(self):
        super().__init__('pointcloud_to_geopoints_node')
        self.get_logger().info("Started PointCloud to GeoPoints Node")

        self.declare_parameter('cloud_topic', '/points')
        self.declare_parameter('sensor_frame', 'sensor_frame')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('max_points', 5)
        self.declare_parameter('process_interval_sec', 1.0)
        self.declare_parameter('tf_timeout', 1.0)
        self.declare_parameter('tf_prefix', '')
        self.declare_parameter('toll_service_name', 'toLL')

        self.cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        self.sensor_frame = self.get_parameter('sensor_frame').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.max_points = self.get_parameter('max_points').get_parameter_value().integer_value
        self.process_interval_sec = self.get_parameter('process_interval_sec').get_parameter_value().double_value
        self.tf_timeout = self.get_parameter('tf_timeout').get_parameter_value().double_value
        self.tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value
        self.toll_service_name = self.get_parameter('toll_service_name').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cache = Cache(Subscriber(self, PointCloud2, self.cloud_topic), 10)

        self.publisher = self.create_publisher(
            Float32MultiArray,
            'local/feature_geo_points_sub',
            10
        )

        self.toll_client = self.create_client(ToLL, self.toll_service_name)
        while not self.toll_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('toLL service not available, waiting again...')

        self._geo_points = []
        self._pending_requests = 0
        self._lock = threading.Lock()
        self._process_timer = self.create_timer(self.process_interval_sec, self._process_timer_callback)
        self.get_logger().info(f"Subscribing to {self.cloud_topic} in frame {self.sensor_frame}")
        self.get_logger().info(f"Using ToLL service: {self.toll_service_name}, process interval: {self.process_interval_sec}s")

    def _process_timer_callback(self):
        msg = self.cache.getElemBeforeTime(self.get_clock().now())
        if msg is None:
            return

        points = self.read_points(msg)
        if not points:
            return

        sampled_points = self.sample_points(points)
        self.transform_and_convert(sampled_points, msg.header.stamp)

    def read_points(self, cloud_msg):
        points = point_cloud2.read_points_list(
            cloud_msg,
            field_names=["x", "y", "z"],
            skip_nans=True
        )
        self.get_logger().info(f'pc2geopoints: Received {len(points)} points.')
        return [(float(p[0]), float(p[1]), float(p[2])) for p in points]

    def sample_points(self, points):
        if len(points) <= self.max_points:
            return points

        stride = len(points) // self.max_points
        return points[::stride][:self.max_points]

    def transform_and_convert(self, points, stamp):
        for point in points:
            try:
                p_odom = self.transform_point(point, self.sensor_frame, self.world_frame, stamp)
                if p_odom is None:
                    continue

                with self._lock:
                    self._pending_requests += 1
                self.call_to_ll_service_async(p_odom)

            except Exception as e:
                self.get_logger().warn(f"Failed to transform/convert point: {e}")
                continue

    def call_to_ll_service_async(self, point):
        if not self.toll_client.service_is_ready():
            self.get_logger().warn("ToLL service not available")
            with self._lock:
                self._pending_requests -= 1
            self._try_publish()
            return

        req = ToLL.Request()
        req.map_point.x = point.x
        req.map_point.y = point.y
        req.map_point.z = point.z

        future = self.toll_client.call_async(req)
        future.add_done_callback(
            partial(self._on_toll_response, point=point)
        )

    def _on_toll_response(self, future, point):
        try:
            response = future.result()
            if response is not None:
                lat = response.ll_point.latitude
                lon = response.ll_point.longitude
                alt = response.ll_point.altitude
                with self._lock:
                    self._geo_points.extend([lat, lon, alt])
        except Exception as e:
            self.get_logger().warn(f"ToLL service exception: {e}")
        finally:
            with self._lock:
                self._pending_requests -= 1
            self._try_publish()

    def _try_publish(self):
        with self._lock:
            if self._pending_requests == 0 and self._geo_points:
                output_msg = Float32MultiArray()
                output_msg.data = self._geo_points.copy()
                self._geo_points.clear()
                self.publisher.publish(output_msg)

    def transform_point(self, point, from_frame, to_frame, stamp):
        try:
            p_sensor = PointStamped()
            p_sensor.header.stamp = stamp
            p_sensor.header.frame_id = from_frame
            p_sensor.point.x = point[0]
            p_sensor.point.y = point[1]
            p_sensor.point.z = point[2]

            print(f"rclpy_time: {rclpy.time.Time()}, timeout: {self.tf_timeout}", flush=True)
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout)
            )
            p_odom = tf2_geometry_msgs.do_transform_point(p_sensor, transform)
            return p_odom.point
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToGeoPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PointCloud to GeoPoints Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()