#!/usr/bin/env python3

from dataclasses import dataclass
from functools import partial
import threading

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from robot_localization.srv import ToLL
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from message_filters import Subscriber, Cache


@dataclass
class NodeConfig:
    cloud_topic: str = '/points'
    sensor_frame: str = 'sensor_frame'
    world_frame: str = 'odom'
    max_points: int = 5
    process_interval_sec: float = 2.0
    tf_timeout: float = 0.5
    tf_prefix: str = ''
    toll_service_name: str = 'toLL'


class ToLLServiceClient:
    def __init__(self, node: Node, service_name: str):
        self._node = node
        self._client = node.create_client(ToLL, service_name)
        while not self._client.wait_for_service(timeout_sec=2.0):
            node.get_logger().debug(f'{service_name} not available, waiting...')
        node.get_logger().info(f'Connected to {service_name}')

    def call_async(self, point, on_response):
        if not self._client.service_is_ready():
            self._node.get_logger().warn('ToLL service not ready')
            return False

        req = ToLL.Request()
        req.map_point.x = point.x
        req.map_point.y = point.y
        req.map_point.z = point.z

        future = self._client.call_async(req)
        future.add_done_callback(on_response)
        return True


class PointCloudProcessor:
    def __init__(self, max_points: int):
        self._max_points = max_points

    def read_points(self, cloud_msg):
        points = point_cloud2.read_points_list(
            cloud_msg,
            field_names=["x", "y", "z"],
            skip_nans=True
        )
        return [(float(p[0]), float(p[1]), float(p[2])) for p in points]

    def sample_points(self, points):
        if len(points) <= self._max_points:
            return points
        stride = len(points) // self._max_points
        return points[::stride][:self._max_points]


class PointCloudToGeoPoints(Node):
    def __init__(self):
        super().__init__('pointcloud_to_geopoints_node')
        self.get_logger().info('Started PointCloud to GeoPoints Node')

        self._declare_parameters()
        self._config = self._load_config()
        self._setup_tf()
        self._setup_subscription()
        self._setup_publisher()
        self._toll_client = ToLLServiceClient(self, self._config.toll_service_name)

        self._geo_points = []
        self._pending_requests = 0
        self._lock = threading.Lock()
        self._processor = PointCloudProcessor(self._config.max_points)

        self.create_timer(self._config.process_interval_sec, self._process_timer_callback)

        self.get_logger().info(
            f"Subscribing to {self._config.cloud_topic} in frame {self._config.sensor_frame}"
        )
        self.get_logger().info(
            f"Using ToLL service: {self._config.toll_service_name}, "
            f"process interval: {self._config.process_interval_sec}s"
        )

    def _declare_parameters(self):
        self.declare_parameter('cloud_topic', '/points')
        self.declare_parameter('sensor_frame', 'sensor_frame')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('max_points', 5)
        self.declare_parameter('process_interval_sec', 2.0)
        self.declare_parameter('tf_timeout', 0.5)
        self.declare_parameter('tf_prefix', '')
        self.declare_parameter('toll_service_name', 'toLL')

    def _load_config(self):
        params = {
            'cloud_topic': self.get_parameter('cloud_topic').get_parameter_value().string_value,
            'sensor_frame': self.get_parameter('sensor_frame').get_parameter_value().string_value,
            'world_frame': self.get_parameter('world_frame').get_parameter_value().string_value,
            'max_points': self.get_parameter('max_points').get_parameter_value().integer_value,
            'process_interval_sec': self.get_parameter('process_interval_sec').get_parameter_value().double_value,
            'tf_timeout': self.get_parameter('tf_timeout').get_parameter_value().double_value,
            'tf_prefix': self.get_parameter('tf_prefix').get_parameter_value().string_value,
            'toll_service_name': self.get_parameter('toll_service_name').get_parameter_value().string_value,
        }
        return NodeConfig(**params)

    def _setup_tf(self):
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _setup_subscription(self):
        self._cache = Cache(
            Subscriber(self, PointCloud2, self._config.cloud_topic),
            1
        )

    def _setup_publisher(self):
        self._publisher = self.create_publisher(
            Float32MultiArray,
            'local/feature_geo_points_sub',
            10
        )

    def _process_timer_callback(self):
        msg = self._cache.getElemBeforeTime(self.get_clock().now())
        if msg is None:
            return

        points = self._processor.read_points(msg)
        if not points:
            return

        sampled = self._processor.sample_points(points)
        self.get_logger().info(f'Received {len(points)} points, sampling {len(sampled)}')
        self._transform_and_convert(sampled, msg.header.stamp)

    def _transform_and_convert(self, points, stamp):
        for point in points:
            p_odom = self._transform_point(point, stamp)
            if p_odom is None:
                continue

            with self._lock:
                self._pending_requests += 1

            if not self._toll_client.call_async(p_odom, self._on_toll_response):
                with self._lock:
                    self._pending_requests -= 1
                self._try_publish()

    def _on_toll_response(self, future):
        try:
            response = future.result()
            if response is not None:
                lat = response.ll_point.latitude
                lon = response.ll_point.longitude
                alt = response.ll_point.altitude
                with self._lock:
                    self._geo_points.extend([lat, lon, alt])
        except Exception as e:
            self.get_logger().error(f'ToLL service call failed: {e}')
        finally:
            with self._lock:
                self._pending_requests -= 1
            self._try_publish()

    def _try_publish(self):
        with self._lock:
            if self._pending_requests == 0 and self._geo_points:
                output_msg = Float32MultiArray()
                output_msg.data = self._geo_points
                self._publisher.publish(output_msg)
                self._geo_points.clear()

    def _transform_point(self, point, stamp):
        p_sensor = PointStamped()
        p_sensor.header.stamp = stamp
        p_sensor.header.frame_id = self._config.sensor_frame
        p_sensor.point.x = point[0]
        p_sensor.point.y = point[1]
        p_sensor.point.z = point[2]

        try:
            transform = self._tf_buffer.lookup_transform(
                self._config.world_frame,
                self._config.sensor_frame,
                Time(),
                timeout=Duration(seconds=self._config.tf_timeout)
            )
            p_odom = tf2_geometry_msgs.do_transform_point(p_sensor, transform)
            return p_odom.point
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToGeoPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PointCloud to GeoPoints Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
