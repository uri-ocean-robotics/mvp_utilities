from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pointcloud_to_geopoints'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@todo.com',
    description='ROS2 node to convert PointCloud2 to geodetic coordinates',
    license='TODO',
    entry_points={
        'console_scripts': [
            'pointcloud_to_geopoints = pointcloud_to_geopoints.pointcloud_to_geopoints_ros:main'
        ],
    },
)
