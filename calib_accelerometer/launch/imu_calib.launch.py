import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_name = 'robot'

    # IMU
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('microstrain_inertial_examples'), 
            'launch/gx5_25_launch.py')])
    )

    # calib
    calib = Node(
        package='calib_accelerometer',
        executable='calib_accelerometer_node',
        name='calib_accelerometer',
        namespace=robot_name,
        output='screen',
        parameters=[
            {'max_samples': 500},
        ],
        # remappings=[('/imu/data', '/imu/data_raw')],        
        emulate_tty=True        
    )   

    return LaunchDescription([
        imu,
        calib,
    ])