import os
import yaml
import pathlib
from launch import LaunchDescription
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    param_config = os.path.join(
        get_package_share_directory('joy_thruster_map'),
        'config',
        'params.yaml'
    )

    ld = LaunchDescription()

    node = Node(
        package='joy_thruster_map',
        executable='joy_thruster_map',
        name='surge_teleop',
        namespace="alpha",
        parameters=[param_config],
        output='screen'   
    )

    ld.add_action(node)

    return ld