import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    package_name = 'mvp_acomm_navsatfix_transform'

    # namespace_arg = DeclareLaunchArgument(
    #     'namespace',
    #     default_value='wamv_rise',
    #     description='Namespace for the node'
    # )

    param_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params.yaml'
    )

    # executable_path = os.path.join(
    #     get_package_prefix(package_name),
    #     'bin',
    #     package_name
    # )

    ld = LaunchDescription()

    node = Node(
        package=package_name,
        executable=package_name,
        name='mvp_acomm_navsatfix_transform_node',
        namespace='wamv_rise',
        parameters=[param_config],
        output='screen'
    )

    # ld.add_action(namespace_arg)
    ld.add_action(node)

    return ld
