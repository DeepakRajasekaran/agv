import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_drive_control')
    default_params = os.path.join(pkg_share, 'config', 'diff_drive_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to YAML file with diff drive kinematic parameters'
    )

    diff_drive_node = Node(
        package='diff_drive_control',
        executable='diff_drive_controller_node',
        name='diff_drive_controller_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        diff_drive_node,
    ])
