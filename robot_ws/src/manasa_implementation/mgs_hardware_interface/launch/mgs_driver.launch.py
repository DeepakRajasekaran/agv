import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('mgs_hardware_interface')
    config_file = os.path.join(pkg_dir, 'config', 'mgs_params.yaml')

    can_interface = os.environ.get('CAN_INTERFACE', 'can0')

    return LaunchDescription([
        Node(
            package='mgs_hardware_interface',
            executable='mgs_hardware_interface_node',
            name='mgs_hardware_interface',
            output='screen',
            parameters=[
                config_file,
                {'can_interface': can_interface}
            ]
        )
    ])
