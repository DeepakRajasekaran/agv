import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get config directory path
    package_dir = get_package_share_directory('roboteq_hardware_interface')
    config_file = os.path.join(package_dir, 'config', 'roboteq_params.yaml')

    return LaunchDescription([
        Node(
            package='roboteq_hardware_interface',
            executable='roboteq_hardware_interface_node',
            name='roboteq_interface',
            output='screen',
            parameters=[config_file]
        )
    ])
