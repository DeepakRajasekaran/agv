import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Directories
    driver_dir = get_package_share_directory('roboteq_driver')

    # Files
    params_file = os.path.join(driver_dir, 'config', 'params.yaml')

    # 1. Standalone Roboteq CAN Driver Node
    roboteq_driver_node = Node(
        package='roboteq_driver',
        executable='roboteq_driver_node',
        name='roboteq_driver_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        roboteq_driver_node
    ])
