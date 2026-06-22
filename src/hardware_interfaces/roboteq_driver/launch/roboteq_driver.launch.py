import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the global parameters file
    config_dir = os.path.join(
        get_package_share_directory('roboteq_driver'),
        'config',
        'params.yaml'
    )

    # Launch the Roboteq Driver Node
    roboteq_driver_node = Node(
        package='roboteq_driver',
        executable='roboteq_driver_node',
        name='roboteq_driver_node',
        output='screen',
        parameters=[config_dir]
    )

    return LaunchDescription([
        roboteq_driver_node
    ])
