import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controller_share = get_package_share_directory('path_follower')
    params_file = os.path.join(controller_share, 'config', 'params.yaml')

    launch_entities = [
        # Run Controller
        Node(
            package='path_follower',
            executable='path_follower_node',
            name='path_follower_node',
            parameters=[params_file],
            output='screen'
        )
    ]

    return LaunchDescription(launch_entities)
