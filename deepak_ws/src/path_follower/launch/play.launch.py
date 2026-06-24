import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controller_share = get_package_share_directory('path_follower')
    
    # Conditionally load persistent tuning parameters if they exist
    custom_params = '/agv_config/path_follower.yaml'
    if os.path.exists(custom_params):
        params_file = custom_params
        print(f"[play.launch.py] Loading custom persistent tuning from {params_file}")
    else:
        params_file = os.path.join(controller_share, 'config', 'params.yaml')
        print(f"[play.launch.py] Loading default compiled tuning from {params_file}")

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
