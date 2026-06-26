import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('line_follower')
    config_file = os.path.join(pkg_dir, 'config', 'line_follower_params.yaml')

    # Read from environment with defaults
    wheel_base = float(os.environ.get('WHEEL_BASE', '0.512'))
    
    # We use a fixed sensor offset of 0.30m from the base_link based on the URDF.
    # In a full setup, this could also be driven by an environment variable.
    sensor_offset_m = 0.30

    return LaunchDescription([
        Node(
            package='line_follower',
            executable='line_follower_node',
            name='line_follower',
            output='screen',
            parameters=[
                config_file,
                {
                    'wheel_base': wheel_base,
                    'sensor_offset_m': sensor_offset_m
                }
            ]
        ),
        Node(
            package='line_follower',
            executable='navigation_state_machine_node',
            name='navigation_state_machine',
            output='screen',
            parameters=[config_file]
        )
    ])
