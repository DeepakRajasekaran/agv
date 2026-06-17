import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    controller_share = get_package_share_directory('line_follower_controller')
    params_file = os.path.join(controller_share, 'config', 'params.yaml')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run MuJoCo simulator headlessly without GUI'
    )

    return LaunchDescription([
        headless_arg,
        # Run Simulator
        Node(
            package='hardware_sim',
            executable='hardware_sim_node.py',
            name='hardware_sim_node',
            parameters=[{'headless': LaunchConfiguration('headless')}],
            output='screen'
        ),
        # Run Controller
        Node(
            package='line_follower_controller',
            executable='line_follower_controller_node',
            name='line_follower_controller_node',
            parameters=[params_file],
            output='screen'
        ),
        # Run Navigation Server
        Node(
            package='nav_server',
            executable='nav_server_node',
            name='nav_server_node',
            parameters=[params_file],
            output='screen'
        )
    ])
