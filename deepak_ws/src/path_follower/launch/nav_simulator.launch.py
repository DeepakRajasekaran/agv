import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    force_track_arg = DeclareLaunchArgument(
        'force_track_detect',
        default_value='false',
        description='Force track detection to True'
    )

    return LaunchDescription([
        force_track_arg,
        Node(
            package='path_follower',
            executable='nav_simulator.py',
            name='nav_simulator',
            output='screen',
            parameters=[
                {'turn_sequence': ['left', 'right', 'straight']},
                {'loop_sequence': True},
                {'force_track_detect': LaunchConfiguration('force_track_detect')}
            ]
        )
    ])
