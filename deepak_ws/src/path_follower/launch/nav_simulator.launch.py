import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    force_track_arg = DeclareLaunchArgument(
        'force_track_detect',
        default_value='false',
        description='Force track detection to True'
    )
    track_detect_stable_arg = DeclareLaunchArgument(
        'track_detect_stable_ms',
        default_value='1000',
        description='Required continuous track_detect duration before start/recovery, in ms'
    )

    return LaunchDescription([
        force_track_arg,
        track_detect_stable_arg,
        Node(
            package='path_follower',
            executable='nav_simulator.py',
            name='nav_simulator',
            output='screen',
            parameters=[
                {'turn_sequence': ['left', 'right', 'straight']},
                {'loop_sequence': True},
                {'force_track_detect': LaunchConfiguration('force_track_detect')},
                {'track_detect_stable_ms': ParameterValue(
                    LaunchConfiguration('track_detect_stable_ms'), value_type=int)}
            ]
        )
    ])
