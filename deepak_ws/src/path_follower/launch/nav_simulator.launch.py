import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_follower',
            executable='nav_simulator.py',
            name='nav_simulator',
            output='screen',
            parameters=[
                {'turn_sequence': ['left', 'right', 'straight']},
                {'loop_sequence': True},
                {'nominal_speed': 0.2}
            ]
        )
    ])
