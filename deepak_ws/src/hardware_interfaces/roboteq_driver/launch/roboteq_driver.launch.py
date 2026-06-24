import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Directories
    driver_dir = get_package_share_directory('roboteq_driver')

    # 1. Standalone Roboteq CAN Driver Node
    roboteq_driver_node = Node(
        package='roboteq_driver',
        executable='roboteq_driver_node',
        name='roboteq_driver_node',
        output='screen',
        parameters=[{
            'can_interface': os.environ.get('CAN_INTERFACE', 'can0'),
            'cmd_topic': os.environ.get('CMD_TOPIC', '/cmd_rpm'),
            'feedback_topic': os.environ.get('FEEDBACK_TOPIC', '/drive/feedback'),
            'gear_ratio': float(os.environ.get('GEAR_RATIO', '1.0')),
            'max_rpm': float(os.environ.get('MAX_RPM', '3000.0'))
        }]
    )

    return LaunchDescription([
        roboteq_driver_node
    ])
