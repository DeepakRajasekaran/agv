import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    diff_drive_dir = get_package_share_directory('diff_drive_hardware')
    roboteq_dir = get_package_share_directory('roboteq_hardware_interface')
    mgs_dir = get_package_share_directory('mgs_hardware_interface')
    line_follower_dir = get_package_share_directory('line_follower')

    enable_line_follower_arg = DeclareLaunchArgument(
        'enable_line_follower',
        default_value='false',
        description='Set to true to launch the line follower node'
    )
    enable_line_follower = LaunchConfiguration('enable_line_follower')

    diff_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(diff_drive_dir, 'launch', 'diff_drive.launch.py'))
    )

    roboteq_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(roboteq_dir, 'launch', 'roboteq_interface.launch.py'))
    )

    mgs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mgs_dir, 'launch', 'mgs_driver.launch.py'))
    )

    line_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(line_follower_dir, 'launch', 'line_follower.launch.py')),
        condition=IfCondition(enable_line_follower)
    )
    return LaunchDescription([
        enable_line_follower_arg,
        roboteq_launch,
        mgs_launch,
        line_follower_launch,
        
        # 2. Delay the diff_drive_hardware stack slightly so that parameters/namespaces are set up properly
        TimerAction(
            period=2.0,
            actions=[diff_drive_launch]
        )
    ])
