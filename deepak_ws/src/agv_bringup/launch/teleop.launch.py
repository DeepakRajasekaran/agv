import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    teleop_type_arg = DeclareLaunchArgument(
        'teleop_type',
        default_value='keyboard',
        description='Type of teleop: "keyboard" or "turtlebot3"'
    )

    teleop_type = LaunchConfiguration('teleop_type')
    
    # 1. teleop_twist_keyboard
    # Need to run in an external terminal, but we define the node here for completeness.
    # It publishes to /cmd_vel by default, so we remap it to /teleop/cmd_vel
    keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        remappings=[('/cmd_vel', '/teleop/cmd_vel')],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('teleop_type'), "' == 'keyboard'"
        ]))
    )
    
    # 2. turtlebot3_teleop (requires TURTLEBOT3_MODEL in env, we can spoof it or set it in launch env)
    turtlebot_node = Node(
        package='turtlebot3_teleop',
        executable='teleop_keyboard',
        output='screen',
        remappings=[('/cmd_vel', '/teleop/cmd_vel')],
        additional_env={'TURTLEBOT3_MODEL': 'burger'},
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('teleop_type'), "' == 'turtlebot3'"
        ]))
    )

    # 3. Optional Stamper (If running teleop standalone without bringup)
    # bringup.launch.py will handle stamping /cmd_vel_out to diff_drive_controller/cmd_vel
    # But if you want to bypass twist_mux in standalone mode, you can map teleop directly to hardware.
    # We will assume this is primarily used when bringup is running, so twist_mux receives /teleop/cmd_vel.

    return LaunchDescription([
        teleop_type_arg,
        keyboard_node,
        turtlebot_node
    ])
