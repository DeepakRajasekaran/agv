"""
Name:        bringup.launch.py
Author:      Deepak Rajasekaran
Date:        2026-06-29
Version:     1.0
Description: Common bringup launch configuration dynamically selecting Deepak's or Manasa's stacks.
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    # Retrieve Launch Configurations as resolved strings
    stack_val = context.perform_substitution(LaunchConfiguration('stack'))
    follower_val = context.perform_substitution(LaunchConfiguration('follower'))
    sim_val = context.perform_substitution(LaunchConfiguration('sim'))
    plc_ip_val = context.perform_substitution(LaunchConfiguration('plc_ip'))
    plc_port_val = context.perform_substitution(LaunchConfiguration('plc_port'))
    force_track_detect_val = context.perform_substitution(LaunchConfiguration('force_track_detect'))
    track_detect_stable_ms_val = context.perform_substitution(LaunchConfiguration('track_detect_stable_ms'))
    
    # Read environment variables
    mode = os.environ.get('MODE', 'HARDWARE').upper()
    sim_tool = os.environ.get('SIM_TOOL', 'MUJOCO').upper()
    
    nodes = []

    if stack_val == 'deepak':
        # Deepak's Stack
        driver_dir = get_package_share_directory('roboteq_driver')
        desc_dir = get_package_share_directory('robot_description')
        bringup_dir = get_package_share_directory('robot_bringup')

        controllers_file = os.path.join(driver_dir, 'config', 'controllers.yaml')
        urdf_file = os.path.join(desc_dir, 'urdf', 'robot.urdf.xacro')

        doc = xacro.process_file(urdf_file, mappings={'mode': mode, 'sim_tool': sim_tool})
        robot_description = {'robot_description': doc.toxml()}

        # 1. Robot State Publisher
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ))

        # 2. ros2_control Node (Hardware Interface Manager)
        nodes.append(Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                robot_description,
                controllers_file,
                {
                    'diff_drive_controller.wheel_separation': float(os.environ.get('WHEEL_BASE', '0.512')),
                    'diff_drive_controller.wheel_radius': float(os.environ.get('WHEEL_RADIUS', '0.08'))
                }
            ],
            output='screen'
        ))

        # 3. Spawners for Controllers
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ))

        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "--controller-manager", "/controller_manager", "-p", controllers_file],
        ))

        # 4. Twist Mux Node
        twist_mux_params = os.path.join(bringup_dir, 'config', 'twist_mux_topics.yaml')
        twist_mux_locks = os.path.join(bringup_dir, 'config', 'twist_mux_locks.yaml')
        
        nodes.append(Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[twist_mux_params, twist_mux_locks, {'use_stamped': False}],
            remappings=[('cmd_vel_out', '/cmd_vel_out')]
        ))

        # 4b. Twist to TwistStamped Relay Node
        nodes.append(Node(
            package='twist_stamper',
            executable='twist_stamper',
            output='screen',
            remappings=[
                ('cmd_vel_in', '/cmd_vel_out'),
                ('cmd_vel_out', '/diff_drive_controller/cmd_vel')
            ]
        ))

        # 5. MGS1600 Driver Node
        nodes.append(Node(
            package='mgs_driver',
            executable='mgs_driver_node',
            output='screen',
            parameters=[
                {'can_interface': 'can0'},
                {'node_id': 5}
            ]
        ))

        # 6. Path Follower Node
        if follower_val == 'true':
            path_follower_params = []
            if os.path.exists('/agv_config/follower_params.yaml'):
                path_follower_params.append('/agv_config/follower_params.yaml')
            path_follower_params.append({
                'safety.track_detect_stable_ms': int(track_detect_stable_ms_val)
            })

            nodes.append(Node(
                package='path_follower',
                executable='path_follower_node',
                output='screen',
                parameters=path_follower_params
            ))

        # 6b. Nav Simulator (Conditional)
        if sim_val == 'true':
            nodes.append(Node(
                package='path_follower',
                executable='nav_simulator.py',
                output='screen',
                parameters=[
                    {'turn_sequence': ['left', 'right', 'straight']},
                    {'loop_sequence': True},
                    {'force_track_detect': force_track_detect_val == 'true'},
                    {'track_detect_stable_ms': int(track_detect_stable_ms_val)}
                ]
            ))


        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(driver_dir, 'launch', 'roboteq_driver.launch.py'))
        ))

    elif stack_val == 'manasa':
        # Manasa's Stack
        diff_drive_dir = get_package_share_directory('diff_drive_hardware')
        roboteq_dir = get_package_share_directory('roboteq_hardware_interface')
        mgs_dir = get_package_share_directory('mgs_hardware_interface')
        line_follower_dir = get_package_share_directory('line_follower')

        diff_drive_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(diff_drive_dir, 'launch', 'diff_drive.launch.py'))
        )

        # Enforce startup delays
        nodes.append(TimerAction(
            period=2.0,
            actions=[diff_drive_launch]
        ))

        if mode == 'HARDWARE':
            nodes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(roboteq_dir, 'launch', 'roboteq_interface.launch.py'))
            ))
            nodes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(mgs_dir, 'launch', 'mgs_driver.launch.py'))
            ))

        if follower_val == 'true':
            nodes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(line_follower_dir, 'launch', 'line_follower.launch.py'))
            ))

    # ponytail: Launch plc_interface unconditionally as it is common for both stacks
    nodes.append(Node(
        package='plc_interface',
        executable='plc_interface_node',
        output='screen',
        parameters=[{
            'plc_ip': plc_ip_val,
            'plc_port': int(plc_port_val)
        }]
    ))

    return nodes

def generate_launch_description():
    # Common arguments
    stack_arg = DeclareLaunchArgument(
        'stack',
        default_value='deepak',
        description='Which implementation stack to run: "deepak" or "manasa"'
    )
    follower_arg = DeclareLaunchArgument(
        'follower',
        default_value='true',
        description='Set to true to launch the line follower nodes'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Set to true to launch simulator'
    )
    
    # Deepak-specific arguments
    force_track_detect_arg = DeclareLaunchArgument(
        'force_track_detect',
        default_value='false',
        description='Force track detection to True in simulator'
    )
    track_detect_stable_ms_arg = DeclareLaunchArgument(
        'track_detect_stable_ms',
        default_value='1000',
        description='Required continuous track_detect duration before start/recovery, in ms'
    )
    plc_ip_arg = DeclareLaunchArgument(
        'plc_ip',
        default_value='192.168.1.5',
        description='IP address of the PLC'
    )
    plc_port_arg = DeclareLaunchArgument(
        'plc_port',
        default_value='502',
        description='Port of the PLC'
    )

    return LaunchDescription([
        stack_arg,
        follower_arg,
        sim_arg,
        force_track_detect_arg,
        track_detect_stable_ms_arg,
        plc_ip_arg,
        plc_port_arg,
        OpaqueFunction(function=launch_setup)
    ])
