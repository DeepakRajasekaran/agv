import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Define launch arguments
    launch_path_follower_arg = DeclareLaunchArgument(
        'launch_path_follower',
        default_value='true',
        description='Launch the path_follower node'
    )
    launch_nav_simulator_arg = DeclareLaunchArgument(
        'launch_nav_simulator',
        default_value='false',
        description='Launch the nav_simulator node'
    )
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
    
    launch_path_follower = LaunchConfiguration('launch_path_follower')
    launch_nav_simulator = LaunchConfiguration('launch_nav_simulator')
    force_track_detect = LaunchConfiguration('force_track_detect')
    track_detect_stable_ms = LaunchConfiguration('track_detect_stable_ms')
    plc_ip = LaunchConfiguration('plc_ip')
    plc_port = LaunchConfiguration('plc_port')

    # Read Environment Variables
    mode = os.environ.get('MODE', 'HARDWARE').upper()
    sim_tool = os.environ.get('SIM_TOOL', 'MUJOCO').upper()
    
    # Parse directories
    driver_dir = get_package_share_directory('roboteq_driver')
    desc_dir = get_package_share_directory('robot_description')
    bringup_dir = get_package_share_directory('robot_bringup')

    # Files
    controllers_file = os.path.join(driver_dir, 'config', 'controllers.yaml')
    urdf_file = os.path.join(desc_dir, 'urdf', 'robot.urdf.xacro')

    # Process Xacro
    doc = xacro.process_file(urdf_file, mappings={'mode': mode, 'sim_tool': sim_tool})
    robot_description = {'robot_description': doc.toxml()}

    nodes = [
        launch_path_follower_arg,
        launch_nav_simulator_arg,
        force_track_detect_arg,
        track_detect_stable_ms_arg,
        plc_ip_arg,
        plc_port_arg
    ]

    # 1. Robot State Publisher (Always runs)
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

    # 3. Spawners for Controllers (Always runs)
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

    # 5. MGS1600 Driver Node (Always runs)
    nodes.append(Node(
        package='mgs_driver',
        executable='mgs_driver_node',
        output='screen',
        parameters=[
            {'can_interface': 'can0'},
            {'node_id': 5}
        ]
    ))

    # 6. Path Follower Node (Conditional)
    path_follower_params = []
    if os.path.exists('/agv_config/follower_params.yaml'):
        path_follower_params.append('/agv_config/follower_params.yaml')
    path_follower_params.append({
        'safety.track_detect_stable_ms': ParameterValue(track_detect_stable_ms, value_type=int)
    })

    nodes.append(Node(
        package='path_follower',
        executable='path_follower_node',
        output='screen',
        parameters=path_follower_params,
        condition=IfCondition(launch_path_follower)
    ))


    # 6c. Nav Simulator (Conditional)
    nodes.append(Node(
        package='path_follower',
        executable='nav_simulator.py',
        output='screen',
        parameters=[
            {'turn_sequence': ['left', 'right', 'straight']},
            {'loop_sequence': True},
            {'force_track_detect': force_track_detect},
            {'track_detect_stable_ms': ParameterValue(track_detect_stable_ms, value_type=int)}
        ],
        condition=IfCondition(launch_nav_simulator)
    ))

    # 7. Mode-specific launches
    if mode == 'HARDWARE':
        # Include roboteq_driver
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(driver_dir, 'launch', 'roboteq_driver.launch.py'))
        ))
        # Launch plc_interface
        nodes.append(Node(
            package='plc_interface',
            executable='plc_interface_node',
            output='screen',
            parameters=[{
                'plc_ip': plc_ip,
                'plc_port': ParameterValue(plc_port, value_type=int)
            }]
        ))
    elif mode == 'SIM':
        if sim_tool == 'MUJOCO':
            nodes.append(LogInfo(msg="Launching Mujoco Simulator..."))
        elif sim_tool == 'GZ':
            nodes.append(LogInfo(msg="Launching Gazebo Simulator..."))
        else:
            nodes.append(LogInfo(msg=f"Unknown SIM_TOOL: {sim_tool}"))

    return LaunchDescription(nodes)
