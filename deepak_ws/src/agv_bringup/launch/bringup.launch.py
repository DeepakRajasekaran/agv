import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Define launch arguments
    launch_path_follower_arg = DeclareLaunchArgument(
        'launch_path_follower',
        default_value='true',
        description='Launch the path_follower node'
    )
    
    launch_path_follower = LaunchConfiguration('launch_path_follower')

    # Read Environment Variables
    mode = os.environ.get('MODE', 'HARDWARE').upper()
    sim_tool = os.environ.get('SIM_TOOL', 'MUJOCO').upper()
    
    # Parse directories
    driver_dir = get_package_share_directory('roboteq_driver')
    desc_dir = get_package_share_directory('robot_description')

    # Files
    controllers_file = os.path.join(driver_dir, 'config', 'controllers.yaml')
    urdf_file = os.path.join(desc_dir, 'urdf', 'robot.urdf.xacro')

    # Process Xacro
    doc = xacro.process_file(urdf_file, mappings={'mode': mode, 'sim_tool': sim_tool})
    robot_description = {'robot_description': doc.toxml()}

    nodes = [launch_path_follower_arg]

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

    # 4. Twist to TwistStamped Relay Node
    nodes.append(Node(
        package='twist_stamper',
        executable='twist_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
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

    nodes.append(Node(
        package='path_follower',
        executable='path_follower_node',
        output='screen',
        parameters=path_follower_params,
        condition=IfCondition(launch_path_follower)
    ))

    # 7. Mode-specific launches
    if mode == 'HARDWARE':
        # Include roboteq_driver
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(driver_dir, 'launch', 'roboteq_driver.launch.py'))
        ))
    elif mode == 'SIM':
        if sim_tool == 'MUJOCO':
            nodes.append(LogInfo(msg="Launching Mujoco Simulator..."))
        elif sim_tool == 'GZ':
            nodes.append(LogInfo(msg="Launching Gazebo Simulator..."))
        else:
            nodes.append(LogInfo(msg=f"Unknown SIM_TOOL: {sim_tool}"))

    return LaunchDescription(nodes)
