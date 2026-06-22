import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    kinematics_share = get_package_share_directory('kinematics')
    kinematics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kinematics_share, 'launch', 'kinematics.launch.py')
        ),
        launch_arguments={'cmd_vel_topic': '/cmd_vel'}.items()
    )

    run_mode = os.environ.get('RUN_MODE', 'simulation').lower()
    
    launch_entities = [headless_arg]
    
    if run_mode in ['simulation', 'sim']:
        print("Simulation mode detected. Launching hardware_sim_node.")
        launch_entities.append(
            # Run Simulator
            Node(
                package='hardware_sim',
                executable='hardware_sim_node.py',
                name='hardware_sim_node',
                parameters=[{'headless': LaunchConfiguration('headless')}],
                output='screen'
            )
        )
    else:
        print("Hardware mode detected. hardware_sim_node will NOT be launched.")

    launch_entities.extend([
        # Run ros2_control diff_drive_controller
        kinematics_launch,
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

    return LaunchDescription(launch_entities)
