import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Fetch package directories
    kinematics_share = get_package_share_directory('kinematics')
    
    # 2. Declare launch arguments
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic to subscribe for velocity commands'
    )
    
    # 3. Fetch environment variables (opt_env logic)
    wheel_base = os.environ.get('WHEEL_BASE', '0.512')
    wheel_radius = os.environ.get('WHEEL_RAD', '0.08')
    gear_ratio = os.environ.get('GEAR_RATIO', '1.0')
    ticks_per_rev = os.environ.get('TICKS_PER_REV', '2048.0')
    can_interface = os.environ.get('CAN_INTERFACE', 'can0')
    run_mode = os.environ.get('RUN_MODE', 'simulation')
    
    print(f"============================================================")
    # Highlight env parameters
    print(f"Kinematics Launch Env Configuration:")
    print(f"  WHEEL_BASE:    {wheel_base}")
    print(f"  WHEEL_RAD:     {wheel_radius}")
    print(f"  GEAR_RATIO:    {gear_ratio}")
    print(f"  TICKS_PER_REV: {ticks_per_rev}")
    print(f"  CAN_INTERFACE: {can_interface}")
    print(f"  RUN_MODE:      {run_mode}")
    print(f"============================================================")
    
    # 4. Generate URDF content by replacing placeholders in robot.urdf
    urdf_template_path = os.path.join(kinematics_share, 'urdf', 'robot.urdf')
    with open(urdf_template_path, 'r') as f:
        urdf_content = f.read()
        
    urdf_content = urdf_content.replace('${CAN_INTERFACE}', can_interface)
    urdf_content = urdf_content.replace('${WHEEL_RADIUS}', wheel_radius)
    urdf_content = urdf_content.replace('${WHEEL_BASE}', wheel_base)
    urdf_content = urdf_content.replace('${GEAR_RATIO}', gear_ratio)
    urdf_content = urdf_content.replace('${TICKS_PER_REV}', ticks_per_rev)
    urdf_content = urdf_content.replace('${RUN_MODE}', run_mode)
    
    # 5. Load and dynamically override controllers config
    controllers_template_path = os.path.join(kinematics_share, 'config', 'controllers.yaml')
    with open(controllers_template_path, 'r') as f:
        controllers_config = yaml.safe_load(f)
        
    # Update wheel separation and radius based on environment parameters
    controllers_config['diff_drive_controller']['ros__parameters']['wheel_separation'] = float(wheel_base)
    controllers_config['diff_drive_controller']['ros__parameters']['wheel_radius'] = float(wheel_radius)
    
    # Write to a modified configuration file inside package directory
    modified_config_path = os.path.join(kinematics_share, 'config', 'modified_controllers.yaml')
    os.makedirs(os.path.dirname(modified_config_path), exist_ok=True)
    with open(modified_config_path, 'w') as f:
        yaml.safe_dump(controllers_config, f)
        
    # 6. Define nodes
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            modified_config_path,
            {'robot_description': urdf_content}
        ],
        output='screen'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content}]
    )
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-ros-args', '--ros-args -r /diff_drive_controller/cmd_vel:=/cmd_vel -r /diff_drive_controller/odom:=/odom/raw'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        cmd_vel_topic_arg,
        controller_manager,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])
