import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Read Environment Variables
    mode = os.environ.get('MODE', 'HARDWARE').upper()
    sim_tool = os.environ.get('SIM_TOOL', 'MUJOCO').upper()
    
    # Parse directories
    driver_dir = get_package_share_directory('roboteq_driver')
    desc_dir = get_package_share_directory('robot_description')

    # Files
    controllers_file = os.path.join(driver_dir, 'config', 'controllers.yaml')
    urdf_file = os.path.join(desc_dir, 'urdf', 'robot.urdf.xacro')

    # Process Xacro (it will read optenv directly during xacro processing, 
    # but we can also pass the mode manually as an argument for safety if we wanted to)
    doc = xacro.process_file(urdf_file, mappings={'mode': mode, 'sim_tool': sim_tool})
    robot_description = {'robot_description': doc.toxml()}

    nodes = []

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

    # 4. Twist to TwistStamped Relay Node (because Jazzy diff_drive_controller dropped Twist support)
    nodes.append(Node(
        package='agv_bringup',
        executable='twist_stamper.py',
        output='screen'
    ))

    # 4. Mode-specific launches
    if mode == 'HARDWARE':
        # Include roboteq_driver
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(driver_dir, 'launch', 'roboteq_driver.launch.py'))
        ))
    elif mode == 'SIM':
        if sim_tool == 'MUJOCO':
            nodes.append(LogInfo(msg="Launching Mujoco Simulator..."))
            # E.g., nodes.append(IncludeLaunchDescription(...mujoco_launch...))
        elif sim_tool == 'GZ':
            nodes.append(LogInfo(msg="Launching Gazebo Simulator..."))
            # E.g., nodes.append(IncludeLaunchDescription(...gz_launch...))
        else:
            nodes.append(LogInfo(msg=f"Unknown SIM_TOOL: {sim_tool}"))

    return LaunchDescription(nodes)
