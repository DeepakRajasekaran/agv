import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Directories
    driver_dir = get_package_share_directory('roboteq_driver')
    desc_dir = get_package_share_directory('robot_description')

    # Files
    params_file = os.path.join(driver_dir, 'config', 'params.yaml')
    controllers_file = os.path.join(driver_dir, 'config', 'controllers.yaml')
    urdf_file = os.path.join(desc_dir, 'urdf', 'robot.urdf.xacro')

    # Process Xacro
    doc = xacro.process_file(urdf_file)
    robot_description = {'robot_description': doc.toxml()}

    # 1. Standalone Roboteq CAN Driver Node
    roboteq_driver_node = Node(
        package='roboteq_driver',
        executable='roboteq_driver_node',
        name='roboteq_driver_node',
        output='screen',
        parameters=[params_file]
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 3. ros2_control Node (Hardware Interface Manager)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='screen'
    )

    # 4. Spawners for Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        roboteq_driver_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])
