"""
diff_drive.launch.py
─────────────────────
Launches the complete ros2_control stack for the diff-drive robot.

Start order (enforced with event handlers):
  1. robot_state_publisher       – parses URDF, publishes static TF
  2. ros2_control_node           – controller_manager (loads our HW plugin)
  3. joint_state_broadcaster     – spawned and activated
  4. diff_drive_controller       – spawned and activated AFTER #3 exits cleanly
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_drive_hardware')
    urdf_file  = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    ctrl_yaml  = os.path.join(pkg_share, 'config', 'controllers.yaml')

    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    robot_description = {'robot_description': robot_description_content}

    # ── 1. robot_state_publisher ─────────────────────────────────────────────
    # Reads the URDF and publishes the fixed joint TF tree (base_link → wheels,
    # caster, etc.). Also re-publishes /joint_states (from joint_state_broadcaster)
    # as TF for the continuous joints.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ── 2. controller_manager (ros2_control_node) ────────────────────────────
    # This is the heart of the ros2_control system.  It:
    #   • loads our DiffDriveHardware plugin (from the URDF <plugin> tag)
    #   • calls on_init / on_configure / on_activate on it
    #   • runs the control loop at update_rate Hz, calling read() then write()
    #   • manages the lifecycle of all controllers
    #
    # Parameters: robot_description (the URDF) + controllers.yaml
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ctrl_yaml],
        output='screen',
    )
    
    # ── twist_stamper ─────────────────────────────────────────────────────────
    # The Jazzy diff_drive_controller enforces TwistStamped; teleop publishes
    # plain Twist. This node bridges the two by adding a header stamp.
    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        output='screen',
        parameters=[{'use_sim_time': False, 'frame_id': 'base_link'}],
        remappings=[
            ('/cmd_vel_in',  '/cmd_vel'),
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ],
    )


    # ── 3. Spawn joint_state_broadcaster ─────────────────────────────────────
    # "spawner" is a helper that calls the controller_manager service to load
    # and activate a controller. It exits cleanly on success.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # ── 4. Spawn diff_drive_controller AFTER joint_state_broadcaster ─────────
    # We register an event handler that waits for the joint_state_broadcaster
    # spawner process to exit (which means it succeeded) before starting the
    # diff_drive_controller spawner.
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    delay_diff_drive_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        twist_stamper_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive_after_jsb,
    ])
