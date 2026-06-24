from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='SocketCAN interface name'
    )
    node_id_arg = DeclareLaunchArgument(
        'node_id', default_value='5',
        description='CANopen Node ID for MGS1600'
    )

    mgs_node = Node(
        package='mgs_driver',
        executable='mgs_driver_node',
        name='mgs_driver_node',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'node_id': LaunchConfiguration('node_id'),
        }]
    )

    return LaunchDescription([
        can_interface_arg,
        node_id_arg,
        mgs_node
    ])
