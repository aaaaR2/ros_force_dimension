"""Launch the Force Dimension node with synchronized DeviceState publishing.

This launch file demonstrates the new DeviceState message with selective
metric publishing for performance optimization.

Reference:
  https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html

"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for Force Dimension node with DeviceState."""

    # Get package directory
    pkg_dir = get_package_share_directory('force_dimension')
    config_file = os.path.join(pkg_dir, '..', '..', '..', 'src',
                                'force_dimension', 'config', 'device_state.yaml')

    # Force Dimension server node with DeviceState configuration
    server_node = Node(
        package="force_dimension",
        executable="node",
        parameters=[config_file],
        emulate_tty=True
    )

    # Diagnostic command to echo the new DeviceState topic
    echo_state_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/robot/feedback/state'],
        output='screen'
    )

    # Diagnostic command to show DeviceState message info
    info_state_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'info', '/robot/feedback/state'],
        output='screen'
    )

    # Delay the echo commands to allow node to start
    delayed_echo_state = TimerAction(
        period=1.0,
        actions=[echo_state_cmd]
    )

    delayed_info_state = TimerAction(
        period=0.5,
        actions=[info_state_cmd]
    )

    return LaunchDescription([
        server_node,
        delayed_info_state,
        # delayed_echo_state  # Uncomment to see live DeviceState messages
    ])
