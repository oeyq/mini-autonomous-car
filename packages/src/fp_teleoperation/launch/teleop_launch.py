from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fp_teleoperation',
            executable='keyboard_controller',
            name='teleop_node',
            output='screen'
        )
    ])
