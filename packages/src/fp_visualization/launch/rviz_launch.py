from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Argument für den Konfigurationspfad deklarieren
        DeclareLaunchArgument(
            'rviz_launch_config',
            default_value='/home/vscode/workspace/packages/src/fp_visualization/rviz/default.rviz',
            description='Path to the RViz config file'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_launch_config')],
            remappings=[
                ('/tf', '/robot/tf'),
                ('/tf_static', '/robot/tf_static')
            ]
        )
    ])
