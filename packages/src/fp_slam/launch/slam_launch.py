from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Declare argument for RViz config
        DeclareLaunchArgument(
            'rviz_launch_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('fp_slam'),
                'config',
                'slam_visualization.rviz'
            ]),
            description='Path to the RViz config file'
        ),

        # SLAM Toolbox node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('fp_slam'),
                'config',
                'slam.yaml'
            ])],
            remappings=[
                ('/tf', '/robot/tf'),
                ('/tf_static', '/robot/tf_static'),
                ('/scan', '/robot/scan')
            ]
        ),

        # RViz node
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
