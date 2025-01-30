from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)

from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('fp_localization'),
        'config',
        'amcl.yaml'

    )

    map_pgm_file = os.path.join(
        get_package_share_directory('fp_localization'),
        'map',
        'mapmap.pgm'
    )

    map_file = os.path.join(
        get_package_share_directory('fp_localization'),
        'map',
        'mapmap.yaml'
    )

    params_file = os.path.join(
        get_package_share_directory('fp_localization'),
        'config',
        'nav2_params.yaml'
    )

    amcl_params_file = os.path.join(
        get_package_share_directory('fp_localization'),
        'config',
        'amcl.yaml'
    )

    map_server_params_file = os.path.join(
        get_package_share_directory('fp_localization'),
        'config',
        'map_server.yaml'
    )

    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    namespace = LaunchConfiguration('namespace', default='robot')
    map_yaml_file = LaunchConfiguration('map', default=map_file)
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')
    params_file = LaunchConfiguration('params_file', default=params_file)
    log_level = LaunchConfiguration('log_level', default='INFO')
    namespace = LaunchConfiguration('namespace', default='robot')

    lifecycle_nodes = ['map_server', 'amcl']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', '/robot/tf'),
                  ('/tf_static', '/robot/tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            namespace=namespace),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[map_server_params_file],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params_file],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(bringup_cmd_group)
    # ld.add_action(map_server_node)
    # ld.add_action(amcl_node)
    # ld.add_action(lifecycle_node)

    # # Declare the launch options
    # ld.add_action(declare_namespace_cmd)
    # ld.add_action(declare_map_yaml_cmd)
    # ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_params_file_cmd)
    # ld.add_action(declare_autostart_cmd)
    # ld.add_action(declare_use_composition_cmd)
    # ld.add_action(declare_container_name_cmd)
    # ld.add_action(declare_use_respawn_cmd)
    # ld.add_action(declare_log_level_cmd)

    # # Add the actions to launch all of the localiztion nodes
    # ld.add_action(load_nodes)
    # ld.add_action(load_composable_nodes)

    return ld
