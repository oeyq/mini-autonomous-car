# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Get the launch directory
    nav2_dir = get_package_share_directory('fp_waypoints')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    waypoint_file = LaunchConfiguration('waypoint_file')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Top-level namespace')
    
    declare_waypoint_file_cmd = DeclareLaunchArgument(
        'waypoint_file',
        default_value=os.path.join(nav2_dir, 'waypoints', 'waypoints.yaml'),
        description='Full path to waypoint yaml file to load')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(namespace=namespace),

        Node(
            name='fp_waypoint_publisher',
            package='fp_waypoints',
            executable='waypoint_publisher',
            parameters=[{'waypoint_file': waypoint_file}],
            remappings=remappings,
            output='screen'),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_waypoint_file_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
