# Copyright (c) 2024 Harun Teper
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Gazebo world launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Package Directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Paths to folders and files
    world = os.path.join(get_package_share_directory('fp_gazebo'), 'worlds', str(
        context.launch_configurations['world_name'])+'.world')

    # Nodes and other launch files
    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gazebo_ros_state': 'true', 
        }.items()
    )

    # Create launch description actions
    launch_description_content = []
    launch_description_content.append(gazebo_launch_cmd)

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Launch Arguments
    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack_decorated',
        description='Gazebo world file name in /worlds folder'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(world_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
