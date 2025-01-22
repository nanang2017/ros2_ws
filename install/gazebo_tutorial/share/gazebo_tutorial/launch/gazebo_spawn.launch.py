"""
launch description
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #gazebo ros package load
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    #pkg_gazebo_ros = get_package_share_directory('gazebo_ros').find('gazebo_ros')

    #workspace package
    pkg_path = os.path.join(get_package_share_directory('gazebo_tutorial'))

    #loading world
    world_path = os.path.join(pkg_path, 'worlds', 'scene.world')

    #load robot (read urdf file, create parameters dictionary, create state publisher node)
    urdf_file = os.path.join(pkg_path, "vision60", "vision60_single.urdf")
    with open(urdf_file, "r") as file:
        robot_description_content = file.read()
    params = {"robot_description": robot_description_content}
    robot_state_publisher = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],)

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'vision60',
            '-x', str(0),
            '-y', str(0.0),
            '-Y', str(0.0),
        ],
        output='screen'
    )

    controller_node = Node(
        package='gazebo_tutorial',
        executable='controller',
        arguments=[],
        output='screen'
    )

    return LaunchDescription(
        [
            start_gazebo_server_cmd,
            start_gazebo_client_cmd,
            robot_state_publisher,
            spawn_entity,
            controller_node,
        ]
    )


"""
achieves
1. robot state publisher successful! -> joint state publisher, controller
2. rviz2 -> docs required
3. topic check -> connection check
4. 
"""

