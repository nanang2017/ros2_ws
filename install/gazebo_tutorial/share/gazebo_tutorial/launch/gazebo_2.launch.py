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
    world_path = os.path.join(pkg_path, 'worlds', 'empty.world')

    #load robot (read urdf file, create parameters dictionary, create state publisher node)
    urdf_file = os.path.join(pkg_path, "vision60", "vision60_double.urdf")
    with open(urdf_file, "r") as file:
        robot_description_content = file.read()
    params = {"robot_description": robot_description_content, "use_sim_time": True}
    params_2 = {"source_list": ["control_signal"], "use_sim_time": True}
    params_3 = {"use_sim_time": True}
    robot_state_publisher = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],)

    #test - joint state publisher
    joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    parameters = [params_2],
    )


    # start gazebo
    gazebo_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments = {'world': world_path, "use_sim_time": True}.items()
    )


    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'vision60',
            #'-x', str(0),
            #'-y', str(0.0),
            #'-Y', str(0.0),
        ],
        output='screen'
    )

    controller_node = Node(
        package='gazebo_tutorial',
        executable='controller',
        arguments=[],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters = [params_3],
    )

    return LaunchDescription(
        [
            gazebo_start,
            robot_state_publisher,
            spawn_entity,
            controller_node,
            #rviz,
            joint_state_publisher,
        ]
    )


"""
achieves
1. robot state publisher successful! -> joint state publisher, controller
2. rviz2 -> docs required
3. topic check -> connection check
4. 
"""

