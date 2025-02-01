"""
launching only gazebo & urdf/xacro file
for joint state publisher gui & checking file
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
import xacro

def generate_launch_description():
    #get xacro description
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_tutorial"), "launch", "xacro_load.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    #gazebo ros package load
    pkg_gazebo_ros=os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
    #pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    #pkg_gazebo_ros = get_package_share_directory('gazebo_ros').find('gazebo_ros')

    # start gazebo
    gazebo_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_gazebo_ros]),
        #launch_arguments = {'world': world_path}.items()
    )

    #workspace package
    pkg_path = os.path.join(get_package_share_directory('gazebo_tutorial'))

    #loading world
    world_path = os.path.join(pkg_path, 'worlds', 'empty.world')



    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'vision60',
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription(
        [
            robot,
            gazebo_start,
            spawn_entity,
            #rviz,
        ]
    )
