#gazebo launch file

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spaceros_gz_sim = get_package_share_directory('spaceros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = PathJoinSubstitution([pkg_spaceros_gz_sim, 'models'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='moon',
            choices=['moon', 'mars', 'enceladus'],
            description='World to load into Gazebo'
        ),
        SetLaunchConfiguration(name='world_file', 
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.sdf')]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([pkg_spaceros_gz_sim, 'worlds',
                                                  LaunchConfiguration('world_file')])],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
    ])



"""
or, another using gz_server

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ros_gz_sim.actions import GzServer


def generate_launch_description():

    declare_world_sdf_file_cmd = DeclareLaunchArgument(
        'world_sdf_file', default_value='',
        description='Path to the SDF world file')

    # Create the launch description and populate
    ld = LaunchDescription([
        GzServer(
            world_sdf_file=LaunchConfiguration('world_sdf_file')
        ),
    ])

    # Declare the launch options
    ld.add_action(declare_world_sdf_file_cmd)

    return ld


"""