import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the use_sim_time argument
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Path to the URDF file
    pkg_path = os.path.join(get_package_share_directory("gazebo_tutorial"))
    urdf_file = os.path.join(pkg_path, "vision60", "vision60.urdf")
    # Read the URDF file content
    with open(urdf_file, "r") as file:
        robot_description_content = file.read()
    # Create parameters dictionary
    params = {"robot_description": robot_description_content, "use_sim_time": use_sim_time}

    #create nodes
    argument_node = DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time")
    state_node = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],)

    rviz_node = Node(package = "rviz2",
                        executable = "rviz2",
                        output = "screen",
                        )
    

    # Return the LaunchDescription
    return LaunchDescription(
        [
            argument_node,
            state_node,
            rviz_node,
        ]
    )



"""
achieves
1. robot state publisher successful! -> joint state publisher, controller
2. rviz2 -> docs required
3. topic check -> connection check
4. 
"""



#gazebo launches
"""
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