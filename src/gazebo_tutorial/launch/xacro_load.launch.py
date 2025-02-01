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
    #use sim time
    use_sim_time = LaunchConfiguration("use_sim_time")

    #workspace package
    pkg_path = os.path.join(get_package_share_directory('gazebo_tutorial'))

    #trying xacro instead of urdf!
    xacro_file = os.path.join(pkg_path, "vision60", "vision60_double.xacro")
    robot_description = xacro.process_file(xacro_file)
    params = {"robot_description": robot_description.toxml(), "use_sim_time": use_sim_time}

    #robot state publisher
    robot_state_publisher = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],)


    return LaunchDescription(
        [
            DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="use sim time"
            ),
            robot_state_publisher,
        ]
    )

"""
    #load robot (read urdf file, create parameters dictionary, create state publisher node)
    urdf_file = os.path.join(pkg_path, "vision60", "vision60_double.urdf")
    with open(urdf_file, "r") as file:
        robot_description_content = file.read()
    params = {"robot_description": robot_description_content}
"""