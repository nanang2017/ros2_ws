"""
controller launch
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

    #control signal to joint_state_publisher
    params = {"source_list": ["control_signal"]}
    #bridge_param = {"name": "config_file", "value": "ros2_ws/src/gazebo_tutorial/bridge.yaml"}

    #launch file get
    no_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_tutorial"), "launch", "no_control.launch.py")]
        ),
    )

    #joint state publisher launch
    joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    parameters = [params],
    )

    #controller launch
    controller_node = Node(
        package='gazebo_tutorial',
        executable='controller',
        arguments=[],
        output='screen'
    )

    #controller manager
    #ros2 run controller_manager ros2_control_node
    #controller_manager = Node(
    #package='controller_manager',
    #executable='ros2_control_node',
    #)

    #gz bridge
    #ros2 run ros_gz_bridge parameter_bridge
    #bridge = Node(
    #package='ros_gz_bridge',
    #executable='parameter_bridge',
    #name="gz_bridge",
    #output='screen',
    #parameters=[],
    #)

    return LaunchDescription(
        [
            no_control,
            joint_state_publisher,
            controller_node,
            #bridge,
            #controller_manager,
        ]
    )

