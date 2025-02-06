import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "my_robot"


    world_file = os.path.join(os.environ['HOME'], 'ros2_ws/src/my_robot/test.world')
    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", world_file, "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    #launch teleop keyboard
    #ros2 run teleop_twist_keyboard teleop_twist_keyboard


    # Launch them all!
    return LaunchDescription(
        [
            gazebo,
        ]
    )