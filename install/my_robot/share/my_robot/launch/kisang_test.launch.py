import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "my_robot"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "robot_2.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    world_file = os.path.join(os.environ['HOME'], 'ros2_ws/src/my_robot/camera.world')
    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", world_file, "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot", "-x", "3", "-y", "2", "-z", "10"],
        output="screen",
    )

    #launch teleop keyboard
    #ros2 run teleop_twist_keyboard teleop_twist_keyboard


    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
        ]
    )