import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 패키지 경로 가져오기
    pkg_four_wheel_robot = get_package_share_directory("four_wheel_robot")
    xacro_file = os.path.join(pkg_four_wheel_robot, "urdf", "robot_1.xacro")

    # robot_description을 xacro 파일에서 직접 불러오기 (변환 없이)
    robot_description_config = xacro.process_file(xacro_file)
    params = {"robot_description": robot_description_config.toxml(), "use_sim_time": use_sim_time}

    # Gazebo 실행
    world_file = os.path.join(pkg_four_wheel_robot, "worlds", "empty.world")  # 필요에 따라 world 파일 경로 수정
    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", world_file, "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    # Gazebo에 로봇 스폰
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "four_wheel_robot"],
        output="screen",
    )

    # robot_state_publisher 실행
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time"),
            gazebo,
            robot_state_publisher,
            spawn_entity,
        ]
    )
