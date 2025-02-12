import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. URDF 파일 읽기 (omni_robot 패키지 내의 urdf/rs_robot.urdf)
    package_name = 'omni_robot'
    urdf_file_name = 'rs_robot.urdf'
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', urdf_file_name)
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 2. Gazebo 런치 파일 포함하기
    # gazebo_ros 패키지 내의 gazebo.launch.py 파일을 포함하여 Gazebo를 실행합니다.
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        # world 인자를 전달하여 기본 world(예: empty.world)를 실행할 수 있습니다.
        launch_arguments={'world': os.path.join(gazebo_ros_share, 'worlds', 'empty.world')}.items()
    )

    # 3. robot_state_publisher 노드 실행 (URDF를 파라미터로 전달)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Gazebo 시뮬레이션 시간 사용
            'robot_description': robot_desc  # 읽은 URDF 문자열
        }]
    )

    # 4. spawn_entity 노드를 사용하여 Gazebo에 로봇 스폰
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'omni_robot',       # Gazebo에서 생성될 로봇 이름
            '-topic', 'robot_description',    # robot_state_publisher가 publish한 URDF를 사용
            '-x', '0',
            '-y', '0',
            '-z', '4'
        ],
        output='screen'
    )

    # 최종 LaunchDescription에 모두 포함합니다.
    return LaunchDescription([
        gazebo,
        rsp_node,
        spawn_entity
    ])
