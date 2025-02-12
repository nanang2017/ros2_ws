from setuptools import setup
import os
from glob import glob

package_name = 'four_wheel_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwan',
    maintainer_email='lih96042015@gmail.com',
    description='Omni-wheel robot package for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_1_controller = four_wheel_robot.robot_1_controller:main'  # 실행 파일 추가
        ],
    },
)
