from setuptools import setup
from glob import glob

package_name = 'omni_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        # meshes 폴더 하위의 각 폴더를 따로 지정
        ('share/' + package_name + '/meshes/DAE/base', glob('meshes/DAE/base/*.stl')),
        ('share/' + package_name + '/meshes/DAE/omni_wheel', glob('meshes/DAE/omni_wheel/*.dae')),
        ('share/' + package_name + '/meshes/DAE/roller', glob('meshes/DAE/roller/*.dae')),
        ('share/' + package_name + '/meshes/STL/base', glob('meshes/STL/base/*.stl')),
        ('share/' + package_name + '/meshes/STL/omni_wheel', glob('meshes/STL/omni_wheel/*.stl')),
        ('share/' + package_name + '/meshes/STL/roller', glob('meshes/STL/roller/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 예: 'node_executable = omni_robot.some_module:main'
        ],
    },
)
