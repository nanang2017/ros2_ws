from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gazebo_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install urdf folder
        (os.path.join('share', package_name, 'vision60'), glob('vision60/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kisangpark',
    maintainer_email='kisangpark@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'controller = gazebo_tutorial.controller:main',
        ],
    },
)


"""

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install urdf folder
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # Install launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='My package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)


"""