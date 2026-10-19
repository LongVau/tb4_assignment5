"""ROS 2 ament_python package definition."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tb4_assignment5'

setup(
    name=package_name,
    version='1.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.xml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='sumika',
    maintainer_email='sumika@todo.todo',
    description='TurtleBot 4 practical assignment 5: AMCL, 3 Nav2 goals, YOLO',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navigate_goals = tb4_assignment5.navigation_node:main',
            'detect_objects = tb4_assignment5.yolo_node:main',
            'record_points = tb4_assignment5.point_logger:main',
            'check_tf_health = tb4_assignment5.tf_health_node:main',
        ],
    },
)
