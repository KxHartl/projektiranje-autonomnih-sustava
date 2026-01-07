from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'student_assignment_02'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),
        ('share/' + package_name + '/config',
            glob('config/*.rviz')),
        ('share/' + package_name + '/world',
            glob('world/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khartl',
    maintainer_email='kh239762@fsb.hr',
    description='ROS2 Python package for autonomous system mapping and path planning',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning_node = student_assignment_02.path_planning_node:main',
            'goal_navigation_node = student_assignment_02.goal_navigation_node:main',
        ],
    },
)
