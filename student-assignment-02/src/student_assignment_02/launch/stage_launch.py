#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Tvoj paket
    student_share = get_package_share_directory('student_assignment_02')
    student_config_dir = os.path.join(student_share, 'config')

    # Argumenti
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='map_01',
        description='World file name (bez .world)'
    )

    declare_use_stage = DeclareLaunchArgument(
        'stage',
        default_value='true',
        description='Start Stage simulator'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz'
    )

    use_stage = LaunchConfiguration('stage')
    use_rviz = LaunchConfiguration('rviz')

    # ------------------------------------------------------------------
    # STAGE
    # ------------------------------------------------------------------
    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage',
        condition=IfCondition(use_stage),
        parameters=[{
            'world_file': os.path.join(
                student_share, 
                'world', 
                'map_01.world'
            ),
            'enforce_prefixes': True,
            'use_stamped_velocity': False,
            'use_static_transformations': True,
        }],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        declare_world,
        declare_use_stage,
        declare_use_rviz,
        stage_node,
    ])
