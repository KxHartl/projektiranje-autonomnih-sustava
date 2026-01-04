#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Complete SLAM mapping launch file
Pokreće sve komponente: Stage simulator, SLAM toolbox, RViz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Tvoj paket
    student_share = get_package_share_directory('student_assignment_02')
    student_config_dir = os.path.join(student_share, 'config')
    launch_dir = os.path.join(student_share, 'launch')

    # Argumenti
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------------------------
    # Include Stage launch
    # ------------------------------------------------------------------
    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'stage_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ------------------------------------------------------------------
    # Include SLAM launch
    # ------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ------------------------------------------------------------------
    # RViz configuration loader
    # ------------------------------------------------------------------
    rviz_config = os.path.join(student_config_dir, 'rviz_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ------------------------------------------------------------------
    # Marker publisher za visualizaciju A* pretrage
    # ------------------------------------------------------------------
    marker_node = Node(
        package='student_assignment_02',
        executable='path_planning_node',
        name='path_planning',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription([
        declare_use_sim_time,
        stage_launch,
        slam_launch,
        rviz_node,
        # marker_node,  # Uncomment kada kreirate path planning čvor
    ])

    return ld
