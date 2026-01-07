#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Online Asynchronous SLAM Launch File
Koristi slam_toolbox za mapiranje okoline u realnom vremenu

FIX: Laser frame mora biti dostupan SLAM-u
Stage koristi /base_scan s frame_id='laser'
Trebamo transform: laser -> base_link (CORRECT DIRECTION!)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory("student_assignment_02"),
            'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # Stage koristi 'laser' frame za /base_scan
    # Trebamo transform: laser -> base_link (omogućava SLAM-u da transformira laser podatke)
    laser_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        # Format: x y z roll pitch yaw qx qy qz qw parent child
        # Laser je 0.15m IZNAD base_link-a
        # Trebamo transform koji ide: laser -> base_link (laser je parent, base_link je child)
        arguments=['0', '0', '-0.15', '0', '0', '0', '1', 'laser', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Asynchronous SLAM Toolbox Node
    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        emulate_tty=True,
        remappings=[
            # Stage koristi /base_scan s frame_id='laser'
            ('/scan', '/base_scan'),
            # TF topics
            ('tf', 'tf'),
            ('tf_static', 'tf_static'),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(laser_to_base_link_tf)  # PRVO objavljujemo TF!
    ld.add_action(start_async_slam_toolbox_node)  # ZATIM SLAM

    return ld
