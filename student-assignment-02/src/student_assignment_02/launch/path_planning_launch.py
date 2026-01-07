#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Planning and Navigation Launch File
Pokreće Stage simulator sa A* path planning i goal navigation 
na već spravljenoj mapi

Usage:
    ros2 launch student_assignment_02 path_planning_launch.py
    ros2 launch student_assignment_02 path_planning_launch.py world:=map_01
    ros2 launch student_assignment_02 path_planning_launch.py map_file:=~/my_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    student_share = get_package_share_directory('student_assignment_02')
    config_dir = os.path.join(student_share, 'config')
    launch_dir = os.path.join(student_share, 'launch')
    
    # Launch arguments
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='map_01',
        description='World file name (bez .world extension)'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(student_share, 'data', 'maps', 'map'),
        description='Path to map file (bez .yaml extension)'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    world_arg = LaunchConfiguration('world')
    map_file_arg = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Include Stage launch
    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'stage_launch.py')
        ),
        launch_arguments={
            'world': world_arg,
            'use_sim_time': use_sim_time,
            'rviz': 'true',
        }.items()
    )
    
    # Map server node - loads pre-recorded map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file_arg,
        }],
        remappings=[
            ('/map', '/map'),
        ]
    )
    
    # Path Planning Node - A* algorithm
    path_planning_node = Node(
        package='student_assignment_02',
        executable='path_planning_node',
        name='path_planning',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Goal Navigation Node - follows planned path
    goal_navigation_node = Node(
        package='student_assignment_02',
        executable='goal_navigation_node',
        name='goal_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'linear_speed': 0.2,
            'angular_speed': 0.5,
            'waypoint_tolerance': 0.15,
            'angle_tolerance': 0.2,
        }]
    )
    
    ld = LaunchDescription([
        declare_world,
        declare_map_file,
        declare_use_sim_time,
        stage_launch,
        map_server_node,
        path_planning_node,
        goal_navigation_node,
    ])
    
    return ld
