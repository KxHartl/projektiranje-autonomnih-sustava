#!/usr/bin/env python3
"""
Navigation Launch File - SAMO A* + PATH FOLLOWER

Koristi nove SIMPLE nodove koje su pisane od nule
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('student_assignment_02')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # =====================================================================
    # SIMPLE A* PATH PLANNER
    # =====================================================================
    astar_planner = Node(
        package='student_assignment_02',
        executable='simple_astar_planner',
        name='simple_astar_planner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )
    
    # =====================================================================
    # SIMPLE PATH FOLLOWER
    # =====================================================================
    path_follower = Node(
        package='student_assignment_02',
        executable='simple_path_follower',
        name='simple_path_follower',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'linear_speed': 0.3},
            {'angular_speed': 1.0},
            {'distance_tolerance': 0.1},
        ]
    )
    
    # =====================================================================
    # LAUNCH DESCRIPTION
    # =====================================================================
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Koristi simulacijsko vrijeme'
        ),
        
        # A* planer - planira putanju
        astar_planner,
        
        # Path follower - sljedi putanju
        path_follower,
    ])
    
    return ld
