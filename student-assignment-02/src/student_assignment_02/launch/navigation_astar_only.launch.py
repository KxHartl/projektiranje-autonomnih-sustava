#!/usr/bin/env python3
"""
Navigation Launch File - SAMO A* PATH PLANNER + NAV2 ADAPTER

BEZ SVEGA DRUGOG!
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
    # A* PATH PLANNER NODE
    # =====================================================================
    astar_planner = Node(
        package='student_assignment_02',
        executable='astar_path_planner',
        name='a_star_path_planner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'allow_diagonal': True},
            {'inflation_distance_m': 0.5},
            {'max_iterations': 50000},
        ]
    )
    
    # =====================================================================
    # NAV2 ADAPTER NODE
    # =====================================================================
    nav2_adapter = Node(
        package='student_assignment_02',
        executable='nav2_adapter',
        name='nav2_adapter',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'linear_speed': 0.3},
            {'angular_speed': 1.0},
            {'distance_tolerance': 0.15},
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
        
        # Nav2 adapter - sljedi putanju
        nav2_adapter,
    ])
    
    return ld
