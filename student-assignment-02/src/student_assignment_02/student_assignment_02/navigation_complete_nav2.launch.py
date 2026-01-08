#!/usr/bin/env python3
"""
Navigation Launch File - Kompletna Nav2 + A* + Adapter
Klassic DWB Controller + Navigation

FIX: Dodan Lifecycle Manager koji aktivira controller_server!
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
    # LIFECYCLE MANAGER - AKTIVIRA CONTROLLER_SERVER!
    # =====================================================================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager_standalone',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},  # KLJUČNO: automatski pokreće
            {'node_names': ['controller_server', 'local_costmap']},
        ]
    )
    
    # =====================================================================
    # LOCAL COSTMAP
    # =====================================================================
    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='costmap_2d_node',
        name='local_costmap',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'always_send_full_costmap': True},
            {'footprint': '[[0.15, 0.15], [0.15, -0.15], [-0.15, -0.15], [-0.15, 0.15]]'},
            {'global_frame': 'map'},
            {'robot_base_frame': 'base_link'},
            {'update_frequency': 10.0},
            {'publish_frequency': 10.0},
            {'width': 30},
            {'height': 30},
            {'resolution': 0.05},
            {'robot_radius': 0.15},
            {'inflation_radius': 0.3},
        ]
    )
    
    # =====================================================================
    # CONTROLLER SERVER - DWB CONTROLLER
    # =====================================================================
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'controller_frequency': 10.0},
            {'min_x_velocity_threshold': 0.001},
            {'min_y_velocity_threshold': 0.5},
            {'min_theta_velocity_threshold': 0.001},
            {'failure_tolerance': 0.3},
            {'progress_checker_plugin': 'progress_checker'},
            {'goal_checker_plugins': ['general_goal_checker']},
            {'controller_plugins': ['FollowPath']},
            
            # Progress Checker
            {'progress_checker': {
                'plugin': 'nav2_core::SimpleProgressChecker',
                'required_movement_radius': 0.5,
                'movement_time_allowance': 10,
            }},
            
            # Goal Checker
            {'general_goal_checker': {
                'plugin': 'nav2_core::SimpleGoalChecker',
                'xy_goal_tolerance': 0.25,
                'yaw_goal_tolerance': 0.25,
                'stateful': True,
            }},
            
            # DWB Controller
            {'FollowPath': {
                'plugin': 'dwb_core::DWBLocalPlanner',
                'debug_trajectory_details': True,
                'min_vel_x': -0.5,
                'min_vel_y': 0.0,
                'max_vel_x': 0.5,
                'max_vel_y': 0.0,
                'max_vel_theta': 1.0,
                'min_speed_xy': 0.0,
                'max_speed_xy': 0.5,
                'min_speed_theta': 0.0,
                'acc_lim_x': 2.5,
                'acc_lim_y': 0.0,
                'acc_lim_theta': 3.2,
                'decel_lim_x': 2.5,
                'decel_lim_y': 0.0,
                'decel_lim_theta': 3.2,
                'vx_samples': 20,
                'vy_samples': 5,
                'vtheta_samples': 20,
                'sim_time': 1.7,
                'linear_granularity': 0.05,
                'angular_granularity': 0.025,
                'transform_tolerance': 0.2,
                'xy_goal_tolerance': 0.25,
                'trans_stopped_velocity': 0.25,
                'short_circuit_trajectory_evaluation': True,
                'stateful': True,
                'critics': [
                    'RotateToGoal',
                    'Oscillation',
                    'BaseObstacle',
                    'GoalAlign',
                    'PathAlign',
                    'PathDist',
                    'GoalDist'
                ],
                'BaseObstacle': {
                    'scale': 0.02,
                    'max_array_size': 400,
                },
                'PathAlign': {
                    'scale': 32.0,
                    'forward_point_distance': 0.1,
                },
                'GoalAlign': {
                    'scale': 24.0,
                    'forward_point_distance': 0.1,
                },
                'PathDist': {
                    'scale': 32.0,
                },
                'GoalDist': {
                    'scale': 24.0,
                },
                'RotateToGoal': {
                    'scale': 32.0,
                    'slowing_factor': 5.0,
                    'lookahead_time': -1,
                },
                'Oscillation': {
                    'scale': 1.0,
                },
            }},
        ]
    )
    
    # =====================================================================
    # A* PATH PLANNER
    # =====================================================================
    astar_planner = Node(
        package='student_assignment_02',
        executable='astar_path_planner_node',
        name='a_star_path_planner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'inflation_distance_m': 0.5},
        ]
    )
    
    # =====================================================================
    # NAV2 ADAPTER
    # =====================================================================
    nav2_adapter = Node(
        package='student_assignment_02',
        executable='nav2_adapter_node',
        name='nav2_adapter',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
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
        
        # REDOSLIJED JE VAŽAN!
        # 1. Lifecycle manager - aktivira servere
        lifecycle_manager,
        
        # 2. Local costmap
        local_costmap,
        
        # 3. Controller server (sa svim parametrima)
        controller_server,
        
        # 4. A* planer
        astar_planner,
        
        # 5. Adapter
        nav2_adapter,
    ])
    
    return ld
