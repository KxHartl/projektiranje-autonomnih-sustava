#!/usr/bin/env python3
"""
Navigation Launch File - ULTRA-MINIMALIST

SAMO:
- A* Path Planner
- Nav2 Controller Server (s inline parametrima - bez YAML)
- Nav2 Adapter

BEZ: BT Navigator, Planner Server, Velocity Smoother, sve komplicirane stvari

Redoslijed pokretanja:
1. Terminal 1: Stage simulator - ros2 launch student_assignment_02 stage_launch.py
2. Terminal 2: AMCL - ros2 launch student_assignment_02 localization_complete_launch.py
3. Terminal 3: RViz - ros2 run rviz2 rviz2 -d <config>
4. Terminal 4: OVA DATOTEKA - Navigacija
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file - SAMO A* + minimalan Nav2 Controller"""
    
    # =====================================================================
    # LAUNCH ARGUMENTI
    # =====================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Koristi simulacijsko vrijeme'
    )
    
    inflation_distance_arg = DeclareLaunchArgument(
        'inflation_distance_m',
        default_value='0.5',
        description='Inflation buffer distanca (metri)'
    )
    
    # =====================================================================
    # 1. A* PATH PLANNER NODE
    # =====================================================================
    astar_node = Node(
        package='student_assignment_02',
        executable='a_star_path_planner',
        name='a_star_path_planner',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'allow_diagonal': True,
                'inflation_radius': 1,
                'inflation_distance_m': LaunchConfiguration('inflation_distance_m'),
                'inflation_cost_threshold': 60,
                'max_iterations': 50000,
                'search_radius': -1,
            }
        ],
    )
    
    # =====================================================================
    # 2. NAV2 CONTROLLER SERVER (DWB - inline parametri, BEZ YAML!)
    # =====================================================================
    # OVO JE KRITIČNO - stvara /follow_path akciju
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'controller_frequency': 20.0,
                'min_x_velocity_threshold': 0.001,
                'min_y_velocity_threshold': 0.5,
                'min_theta_velocity_threshold': 0.001,
                'failure_tolerance': 0.3,
                'progress_checker_plugin': 'progress_checker',
                'goal_checker_plugins': ['general_goal_checker'],
                'controller_plugins': ['FollowPath'],
                # Progress checker
                'progress_checker': {
                    'plugin': 'nav2_controller::SimpleProgressChecker',
                    'required_movement_radius': 0.5,
                    'movement_time_allowance': 10.0,
                },
                # Goal checker
                'general_goal_checker': {
                    'stateful': True,
                    'plugin': 'nav2_controller::SimpleGoalChecker',
                    'xy_goal_tolerance': 0.25,
                    'yaw_goal_tolerance': 0.25,
                },
                # DWB Local Planner
                'FollowPath': {
                    'plugin': 'dwb_core::DWBLocalPlanner',
                    'debug_trajectory_details': True,
                    'min_vel_x': 0.0,
                    'min_vel_y': 0.0,
                    'max_vel_x': 0.26,
                    'max_vel_y': 0.0,
                    'max_vel_theta': 1.0,
                    'min_speed_xy': 0.0,
                    'max_speed_xy': 0.26,
                    'min_speed_theta': 0.0,
                    'acc_lim_x': 2.5,
                    'acc_lim_y': 0.0,
                    'acc_lim_theta': 3.2,
                    'decel_lim_x': -2.5,
                    'decel_lim_y': 0.0,
                    'decel_lim_theta': -3.2,
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
                    'critics': ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'],
                    'BaseObstacle.scale': 0.02,
                    'PathAlign.scale': 32.0,
                    'PathAlign.forward_point_distance': 0.1,
                    'GoalAlign.scale': 24.0,
                    'GoalAlign.forward_point_distance': 0.1,
                    'PathDist.scale': 32.0,
                    'GoalDist.scale': 24.0,
                    'RotateToGoal.scale': 32.0,
                    'RotateToGoal.slowing_factor': 5.0,
                    'RotateToGoal.lookahead_time': -1.0,
                },
            }
        ],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ]
    )
    
    # =====================================================================
    # 3. NAV2 ADAPTER NODE
    # Hvata /planned_path od A* planera i šalje je Nav2 FollowPath akciji
    # =====================================================================
    nav2_adapter_node = Node(
        package='student_assignment_02',
        executable='nav2_adapter',
        name='nav2_adapter',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    
    # =====================================================================
    # LAUNCH DESCRIPTION
    # =====================================================================
    ld = LaunchDescription([
        # Argumenti
        use_sim_time_arg,
        inflation_distance_arg,
        
        # Redoslijed je BITAN!
        # 1. Prvo A* planer
        astar_node,
        
        # 2. Nav2 Controller (OVO STVARA /follow_path akciju)
        controller_server,
        
        # 3. Adapter koji povezuje A* s Nav2
        nav2_adapter_node,
    ])
    
    return ld
