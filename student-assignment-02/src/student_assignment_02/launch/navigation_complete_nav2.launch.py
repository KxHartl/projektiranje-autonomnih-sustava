#!/usr/bin/env python3
"""
Navigation Complete with Nav2 Launch File
Integrira A* global path planning s Nav2 lokalnim planerom
Nav2 automatski sljedi putanju generirane A* planaterom
Za korištenje: ros2 launch student_assignment_02 navigation_complete_nav2.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file za A* planiranje putanje s Nav2 localnim planerom"""
    
    # Paket direktorij
    student_pkg_dir = get_package_share_directory('student_assignment_02')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Konfiguracija datoteke
    nav2_params_file = os.path.join(student_pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(student_pkg_dir, 'config', 'rviz_navigation.rviz')
    
    # =====================================================================
    # LAUNCH ARGUMENTI
    # =====================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Koristi simulacijsko vrijeme'
    )
    
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='5.0',
        description='Goal X koordinata (m)'
    )
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='5.0',
        description='Goal Y koordinata (m)'
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
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'start_x': 0.0,
                'start_y': 0.0,
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
    # 2. NAV2 BRINGUP - Kompletna Nav2 stack
    # =====================================================================
    # Nav2 sadrži:
    # - planner_server (globalni planer)
    # - controller_server (lokalni planer - DWB, Regulated Pure Pursuit itd.)
    # - lifecycle_manager
    # - behavior_tree_navigator
    # - velocity_smoother
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam': 'False',  # Trebamo mapu, nije SLAM
            'params_file': nav2_params_file,
        }.items(),
    )
    
    # =====================================================================
    # 3. NAV2 TO A* ADAPTER NODE
    # =====================================================================
    # Ovaj čvor hvata putanju od A* planera i slanje je Nav2
    # kao global plan (umjesto korištenja Nav2 planner_server-a)
    nav2_adapter_node = Node(
        package='student_assignment_02',
        executable='nav2_adapter',
        name='nav2_adapter',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('/astar_path', '/planned_path'),
        ]
    )
    
    # =====================================================================
    # 4. RViz2 - VIZUALIZACIJA
    # =====================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # =====================================================================
    # LAUNCH DESCRIPTION
    # =====================================================================
    ld = LaunchDescription([
        # Argumenti
        use_sim_time_arg,
        goal_x_arg,
        goal_y_arg,
        inflation_distance_arg,
        
        # Čvorovi
        astar_node,
        nav2_bringup,
        nav2_adapter_node,  # Adapter koji povezuje A* s Nav2
        rviz_node,
    ])
    
    return ld
