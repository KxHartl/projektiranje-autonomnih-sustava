#!/usr/bin/env python3
"""
Navigation Launch File - MINIMALIST
SAMO:
- A* Path Planner
- Nav2 Controller Server (DWB) - KRITIČNO!
- Nav2 Velocity Smoother
- Nav2 Adapter

BEZ: BT Navigator, Planner Server, Lifecycle Manager (komplicirani)

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
    """Launch file - SAMO A* + Nav2 Controller"""
    
    # Paket direktorij
    student_pkg_dir = get_package_share_directory('student_assignment_02')
    
    # Konfiguracija datoteke
    nav2_params_file = os.path.join(student_pkg_dir, 'config', 'nav2_params.yaml')
    
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
    # 2. NAV2 CONTROLLER SERVER (DWB - lokalni planer)
    # =====================================================================
    # VAŽNO: Ovo je ono što stvara /follow_path akciju!
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ]
    )
    
    # =====================================================================
    # 3. NAV2 VELOCITY SMOOTHER (glatče zapovijedane brzine)
    # =====================================================================
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
        ]
    )
    
    # =====================================================================
    # 4. NAV2 ADAPTER NODE
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
        
        # 2. Nav2 komponente (samo potrebne)
        # VAŽNO: controller_server MORA biti pokrenut prije adapter-a!
        controller_server,
        velocity_smoother,
        
        # 3. Adapter koji povezuje A* s Nav2
        nav2_adapter_node,
    ])
    
    return ld
