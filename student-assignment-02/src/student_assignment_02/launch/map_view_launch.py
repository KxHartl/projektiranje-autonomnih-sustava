#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Map View Launch File - V8 (KONTINUIRANO OBJAVLJUJE /map)
Prikazi Stage simulator s mapom iz mapped_maps/ direktorija

V8: 
- Map Server KONTINUIRANO objavljuje /map (svaku sekundu)
- RViz koristi Transient Local QoS
- Sve je optimizirano za map display

Usage:
    ros2 launch student_assignment_02 map_view_launch.py
"""

import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Dobij direktorij naseg paketa (iz install/)
    student_share = get_package_share_directory('student_assignment_02')
    student_config_dir = os.path.join(student_share, 'config')
    student_world_dir = os.path.join(student_share, 'world')
    
    # Direktna putanja do student-assignment-02/
    student_assignment_root = os.path.expanduser(
        '~/FSB/projektiranje-autonomnih-sustava/student-assignment-02'
    )
    mapped_maps_dir = os.path.join(student_assignment_root, 'mapped_maps')
    map_file = os.path.join(mapped_maps_dir, 'map_01', 'map_01.yaml')

    # RViz konfiguracija
    rviz_config_file = os.path.join(student_config_dir, 'rviz_config.rviz')

    # Argumenti
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')

    # Direktna path do world datoteke
    world_file = os.path.join(student_world_dir, 'map_01.world')

    print(f"\n" + "="*70)
    print(f"map_view_launch.py V8 - KONTINUIRANO OBJAVLJUJE /map")
    print(f"="*70)
    print(f"Student assignment root: {student_assignment_root}")
    print(f"Mapped maps dir:         {mapped_maps_dir}")
    print(f"Map file:                {map_file}")
    print(f"RViz config:             {rviz_config_file}")
    print(f"="*70)
    print()

    # Provjeri da li mapa postoji
    if not os.path.exists(map_file):
        print(f"⚠️  WARNING: Map file ne postoji!")
        print(f"   Tražim u: {map_file}")
        print()
    else:
        print(f"✅ Map file pronađen: {os.path.basename(map_file)}")
        print()

    # Provjeri RViz konfiguraciju
    if not os.path.exists(rviz_config_file):
        print(f"⚠️  WARNING: RViz config ne postoji!")
        print(f"   Tražim u: {rviz_config_file}")
        print()
    else:
        print(f"✅ RViz config pronađen")
        print()

    # ====================================================================
    # 1. STAGE SIMULATOR
    # ====================================================================
    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'world_file': world_file,
            'enforce_prefixes': False,
            'use_stamped_velocity': False,
            'use_static_transformations': False,
            'publish_ground_truth': True,
        }],
        remappings=[
            ('/odom', '/odom'),
            ('/base_pose_ground_truth', '/ground_truth'),
        ]
    )

    # ====================================================================
    # 2. STATIC TF: map -> odom (ZA FIKSATI FRAME PROBLEM)
    # ====================================================================
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================================================================
    # 3. STATIC TF: laser -> base_link
    # ====================================================================
    laser_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '-0.15', '0', '0', '0', '1', 'laser', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================================================================
    # 4. ROBOT STATE PUBLISHER
    # ====================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': """
<robot name="robot">
  <link name="base_link"/>
  <link name="laser"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
            """
        }],
        remappings=[]
    )

    # ====================================================================
    # 5. MAP SERVER - Kontinuirano objavljuje /map
    # ====================================================================
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time},
            {'publish_interval': 1.0},  # NOVO! Objavljuj /map svaku sekundu
        ],
        remappings=[
            ('/map', '/map'),
            ('/map_metadata', '/map_metadata'),
        ]
    )

    # ====================================================================
    # 6. LIFECYCLE MANAGER - Aktivira Map Server automatski
    # ====================================================================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    # ====================================================================
    # 7. RVIZ - Vizualizacija s konfiguracijom
    # ====================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=[
            '-d', rviz_config_file
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================================================================
    # Kombiniraj sve
    # ====================================================================
    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        stage_node,
        map_to_odom_tf,
        laser_to_base_link_tf,
        robot_state_publisher,
        map_server_node,
        lifecycle_manager,
        rviz_node,
    ])
