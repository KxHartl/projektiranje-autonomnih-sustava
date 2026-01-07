#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Map View Launch File
Prikazi Stage simulator s prethodno spremenljenom mapom u RViz-u

Napomena: Ovo se koristi NAKON sto ste vec napravili mapu s mapiranjem
Mapa se koristi za vizualizaciju, ne za mapiranje

Usage:
    ros2 launch student_assignment_02 map_view_launch.py
    ros2 launch student_assignment_02 map_view_launch.py map_file:=$HOME/map_01.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Dobij direktorij naseg paketa
    student_share = get_package_share_directory('student_assignment_02')
    student_config_dir = os.path.join(student_share, 'config')
    student_world_dir = os.path.join(student_share, 'world')

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

    # Map file argument - Korisnik prosledjuje path do map_XX.yaml datoteke
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.expanduser('~/map_01.yaml'),
        description='Full path to map yaml file (e.g., ~/map_01.yaml)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    map_file = LaunchConfiguration('map_file')

    # Direktna path do world datoteke
    world_file = os.path.join(student_world_dir, 'map_01.world')

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
    # 2. STATIC TF: laser -> base_link
    # ====================================================================
    laser_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '-0.15', '0', '0', '0', '1', 'laser', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================================================================
    # 3. ROBOT STATE PUBLISHER
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
    # 4. MAP SERVER - Ucitaj spremenljenu mapu
    # ====================================================================
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/map', '/map'),
            ('/map_metadata', '/map_metadata'),
        ]
    )

    # ====================================================================
    # 5. RVIZ - Vizualizacija
    # ====================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=[
            '-d', os.path.join(student_config_dir, 'rviz_config.rviz')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================================================================
    # Kombiniraj sve
    # ====================================================================
    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_map_file,
        stage_node,
        laser_to_base_link_tf,
        robot_state_publisher,
        map_server_node,
        rviz_node,
    ])
