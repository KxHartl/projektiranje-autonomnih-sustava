#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Map View Launch File - V4 (CRITICAL FIX)
Prikazi Stage simulator s mapom iz mapped_maps/ direktorija

FIX: Koristi ROS_WORKSPACE ili eksplicitnu putanju, ne install/

Usage:
    ros2 launch student_assignment_02 map_view_launch.py
"""

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_workspace_root():
    """
    Pronađi root direktorij workspace-a (student-assignment-02)
    Koristi: git rev-parse --show-toplevel ILI environment varijabla
    """
    try:
        # Pokušaj s git-om
        result = subprocess.run(
            ['git', 'rev-parse', '--show-toplevel'],
            capture_output=True,
            text=True,
            cwd=os.path.expanduser('~/FSB/projektiranje-autonomnih-sustava/student-assignment-02')
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except Exception as e:
        print(f"Debug: git command failed: {e}")
    
    # Fallback: Koristi home + poznatih putanja
    return os.path.expanduser('~/FSB/projektiranje-autonomnih-sustava/student-assignment-02')


def generate_launch_description():
    # Dobij direktorij naseg paketa (iz install/)
    student_share = get_package_share_directory('student_assignment_02')
    student_config_dir = os.path.join(student_share, 'config')
    student_world_dir = os.path.join(student_share, 'world')
    
    # CRITICAL FIX: Pronađi pravi root direktorij (student-assignment-02/)
    # NE install/student_assignment_02/, već student-assignment-02/
    workspace_root = get_workspace_root()
    mapped_maps_dir = os.path.join(workspace_root, 'mapped_maps')
    map_file = os.path.join(mapped_maps_dir, 'map_01', 'map_01.yaml')

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
    print(f"map_view_launch.py V4 - CRITICAL FIX")
    print(f"="*70)
    print(f"Workspace root:  {workspace_root}")
    print(f"Mapped maps dir: {mapped_maps_dir}")
    print(f"Map file:        {map_file}")
    print(f"World file:      {world_file}")
    print(f"="*70)
    print()

    # Provjeri da li mapa postoji
    if not os.path.exists(map_file):
        print(f"⚠️  WARNING: Map file ne postoji!")
        print(f"   Tražim u: {map_file}")
        print(f"   Kreiraj direktorij i spremi mapu:")
        print(f"   mkdir -p {os.path.dirname(map_file)}")
        print(f"   cp ~/temp_map.* {os.path.dirname(map_file)}/map_01.*")
        print()
    else:
        print(f"✅ Map file pronađen!")
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
    # 4. MAP SERVER - Ucitaj spremenljenu mapu iz mapped_maps/map_01/
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
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================================================================
    # Kombiniraj sve
    # ====================================================================
    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        stage_node,
        laser_to_base_link_tf,
        robot_state_publisher,
        map_server_node,
        rviz_node,
    ])
