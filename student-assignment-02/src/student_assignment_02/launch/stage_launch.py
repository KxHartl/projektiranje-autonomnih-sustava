#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Tvoj paket
    student_share = get_package_share_directory('student_assignment_02')
    student_config_dir = os.path.join(student_share, 'config')

    # Argumenti
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='map_01',
        description='World file name (bez .world)'
    )

    declare_use_stage = DeclareLaunchArgument(
        'stage',
        default_value='true',
        description='Start Stage simulator'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    use_stage = LaunchConfiguration('stage')
    use_rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------------------------
    # STAGE
    # ------------------------------------------------------------------
    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage',
        condition=IfCondition(use_stage),
        parameters=[{
            'world_file': os.path.join(
                student_share, 
                'world', 
                'map_01.world'
            ),
            'enforce_prefixes': False,  # VAŽNO: False da bi topici bili dostupni
            'use_stamped_velocity': False,
            'use_static_transformations': True,
        }],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/odom', '/odom'),
            ('/base_pose_ground_truth', '/ground_truth'),
        ]
    )

    # ------------------------------------------------------------------
    # ROBOT STATE PUBLISHER - Za TF između base_link i base_scan
    # ------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': """
<robot name="robot">
  <link name="base_link"/>
  <link name="base_scan"/>
  <joint name="base_scan_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_scan"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
            """
        }],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    # ------------------------------------------------------------------
    # STATIC TF PUBLISHER - Za odom -> base_link transformaciju
    # ------------------------------------------------------------------
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ------------------------------------------------------------------
    # RVIZ
    # ------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(student_config_dir, 'rviz_config.rviz')],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_world,
        declare_use_stage,
        declare_use_rviz,
        declare_use_sim_time,
        stage_node,
        robot_state_publisher,
        static_tf_publisher,
        rviz_node,
    ])
