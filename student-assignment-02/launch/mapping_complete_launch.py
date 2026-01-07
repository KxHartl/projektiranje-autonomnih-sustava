#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('student_assignment_02')
    
    # Config file paths
    mapper_params_file = PathJoinSubstitution([
        pkg_dir,
        'config',
        'mapper_params_online_async.yaml'
    ])
    
    rviz_config_file = PathJoinSubstitution([
        pkg_dir,
        'config',
        'rviz_config.rviz'
    ])
    
    ld = LaunchDescription()
    
    # 1. Static transform publisher (laser -> base_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '-0.15',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        output='screen',
        namespace=''
    )
    ld.add_action(static_tf)
    
    # 2. SLAM Toolbox - Online Async mode
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[mapper_params_file],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    ld.add_action(slam_node)
    
    # 3. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    ld.add_action(rviz_node)
    
    return ld
