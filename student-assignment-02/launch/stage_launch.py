#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('student_assignment_02')
    
    # Use PathJoinSubstitution za ispravnu kombinaciju putanja s LaunchConfiguration
    config_file = PathJoinSubstitution([
        LaunchConfiguration('share_dir', default=pkg_dir),
        'config',
        'mapper_params_online_async.yaml'
    ])
    
    # Launch description
    ld = LaunchDescription()
    
    # 1. Stage simulator
    stage_node = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
             os.path.join(pkg_dir, 'worlds', 'stage_sim.world')],
        output='screen'
    )
    ld.add_action(stage_node)
    
    # 2. RViz
    rviz_config = PathJoinSubstitution([
        pkg_dir,
        'config',
        'rviz_config.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    ld.add_action(rviz_node)
    
    # 3. Static transform publisher (laser -> base_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '-0.15',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        output='screen'
    )
    ld.add_action(static_tf)
    
    # 4. SLAM Toolbox (Online Async)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[config_file]
    )
    ld.add_action(slam_node)
    
    return ld
