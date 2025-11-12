from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    trajectory_publisher = Node(
        package='fanuc_m10ia_support',
        executable='test_trajectory.py',
        name='trajectory_publisher',
        output='screen'
    )
    return LaunchDescription([
        trajectory_publisher
    ])
