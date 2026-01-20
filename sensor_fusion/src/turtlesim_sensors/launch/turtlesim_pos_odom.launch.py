from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            prefix='gnome-terminal --',
            output='screen',
        ),
        Node(
            package='turtlesim_sensors_template',
            executable='positioning_system',
        ),
        Node(
            package='turtlesim_sensors_template',
            executable='odometry',
        )
    ])