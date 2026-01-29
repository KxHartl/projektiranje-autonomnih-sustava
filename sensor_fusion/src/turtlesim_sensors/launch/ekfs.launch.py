from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os

def generate_launch_description():

    turtlesim_params_1 = os.path.join(
        get_package_share_directory("turtlesim_sensors"),
        "config",
        "ekf_odom_base.yaml"
    )
    
    turtlesim_params_2 = os.path.join(
        get_package_share_directory("turtlesim_sensors"),
        "config",
        "ekf_map_odom.yaml"
    )

    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_1',
            output='screen',
            parameters=[
                turtlesim_params_1
            ],
            remappings=[
                ('odometry/filtered', 'odometry/filtered_twist')
            ]
        ),
        
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_2',
            output='screen',
            parameters=[
                turtlesim_params_2
            ],
            remappings=[
                ('odometry/filtered', 'odometry/filtered_pose')
            ]
        ),
    ])
