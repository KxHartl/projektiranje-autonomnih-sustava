import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Deklaracija argumenata
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.expanduser('~/my_map'),
        description='Path to the pre-mapped map directory'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Putanja do Stage launch fajla
    stage_launch_dir = os.path.join(
        get_package_share_directory('student_assignment_02'),
        'launch'
    )

    # Parametri za Stage simulator
    stage_world = os.path.join(
        get_package_share_directory('student_assignment_02'),
        'world',
        'stage.world'
    )

    return LaunchDescription([
        map_path_arg,
        use_sim_time_arg,

        # Stage simulator
        Node(
            package='stage_ros',
            executable='stageros',
            name='stageros',
            arguments=[stage_world],
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
        ),

        # Map Server - učitava unaprijed mapiranu mapu
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': os.path.expanduser('~/my_map/map.yaml'),
                'use_sim_time': True,
            }],
        ),

        # Lifecycle Manager za Map Server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
    ])
