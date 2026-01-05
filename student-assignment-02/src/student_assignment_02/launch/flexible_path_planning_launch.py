import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package direktorij
    package_dir = get_package_share_directory('student_assignment_02')
    
    # Deklaracija argumenata
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='my_map',
        description='Name of the map in data/maps/ (e.g., "my_map", "office", "lab")'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Stage'
    )

    # Putanja do Stage world fajla
    stage_world = os.path.join(
        package_dir,
        'world',
        'stage.world'
    )
    
    # Pronađi projekt root - ide "gore" iz package_dir
    # package_dir = .../install/student_assignment_02/share/student_assignment_02
    # Trebam doci do .../student-assignment-02/
    project_root = os.path.abspath(os.path.join(package_dir, '../../..'))
    
    # Putanja do mape - biti će formirana s map_name parametrom
    # Trebam default putanju jer se map_name ne može koristiti direktno u os.path.join
    map_yaml = os.path.join(
        project_root,
        'data',
        'maps',
        'my_map',  # Default
        'map.yaml'
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,

        # ========== SIMULATOR ==========
        # Stage simulator (koristi stage_ros2)
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage_ros2',
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

        # ========== MAP MANAGEMENT ==========
        # Map Server - učitava odabranu mapu iz data/maps/
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
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

        # ========== PATH PLANNING & NAVIGATION ==========
        # Path Planning Node (A*)
        Node(
            package='student_assignment_02',
            executable='path_planning_node',
            name='path_planning_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
        ),

        # Goal Navigation Node
        Node(
            package='student_assignment_02',
            executable='goal_navigation_node',
            name='goal_navigation_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'linear_speed': 0.2,
                'angular_speed': 0.5,
                'waypoint_tolerance': 0.15,
                'angle_tolerance': 0.2,
            }],
        ),
    ])
