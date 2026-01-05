import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import subprocess

def get_project_root():
    """Pronađi korijen projekta"""
    try:
        # Pokušaj pronađi GIT korijen
        result = subprocess.run(
            ['git', 'rev-parse', '--show-toplevel'],
            cwd=os.path.dirname(__file__),
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except:
        pass
    
    # Fallback: pronađi data/maps direktorij od package_share_directory
    package_dir = get_package_share_directory('student_assignment_02')
    # student_assignment_02/install/student_assignment_02/share/student_assignment_02
    # Trebam doći do student_assignment_02/ (glavna direktiva)
    # ../../../ = ide iz share/student_assignment_02 u student_assignment_02
    return os.path.abspath(os.path.join(package_dir, '../../..'))

def generate_launch_description():
    # Package direktorij
    package_dir = get_package_share_directory('student_assignment_02')
    
    # Pronađi korijen projekta
    project_root = get_project_root()
    
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
    
    # Putanja do mape koristi projekt root
    # Dinamička interpolacija za map_name
    map_dir = os.path.join(project_root, 'data', 'maps')
    
    # Kreiraj mapu po map_name
    def get_map_yaml():
        map_name = LaunchConfiguration('map_name').perform(None)
        return os.path.join(map_dir, map_name, 'map.yaml')
    
    # Koristi direktan path
    map_yaml_path = os.path.join(
        project_root,
        'data',
        'maps',
        '${map_name}',  # Bit će substituiran pri izvršavanju
        'map.yaml'
    ).replace('${map_name}', LaunchConfiguration('map_name').perform(None) or 'my_map')
    
    # Jednostavnije: koristi evaluator
    class MapYamlEvaluator:
        def __init__(self, root):
            self.root = root
        
        def __str__(self):
            return str(self.root)
    
    # Build putanja na osnovu map_name
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
