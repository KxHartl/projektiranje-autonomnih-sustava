from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'student_assignment_02'

# Base directory of this package (this file's directory)
package_dir = os.path.dirname(os.path.realpath(__file__))

# Find all packages (trebam biti specifičan)
packages = find_packages(where='', include=['student_assignment_02', 'student_assignment_02.*'])

print(f"[INFO] Found packages: {packages}")

# Base data files (sources must be relative paths)
data_files = [
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ('share/' + package_name, ['package.xml']),
]

# add launch, config, world files if present (use relative source paths)
launch_dir = os.path.join(package_dir, 'launch')
if os.path.isdir(launch_dir):
    launch_files = [os.path.join('launch', f) for f in os.listdir(launch_dir) if f.endswith('.py')]
    if launch_files:
        data_files.append(('share/' + package_name + '/launch', launch_files))

config_dir = os.path.join(package_dir, 'config')
if os.path.isdir(config_dir):
    rviz_files = [os.path.join('config', f) for f in os.listdir(config_dir) if f.endswith('.rviz')]
    yaml_files = [os.path.join('config', f) for f in os.listdir(config_dir) if f.endswith('.yaml')]
    if rviz_files:
        data_files.append(('share/' + package_name + '/config', rviz_files))
    if yaml_files:
        data_files.append(('share/' + package_name + '/config', yaml_files))

world_dir = os.path.join(package_dir, 'world')
if os.path.isdir(world_dir):
    world_files = [os.path.join('world', f) for f in os.listdir(world_dir) if f.endswith('.world')]
    png_files = [os.path.join('world', f) for f in os.listdir(world_dir) if f.endswith('.png')]
    if world_files:
        data_files.append(('share/' + package_name + '/world', world_files))
    if png_files:
        data_files.append(('share/' + package_name + '/world', png_files))
    include_dir = os.path.join(world_dir, 'include')
    if os.path.isdir(include_dir):
        include_files = [os.path.join('world', 'include', f) for f in os.listdir(include_dir) if f.endswith('.inc')]
        if include_files:
            data_files.append(('share/' + package_name + '/world/include', include_files))

# If mapped_maps exists inside the package, include all files and preserve
# subdirectory structure by adding a tuple per directory. Source paths
# must be relative to the package directory (no leading slash).
mapped_maps_root = os.path.join(package_dir, 'mapped_maps')
if os.path.isdir(mapped_maps_root):
    for root, dirs, files in os.walk(mapped_maps_root):
        if not files:
            continue
        rel = os.path.relpath(root, package_dir)
        # rel will be like 'mapped_maps' or 'mapped_maps/map_01'
        dest_rel = rel.replace('mapped_maps', os.path.join('mapped_maps'))
        src_paths = [os.path.join(rel, f) for f in files]
        dest = os.path.join('share', package_name, rel)
        data_files.append((dest, src_paths))

print(f"[INFO] Data files: {data_files}")

setup(
    name=package_name,
    version='0.0.0',
    packages=packages,
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khartl',
    maintainer_email='kh239762@fsb.hr',
    description='ROS2 Python package for autonomous system mapping and path planning',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a_star_path_planner = student_assignment_02.a_star_path_planner:main',
            'map_republisher = student_assignment_02.map_republisher:main',
            'path_planning_node = student_assignment_02.path_planning_node:main',
            'goal_navigation_node = student_assignment_02.goal_navigation_node:main',
        ],
    },
)
