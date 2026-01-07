#!/usr/bin/env python3
"""
setup.py za student_assignment_02 ROS 2 Python paket

Ovo je ispravljeno kako bi sigurno pronašlo student_assignment_02 direktorij
s __init__.py datotekom.
"""

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'student_assignment_02'

# Pronađi sve packages u current direktoriju
# where='.' znači počni od trenutnog direktorija
# include=['student_assignment_02'] znači pronađi samo ovaj package
packages = find_packages(where='.', include=['student_assignment_02*'])

print(f"\n=== SETUP.PY DEBUG ===")
print(f"Current working directory: {os.getcwd()}")
print(f"Script location: {os.path.dirname(os.path.abspath(__file__))}")
print(f"Packages found: {packages}")
print(f"Contents of current dir: {os.listdir('.')}")

if os.path.isdir(package_name):
    print(f"Contents of {package_name}: {os.listdir(package_name)}")
    if '__init__.py' in os.listdir(package_name):
        print(f"✓ {package_name}/__init__.py EXISTS")
    else:
        print(f"✗ {package_name}/__init__.py MISSING!")
else:
    print(f"✗ {package_name} direktorij ne postoji!")

if not packages:
    raise RuntimeError(
        f"\n[ERROR] find_packages() nije pronašao niti jedan package!\n"
        f"Provjera:\n"
        f"1. Je li {package_name}/__init__.py kreiran?\n"
        f"2. Je li struktura ispravna?\n"
        f"   src/student_assignment_02/\n"
        f"   ├── student_assignment_02/ (package direktorij)\n"
        f"   │   ├── __init__.py\n"
        f"   │   ├── a_star_path_planner.py\n"
        f"   │   └── ..."
    )

print(f"=== SETUP.PY DEBUG GOTOV ===\n")

# Pronađi sve data files
data_files = []

# Resource file
data_files.append(('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]))

# package.xml
data_files.append(('share/' + package_name, ['package.xml']))

# Launch files
if os.path.isdir('launch'):
    launch_files = glob('launch/*.py')
    if launch_files:
        data_files.append(('share/' + package_name + '/launch', launch_files))

# Config files
if os.path.isdir('config'):
    config_files = glob('config/*.yaml') + glob('config/*.rviz')
    if config_files:
        data_files.append(('share/' + package_name + '/config', config_files))

# World files
if os.path.isdir('world'):
    world_files = glob('world/*.world') + glob('world/*.png')
    if world_files:
        data_files.append(('share/' + package_name + '/world', world_files))
    # Include files
    include_files = glob('world/include/*.inc')
    if include_files:
        data_files.append(('share/' + package_name + '/world/include', include_files))

# Mapped maps
if os.path.isdir('mapped_maps'):
    for root, dirs, files in os.walk('mapped_maps'):
        if files:
            rel_path = os.path.relpath(root, '.')
            dest = os.path.join('share', package_name, rel_path)
            src_files = [os.path.join(rel_path, f) for f in files]
            data_files.append((dest, src_files))

print(f"Data files: {len(data_files)} directories\n")

setup(
    name=package_name,
    version='0.0.0',
    packages=packages,
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khartl',
    maintainer_email='kh239762@fsb.hr',
    description='ROS2 Python package for autonomous system mapping and path planning using A* algorithm',
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
