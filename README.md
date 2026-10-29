# Design of autonomous systems: course assignments

**Projektiranje autonomnih sustava**

> Design of Autonomous Systems, MSc in Mechatronics and Robotics, 3rd semester, FSB, University of Zagreb, 2025/2026

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E)
![ros2_control](https://img.shields.io/badge/control-ros2__control-22314E)
![Python](https://img.shields.io/badge/Python-3776AB)
![Stage](https://img.shields.io/badge/simulator-Stage-4b5563)

The assignments and lab work of the course, built on ROS 2 Humble. They cover a ros2_control
set-up for an industrial arm, sensor fusion with an extended Kalman filter, and mapping,
localisation and path planning for a mobile robot.

**The course's main project is a separate repository:
[pas-dual-arm](https://github.com/KxHartl/pas-dual-arm)**, a simulated mobile manipulator with
two arms that drives to a box, lifts it with both arms and carries it to a marked spot in another
room (ROS 2 Humble, Gazebo Fortress, Nav2, MoveIt).

## What's inside

| Folder | Contents |
|---|---|
| `student-assignment-01/` | Assignment 1: a mock ros2_control system for the six-axis Fanuc M10iA, with RViz visualisation, a forward position controller and a joint trajectory controller, and launch files that publish positions and trajectories to each |
| `student-assignment-02/` | Assignment 2: a mobile robot in the Stage simulator, mapped with SLAM, localised with AMCL, and driven along a path from a custom A* planner (a D* planner and a Nav2 variant are also in the package) |
| `sensor_fusion/` | lab work: noisy odometry and position sensors for turtlesim, fused with `robot_localization` EKF nodes in an odom and a map frame |
| `exercises-03/` | lab work: a ROS 2 workspace based on [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) by the ros-controls project (Apache-2.0, licence kept in the folder) |
| `seminar/` | the starting workspace for the seminar project, with the task notes, backlog and roadmap. The project itself continued in [pas-dual-arm](https://github.com/KxHartl/pas-dual-arm). The lecture slides and the assignment PDF belong to the teachers and are not published here |

Each assignment folder has its own README with the full steps (in English).

## Build and run

Each assignment is a colcon workspace. For example, assignment 1:

```bash
cd student-assignment-01
colcon build
source install/setup.bash
ros2 launch fanuc_m10ia_support fanuc_controllers.launch.py
```

Assignment 2 runs in three separate stages. Its README explains driving the robot, saving the
map and setting the goal in between.

```bash
cd student-assignment-02
colcon build --symlink-install
source install/setup.bash

# 1. mapping
ros2 launch student_assignment_02 mapping_complete_launch.py
# 2. localisation on a saved map
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_02
# 3. A* path planning, next to the localisation
ros2 launch student_assignment_02 a_star_path_planner.launch.py
```
