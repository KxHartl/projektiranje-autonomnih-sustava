# 🤖 STUDENT ASSIGNMENT 02 - A* Path Planning

**Mapping (SLAM) → Localization (AMCL) → Path Planning (A*)**

Complete implementation of autonomous robotics. This README guides you step by step from start to finish.

> ⚠️ **IMPORTANT**: Carefully follow all steps in order. If something doesn't work, see [Troubleshooting](#troubleshooting).

---

## 📋 Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation--first-time-)
3. [Step 1: Mapping](#step-1-mapping)
4. [Step 2: Localization](#step-2-localization)
5. [Step 3: A* Path Planning](#step-3-a-path-planning)
6. [Parameters](#parameters)
7. [Troubleshooting](#troubleshooting)

---

## 📦 Prerequisites

Verify that you have:

```bash
ros2 --version
```

You should see: `ROS 2 Humble ...`

If you don't have ROS 2 Humble installed, [follow the official installation guide](https://docs.ros.org/en/humble/Installation.html).

---

## 🔧 Installation (FIRST TIME)

### Step 1: Clone Repository

```bash
# Create a directory for the project
mkdir -p ~/assignment_02_ws
cd ~/assignment_02_ws

# Clone repository
git clone https://github.com/KxHartl/projektiranje-autonomnih-sustava.git

# Navigate to the assignment folder
cd projektiranje-autonomnih-sustava/student-assignment-02
```

**Expected structure**:
```
~/assignment_02_ws/
└── projektiranje-autonomnih-sustava/
    └── student-assignment-02/
        ├── src/
        ├── launch/
        ├── README.md
        └── ...
```

### Step 2: Build Project

```bash
# Make sure you're in the student-assignment-02 directory
pwd
# Should output: .../student-assignment-02

# Clean old build files
rm -rf build/ install/ log/

# Build the project
colcon build --symlink-install
```

### Step 3: Source Setup

```bash
# Load the environment
source install/setup.bash

**✅ INSTALLATION COMPLETE!**
```


### 🗺️ STEP 1: Mapping

**Goal**: Map the environment using SLAM Toolbox

### 1.1 Terminal 1: SLAM Mapping

Open a new terminal:

```bash
cd ~/assignment_02_ws/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 launch student_assignment_02 mapping_complete_launch.py
```

**You should see**: RViz window with the map being built.

### 1.2 Terminal 2: Robot Control

Open a new terminal:

```bash
cd ~/assignment_02_ws/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**You should see**: Message "Publishing twist"

**Goal**: Drive the robot throughout the entire environment so SLAM can map everything.

**Tip**: Drive in a U-shape pattern, covering all walls and corners.

### 1.3 Terminal 3: Save Map

Open a new terminal:

```bash
cd ~/assignment_02_ws/projektiranje-autonomnih-sustava/student-assignment-02

# Check the number of the last map
ls src/student_assignment_02/mapped_maps/
```

You will see something like:
```
map_01  map_02  map_03  map_04
```

If the last one is `map_04`, save as `map_05`.

Open a new terminal:

```bash
cd ~/assignment_02_ws/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/mapped_maps/

mkdir map_05
```

**Now save the map** (replace USERNAME with your username - check with `whoami`):

```bash
# First check your username
whoami
```

If the output is `hartl`, then the command is:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name:
    data: '/home/hartl/assignment_02_ws/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/mapped_maps/map_05/map_05'"
```

> ⚠️ **IMPORTANT**: Replace `hartl` with the result of the `whoami` command!

**You should see**: Message "Map saved successfully"

### 1.4 Verify Map Save

In the same terminal:

```bash
ls -la src/student_assignment_02/mapped_maps/map_05/
```

You should see:
```
map_05.pgm   (map image)
map_05.yaml  (metadata)
```

**If you see these files: ✅ MAPPING COMPLETE!**

### 1.5 Stop Simulators

In all terminals, press **CTRL+C** to stop everything.

---

## 📍 STEP 2: Localization

**Goal**: Localize the robot with AMCL using the saved map

### 2.1 Terminal 1: Localization

Open a new terminal:

```bash
cd ~/assignment_02_ws/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

# Replace map_02 with your map number if different!
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_02
```

**You should see**: RViz with the map 

### 2.2 Terminal 2: Robot Control

Open a new terminal:

```bash
cd ~/assignment_02_ws/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**IMPORTANT**: Move the robot with the **I** key several times!


**✅ LOCALIZATION COMPLETE!**


## 🎯 STEP 3: A* Path Planning

**Goal**: Plan a path using the A* algorithm

### 3.1 Terminal 1: A* Path Planner

Open a new terminal:

```bash
cd ~/assignment_02_ws/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 launch student_assignment_02 a_star_path_planner.launch.py
```

**You should see**:
```
[INFO] A* Path Planner Node: Started
[INFO] Inflation distance: 0.5m
[INFO] Using base_link for starting point
```
### 3.6 Setting the Goal

**In RViz**:

1. In the top toolbar, click on **2D Goal Pose** (green arrow with red target point)
2. Click on the map where you want the goal
3. Drag the mouse slightly to set the direction
4. Release the mouse

**Expected result**: 
- 🟢 **Green line** appears (this is your path!)
- 🟠 **Orange cubes** show the buffer (0.5 m from walls)
- 🔴 **Gray spheres** show explored nodes
- 🟡 **Yellow spheres** show the algorithm's frontier

**You should see a message in terminal 3**:
```
[INFO] Path found! Length: XX nodes
[INFO] Path planning took X.XX seconds
```

**✅ A* PATH PLANNING COMPLETE!**

### 3.7 Try Multiple Goals

You can click on different locations on the map and the path will be replanned each time!

---

## ⚙️ Parameters

### Changing the Map

If you need to use a different map (e.g., map_04):

```bash
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_04
```

### A* Path Planner - Custom Parameters

```bash
# Example: Smaller buffer (0.15 m instead of 0.5 m)
ros2 launch student_assignment_02 a_star_path_planner.launch.py inflation_distance_m:=0.15

# Example: More iterations
ros2 launch student_assignment_02 a_star_path_planner.launch.py max_iterations:=100000

# Example: Without diagonal movement
ros2 launch student_assignment_02 a_star_path_planner.launch.py allow_diagonal:=false
```

---

## 🔍 Troubleshooting

### Problem: "Command not found" for `ros2 launch`

**Solution**: You need to source the setup:

```bash
cd ~/assignment_02_ws/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash
```

### Problem: "CMake Error" during build

**Solution**: Clean and try again:

```bash
cd ~/assignment_02_ws/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build/ install/ log/
colcon build --packages-select student_assignment_02 --symlink-install
```

### Problem: Map is not saving

**Expected**: See message "Map saved successfully"

**If you don't see it**: Check if SLAM Toolbox is running:

```bash
ros2 topic list | grep slam
```

You should see `/slam_toolbox/...` topics.

### Problem: Path not found

**Expected**: You should see a green line in RViz.

**If you don't see it**:
1. Check that the start position is valid (robot in free space)
2. Check that the goal is valid (click in free space)
3. Increase `max_iterations`:
   ```bash
   ros2 launch student_assignment_02 a_star_path_planner.launch.py max_iterations:=100000
   ```

### Problem: RViz doesn't show the map

**Expected**: See the map as a grid.

**If you don't see it**:
1. Check that Fixed Frame is set to `map`
2. Add `/map` as OccupancyGrid
3. Check if `/map` topic is available:
   ```bash
   ros2 topic echo /map --once | head -20
   ```

### Problem: Transform `base_link` doesn't exist

**Expected**: `ros2 run tf2_ros tf2_echo map base_link` shows coordinates.

**If you get an error**:
1. Check if localization is running
2. Move the robot more
3. Check the TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Problem: Need old code/map

**Old code**: GitHub history:
```bash
cd ~/assignment_02_ws/projektiranje-autonomnih-sustava
git log --oneline
```

**Old map**: All maps are in the `src/student_assignment_02/mapped_maps/` directory.

---

## 📊 Monitoring & Debug

### Check All Active Nodes

```bash
ros2 node list
```

You should see:
```
/a_star_path_planner
/amcl
/stage_ros2
/rviz2
...
```

### Check All Topics

```bash
ros2 topic list
```

### Print Path (Live)

```bash
ros2 topic echo /planned_path
```

### Print Map (Live)

```bash
ros2 topic echo /map --once
```

---

## 📁 Directory Structure

```
~/assignment_02_ws/
└── projektiranje-autonomnih-sustava/
    └── student-assignment-02/
        ├── src/student_assignment_02/
        │   ├── student_assignment_02/
        │   │   ├── a_star_path_planner.py     ← Main code
        │   │   ├── map_republisher.py
        │   │   └── ...
        │   ├── launch/
        │   │   ├── mapping_complete_launch.py       ← Mapping
        │   │   ├── localization_complete_launch.py  ← Localization
        │   │   └── a_star_path_planner.launch.py    ← A* Planner
        │   ├── mapped_maps/
        │   │   ├── map_01/
        │   │   ├── map_05/        ← NEW MAP
        │   │   └── ...
        │   ├── config/
        │   └── setup.py
        ├── README.md              ← THIS FILE
        └── ...
```
---

## 📚 Additional Information

### ROS 2 Documentation
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

### Algorithms
- [A* Search](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
- [AMCL Localization](https://wiki.ros.org/amcl)

### Simulators
- [Stage ROS2](https://github.com/ros-simulation/stage_ros2)

---

## 👨‍💻 Author

**Kresimir Hartl** (KxHartl)  
Faculty of Mechanical Engineering and Naval Architecture, Zagreb  
January 2026

---

## 📞 Support

If you need help:

1. Check the [Troubleshooting](#troubleshooting) section
2. Check the [ROS 2 documentation](https://docs.ros.org/en/humble/)
3. Open a GitHub issue: [Issues](https://github.com/KxHartl/projektiranje-autonomnih-sustava/issues)

---

**Status**: ✅ COMPLETE  
**Version**: 1.1.0  
**Date**: January 7, 2026  
**Last Modified**: January 7, 2026
