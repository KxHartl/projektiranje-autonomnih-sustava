# Assignment 1 – ROS2 Control Mock System for Fanuc M10iA Robot

This repository contains a complete solution for a student assignment implementing a mock ros2_control system for the industrial Fanuc M10iA robot with 6 degrees of freedom. The project includes robot visualization, ros2_control interface with two different controllers, and automated functionality tests.

## Repository Structure

```
student-assignment-01/
└── src/fanuc_m10ia_support/
    ├── config/                       # YAML controller configurations
    │   └── fanuc_controllers.yaml
    ├── launch/                       # Launch files for system startup
    │   ├── view_robot.launch.py
    │   ├── fanuc_controllers.launch.py
    │   ├── publish_forward_positions.launch.py
    │   └── publish_trajectory.launch.py
    ├── urdf/                         # Robot descriptions and ros2_control definitions
    │   ├── m10ia.xacro
    │   ├── m10ia_macro.xacro
    │   └── m10ia_ros2_control.xacro
    ├── meshes/                       # 3D visual and collision models
    ├── rviz/                         # RViz configurations
    └── test/                         # Automated tests
        ├── test_trajectory.py
        └── forward_position_publisher.py
```

## Installation and Setup

### 1. Clone Repository

```bash
git clone https://github.com/KxHartl/projektiranje-autonomnih-sustava.git
cd projektiranje-autonomnih-sustava/student-assignment-01
```

### 2. Install Dependencies

```bash
sudo apt update
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-controller-manager \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-xacro \
                 python3-pip
```

### 3. Build Workspace

```bash
colcon build
source install/setup.bash
```

## Execution and Testing

### Task 1: Robot Visualization with GUI Control

Launches robot visualization in RViz with the ability to manually set joint positions via GUI sliders, without the ros2_control system.

**Terminal 1:**
```bash
source install/setup.bash
ros2 launch fanuc_m10ia_support view_robot.launch.py
```

Result: Opens RViz with a 3D robot model and GUI window for joint control.

---

### Task 2: Launch with ros2_control Controllers

Launches the mock ros2_control system with two controllers:
- `forward_position_controller` (active)
- `joint_trajectory_controller` (loaded, inactive)

**Terminal 1:**
```bash
source install/setup.bash
ros2 launch fanuc_m10ia_support fanuc_controllers.launch.py
```

#### Manual Testing of forward_position_controller

**Terminal 2:**
```bash
source install/setup.bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]" --once
```

Result: Robot instantly "jumps" to the specified position.

#### Monitor Joint States

**Terminal 2:**
```bash
ros2 topic echo /joint_states
```

#### Check Controller Status

**Terminal 2:**
```bash
ros2 control list_controllers
```

Expected output:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
forward_position_controller[forward_command_controller/ForwardCommandController] active
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive
```

#### Switching Between Controllers

**Terminal 2:**
```bash
# Deactivate forward_position_controller and activate joint_trajectory_controller
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller

# Return to forward_position_controller
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller
```

#### Testing joint_trajectory_controller

**Terminal 2:**
```bash
source install/setup.bash
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller
cd src/fanuc_m10ia_support/test
python3 test_trajectory.py
```

Result: Robot smoothly executes a trajectory through defined points.

---

### Task 3: Automatic Position Publishing (forward_position_controller)

The launch file automatically publishes a series of test positions to the `forward_position_controller`.

**NOTE:** For this task, `forward_position_controller` must be active!

**Terminal 1:**
```bash
source install/setup.bash
ros2 launch fanuc_m10ia_support fanuc_controllers.launch.py
```

**Terminal 2 - Controller Check and Activation:**
```bash
source install/setup.bash
# Check controller status
ros2 control list_controllers

# If joint_trajectory_controller is active, switch to forward_position_controller
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller
```

**Terminal 3 - Launch Automatic Test:**
```bash
source install/setup.bash
ros2 launch fanuc_m10ia_support publish_forward_positions.launch.py
```

Result: Robot automatically "jumps" through the following positions (every 2 seconds):
- [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
- [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
- [1.0, 0.5, 0.0, 0.0, 0.0, 0.0]
- [0.5, 0.3, -0.2, 0.0, 0.5, 0.0]
- [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

---

### Task 4: Automatic Trajectory Publishing (joint_trajectory_controller)

The launch file automatically publishes a test trajectory to the `joint_trajectory_controller`.

**NOTE:** For this task, `joint_trajectory_controller` must be active!

**Terminal 1:**
```bash
source install/setup.bash
ros2 launch fanuc_m10ia_support fanuc_controllers.launch.py
```

**Terminal 2 - Controller Check and Activation:**
```bash
source install/setup.bash
# Check controller status
ros2 control list_controllers

# Deactivate forward_position_controller and activate joint_trajectory_controller
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller

# Verify the change was successful
ros2 control list_controllers
```

Expected output after change:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
forward_position_controller[forward_command_controller/ForwardCommandController] inactive
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
```

**Terminal 3 - Launch Automatic Test:**
```bash
source install/setup.bash
ros2 launch fanuc_m10ia_support publish_trajectory.launch.py
```

Result: Robot smoothly executes a complex trajectory defined in the script.

---

## Useful Commands for Checking and Debugging

```bash
# List all available topics
ros2 topic list

# Monitor current joint positions
ros2 topic echo /joint_states

# Check available controllers and their status
ros2 control list_controllers

# List all hardware interfaces
ros2 control list_hardware_interfaces

# Check transformations between frames
ros2 run tf2_ros tf2_echo base_link tool0
```

## Technical Details

### Mock Hardware System

The project uses the `mock_components/GenericSystem` plugin that simulates the robot's hardware interface without requiring real hardware. The configuration is located in `urdf/m10ia_ros2_control.xacro`.

### Controllers

- **forward_position_controller**: Receives position commands and sets them instantly. No interpolation.
- **joint_trajectory_controller**: Receives trajectory commands via action server and executes smooth movements between points with velocity control.

**IMPORTANT:** Only one of these two controllers can be active at the same time! Before testing task 3 or 4, make sure to check and change the active controller if necessary.

### Configuration

All controller configuration is located in `config/fanuc_controllers.yaml` which defines:
- System update rate (100 Hz)
- Controller types
- Joints controlled by each controller
- Command and state interfaces

## References

- [ROS2 Control documentation](https://control.ros.org/)
- [Example 7: Full tutorial with a 6DOF robot](https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html)
- [Mock Components](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html)

## Author

Krešimir Hartl  
Repository: https://github.com/KxHartl/projektiranje-autonomnih-sustava
