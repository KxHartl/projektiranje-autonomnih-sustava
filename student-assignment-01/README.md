# Fanuc M10iA - ros2_control Mock Sustav

Mock sustav za 6-DOF Fanuc M10iA robot implementiran s ros2_control okvirom.

## Preduvjeti

- Ubuntu 22.04
- ROS 2 Humble

**Instalacija paketa:**
```bash
sudo apt update
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-xacro ros-humble-rviz2
```

## Instalacija

```bash
# Kreiranje workspace-a
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Kloniranje repozitorija
git clone https://github.com/KxHartl/projektiranje-autonomnih-sustava.git

# Instalacija zavisnosti
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

**Dodajte u `~/.bashrc` za automatsko sourcing:**
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Struktura

```
student-assignment-01/
├── config/
│   └── fanuc_controllers.yaml      # Konfiguracija controllera
├── launch/
│   ├── view_robot.launch.py        # Launch 1: GUI vizualizacija
│   ├── robot_controller.launch.py  # Launch 2: Controlleri
│   ├── forward_position.launch.py  # Launch 3: Pozicije
│   └── trajectory_control.launch.py # Launch 4: Trajektorije
├── urdf/
│   ├── fanuc_m10ia.urdf.xacro
│   ├── fanuc_m10ia_macro.xacro
│   └── fanuc.ros2_control.xacro
└── meshes/visual/
```

## Pokretanje

### 1. Vizualizacija s GUI kontrolom

```bash
ros2 launch student-assignment-01 view_robot.launch.py
```

Pokreće RViz i GUI za ručno upravljanje zglobovima.

### 2. Robot s controllerima

```bash
ros2 launch student-assignment-01 robot_controller.launch.py
```

Pokreće ros2_control mock sustav s controllerima:
- `forward_position_controller` (aktivan)
- `joint_trajectory_controller` (učitan, neaktivan)
- `joint_state_broadcaster`

**Provjera statusa:**
```bash
ros2 control list_controllers
```

**Prebacivanje controllera:**
```bash
# Na trajectory controller
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller

# Natrag na position controller
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller
```

### 3. Slanje pozicija (forward_position_controller)

**Prije:** Osigurajte da je Launch 2 pokrenut i `forward_position_controller` aktivan.

```bash
# Terminal 2
ros2 launch student-assignment-01 forward_position.launch.py
```

**Ručno slanje:**
```bash
# Neutralna pozicija
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

# Testna pozicija
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3, 0.8, -0.2, 0.4, -0.1]"
```

### 4. Slanje trajektorija (joint_trajectory_controller)

**Prije:**
1. Pokrenite Launch 2
2. Prebacite na trajectory controller (vidjeti gore)

```bash
# Terminal 2
ros2 launch student-assignment-01 trajectory_control.launch.py
```

**Ručno slanje trajektorije:**
```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
  points: [
    {positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 0, nanosec: 0}},
    {positions: [0.5, -0.5, 0.5, -0.5, 0.5, -0.5], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2, nanosec: 0}},
    {positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 4, nanosec: 0}}
  ]
}"
```

## Testiranje

### Test 1: GUI vizualizacija
```bash
ros2 launch student-assignment-01 view_robot.launch.py
```
Očekivano: RViz + GUI klizači za zglobove

### Test 2: Forward Position Controller
```bash
# Terminal 1
ros2 launch student-assignment-01 robot_controller.launch.py

# Terminal 2
ros2 control list_controllers

# Terminal 3
ros2 launch student-assignment-01 forward_position.launch.py
```
Očekivano: Robot se kreće prema zadanim pozicijama (bez interpolacije)

### Test 3: Joint Trajectory Controller
```bash
# Terminal 1
ros2 launch student-assignment-01 robot_controller.launch.py

# Terminal 2
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller

# Terminal 3
ros2 launch student-assignment-01 trajectory_control.launch.py
```
Očekivano: Glatko kretanje s interpolacijom između točaka

## Provjera sustava

```bash
# Topici
ros2 topic list
ros2 topic echo /joint_states

# Controlleri
ros2 control list_controllers
ros2 control list_hardware_interfaces

# Frekvencija
ros2 topic hz /joint_states
```

## Česti problemi

### Paket nije pronađen
```bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

### Controller se ne učitava
Provjerite YAML konfiguraciju i imena zglobova:
```bash
cat ~/ros2_ws/src/student-assignment-01/config/fanuc_controllers.yaml
ros2 control list_controller_types
```

### Robot se ne kreće
- Provjerite status controllera: `ros2 control list_controllers`
- Provjerite sučelja: `ros2 control list_hardware_interfaces`
- Provjerite ograničenja zglobova u URDF-u

### RViz ne prikazuje robota
- Dodajte RobotModel display
- Postavite Fixed Frame na "world" ili "base_link"
- Provjerite: `ros2 topic echo /robot_description --once`

### Mock hardware greška
Provjerite xacro sintaksu:
```bash
xacro ~/ros2_ws/src/student-assignment-01/urdf/fanuc_m10ia.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

## Napomene

**Mock sustav** simulira hardversko sučelje bez fizičkog robota - omogućuje testiranje algoritama i integracija bez hardvera.

**Controlleri:**
- `forward_position_controller` - direktno prosljeđivanje pozicija bez interpolacije
- `joint_trajectory_controller` - glatko kretanje s interpolacijom i PID kontrolom (preporučeno)

**Najbolje prakse:**
- Uvijek sourcajte okruženje
- Provjerite status controllera prije slanja naredbi
- Koristite razumne vrijednosti (u radijanima)
- Testirajte u malim koracima

## Resursi

- [ros2_control dokumentacija](https://control.ros.org/)
- [Example 7: 6DOF robot](https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html)
- [ros2_controllers](https://control.ros.org/humble/doc/ros2_controllers/doc/controllers_index.html)

---

**Projekt:** Projektiranje autonomnih sustava  
**GitHub:** https://github.com/KxHartl/projektiranje-autonomnih-sustava