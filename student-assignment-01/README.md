# Zadaća 1 - ROS2 Control Mock Sustav za Fanuc M10iA Robot

## Opis

Ovaj repozitorij sadrži implementaciju ros2_control mock sustava za 6DoF Fanuc M10iA robota.

## Struktura paketa

```
student-assignment-01/
└── src/
    └── fanuc_m10ia_support/
        ├── config/
        │   └── fanuc_controllers.yaml
        ├── launch/
        │   ├── view_robot.launch.py
        │   └── fanuc_controllers.launch.py
        ├── urdf/
        │   ├── m10ia.xacro
        │   ├── m10ia_macro.xacro
        │   └── m10ia_ros2_control.xacro
        ├── meshes/
        └── rviz/
```

## Preuzimanje i instalacija

### 1. Kloniranje repozitorija

```bash
git clone https://github.com/KxHartl/projektiranje-autonomnih-sustava.git
cd projektiranje-autonomnih-sustava/student-assignment-01
```

### 2. Instalacija ovisnosti

```bash
sudo apt update
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager
sudo apt install ros-humble-joint-state-publisher-gui ros-humble-xacro
```

### 3. Build workspace

```bash
cd ~/projektiranje-autonomnih-sustava/student-assignment-01
colcon build
source install/setup.bash
```

## Pokretanje

### 1. Vizualizacija robota s GUI upravljanjem

Ova launch datoteka pokreće vizualizaciju robota u RViz-u s mogućnošću upravljanja zglobovima kroz GUI.

```bash
ros2 launch fanuc_m10ia_support view_robot.launch.py
```

**Značajke:**
- Prikazuje 3D model Fanuc M10iA robota
- GUI slider za ručno postavljanje pozicija zglobova
- Bez ros2_control sustava

---

### 2. Robot s ros2_control kontrolerima

Ova launch datoteka pokreće robota s aktivnim `forward_position_controller` i učitanim (ali neaktivnim) `joint_trajectory_controller`.

```bash
ros2 launch fanuc_m10ia_support fanuc_controllers.launch.py
```

**Što se pokreće:**
- Controller Manager s mock hardverskim sustavom
- Robot State Publisher
- RViz2 za vizualizaciju
- `joint_state_broadcaster` (aktivan)
- `forward_position_controller` (aktivan)
- `joint_trajectory_controller` (učitan, neaktivan)

#### Testiranje forward_position_controller

Nakon pokretanja, možete slati naredbe pozicija:

```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]"
```

#### Promjena između kontrolera

Za deaktivaciju `forward_position_controller` i aktivaciju `joint_trajectory_controller`:

```bash
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller
```

Za povratak na `forward_position_controller`:

```bash
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller
```

#### Provjera statusa kontrolera

```bash
ros2 control list_controllers
```

---

## Konfiguracija kontrolera

### Forward Position Controller

Kontroler prima naredbe pozicija za sve zglobove istovremeno preko topic-a `/forward_position_controller/commands`.

**Konfiguracija** (`config/fanuc_controllers.yaml`):
```yaml
forward_position_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    interface_name: position
```

### Joint Trajectory Controller

Kontroler prima trajektorije preko action servera i izvršava glatke pokrete između točaka.

**Konfiguracija** (`config/fanuc_controllers.yaml`):
```yaml
joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

## ROS2 Control Mock Sustav

Mock sustav omogućava testiranje kontrolera bez potrebe za pravim hardverom. Konfiguriran je u `urdf/m10ia_ros2_control.xacro`:

```xml
<ros2_control name="FanucM10iaSystem" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>
  </hardware>
  <joint name="joint_1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- ... ostali zglobovi ... -->
</ros2_control>
```

## Korisne naredbe

### Provjera dostupnih topic-a

```bash
ros2 topic list
```

### Praćenje stanja zglobova

```bash
ros2 topic echo /joint_states
```

### Lista aktivnih kontrolera

```bash
ros2 control list_controllers
```

### Učitavanje kontrolera ručno

```bash
ros2 control load_controller <controller_name>
ros2 control set_controller_state <controller_name> active
```

## Reference

- [ROS2 Control Documentation](https://control.ros.org/)
- [Example 7: Full tutorial with a 6DOF robot](https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html)
- [Mock Components](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html)

## Autor

KreŠimir Hartl  
Repozitorij: https://github.com/KxHartl/projektiranje-autonomnih-sustava
