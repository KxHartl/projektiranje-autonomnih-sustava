# Zadaća 1 - ROS2 Control Mock sustav Fanuc M10iA

Ovaj repozitorij sadrži rješenje zadaće: mock ros2_control sustav za 6DoF Fanuc M10iA robot, s automatiziranim testiranjem funkcionalnosti kontrolera.

## Struktura projekta
```
student-assignment-01/
├── src/fanuc_m10ia_support/
│   ├── config/fanuc_controllers.yaml
│   ├── launch/view_robot.launch.py
│   ├── launch/fanuc_controllers.launch.py
│   ├── urdf/m10ia.xacro
│   ├── urdf/m10ia_macro.xacro
│   ├── urdf/m10ia_ros2_control.xacro
│   ├── meshes/
│   ├── rviz/
│   └── test/test_trajectory.py
```

## 1. Preuzimanje i instalacija

```bash
git clone https://github.com/KxHartl/projektiranje-autonomnih-sustava.git
cd projektiranje-autonomnih-sustava/student-assignment-01
sudo apt update
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager \
 ros-humble-joint-state-publisher-gui ros-humble-xacro python3-pip
colcon build
source install/setup.bash
```

## 2. Pokretanje vizualizacije robota (GUI način)
```bash
ros2 launch fanuc_m10ia_support view_robot.launch.py
```
Slideri omogućuju ručno pomicanje zglobova u RViz-u bez ros2_control.

## 3. Pokretanje s ros2_control kontrolerima
```bash
ros2 launch fanuc_m10ia_support fanuc_controllers.launch.py
```
Pokreće:
- mock ros2_control hardware (`mock_components/GenericSystem`)
- `joint_state_broadcaster` (aktivan)
- `forward_position_controller` (aktivan)
- `joint_trajectory_controller` (neaktivan)

## 4. Testiranje forward_position_controller
Slanje naredbi:
```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" --once
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]" --once
ros2 topic echo /joint_states
```
Robot odmah "skače" na zadanu poziciju.

## 5. Testiranje joint_trajectory_controller (automatizirani test)
1. Aktiviraj controller:
   ```bash
   ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller
   ros2 control list_controllers
   # joint_trajectory_controller mora biti "active"
   ```
2. Pokreni test skriptu za trajektoriju:
   ```bash
   cd src/fanuc_m10ia_support/test
   chmod +x test_trajectory.py
   python3 test_trajectory.py
   ```

U RViz-u robot glatko prolazi kroz definirane točke trajektorije.

### O testnoj skripti

- Skripta koristi ROS2 action server: `/joint_trajectory_controller/follow_joint_trajectory`
- Šalje četiri točke (polazna, pomak, povratak)
- Potrebni paketi: `pip3 install rclpy control-msgs trajectory-msgs`
- Slobodno proširi trajektorije za dodatne testove

## Konfiguracija

- YAML: `src/fanuc_m10ia_support/config/fanuc_controllers.yaml` sadrži konfiguracije za oba kontrolera
- URDF/XACRO: `src/fanuc_m10ia_support/urdf/` - robot opis + ros2_control interface
- Launch: `src/fanuc_m10ia_support/launch/` - pokretanje svih načina rada
- Test: `src/fanuc_m10ia_support/test/` - automatizirane funkcionalnosti

## Detalji sustava
- Forward position kontroler radi instantno
- Joint trajectory kontroler interpolira gibanje
- Prijelaz između oba mogući u runtimeu:
```bash
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller
```

## Reference
- https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html
- https://github.com/KxHartl/projektiranje-autonomnih-sustava

## Autor
KreŠimir Hartl
