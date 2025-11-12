# Zadaća 1 – ROS2 Control mock sustav za Fanuc M10iA robot

Ovaj repozitorij sadrži potpuno rješenje studentske zadaće za implementaciju mock ros2_control sustava industrijskog robota Fanuc M10iA s 6 stupnjeva slobode. Projekt uključuje vizualizaciju robota, ros2_control sučelje s dva različita kontrolera te automatizirane testove funkcionalnosti.

## Struktura repozitorija

```
student-assignment-01/
└── src/fanuc_m10ia_support/
    ├── config/                       # YAML konfiguracije kontrolera
    │   └── fanuc_controllers.yaml
    ├── launch/                       # Launch datoteke za pokretanje sustava
    │   ├── view_robot.launch.py
    │   ├── fanuc_controllers.launch.py
    │   ├── publish_forward_positions.launch.py
    │   └── publish_trajectory.launch.py
    ├── urdf/                         # Robot opisi i ros2_control definicije
    │   ├── m10ia.xacro
    │   ├── m10ia_macro.xacro
    │   └── m10ia_ros2_control.xacro
    ├── meshes/                       # 3D vizualni i kolizijski modeli
    ├── rviz/                         # RViz konfiguracije
    └── test/                         # Automatizirani testovi
        ├── test_trajectory.py
        └── forward_position_publisher.py
```

## Instalacija i priprema

### 1. Preuzimanje repozitorija

```bash
git clone https://github.com/KxHartl/projektiranje-autonomnih-sustava.git
cd projektiranje-autonomnih-sustava/student-assignment-01
```

### 2. Instalacija ovisnosti

```bash
sudo apt update
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-controller-manager \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-xacro \
                 python3-pip
```

### 3. Gradnja workspace-a

```bash
colcon build
source install/setup.bash
```

## Pokretanje i testiranje

### Zadatak 1: Vizualizacija robota s GUI upravljanjem

Pokreće se vizualizacija robota u RViz-u s mogućnošću ručnog postavljanja pozicija zglobova putem GUI slidera, bez ros2_control sustava.

**Terminal 1:**
```bash
source install/setup.bash
ros2 launch fanuc_m10ia_support view_robot.launch.py
```

Rezultat: Otvara se RViz s 3D modelom robota i GUI prozorom za upravljanje zglobovima.

---

### Zadatak 2: Pokretanje s ros2_control kontrolerima

Pokreće se mock ros2_control sustav s dva kontrolera:
- `forward_position_controller` (aktivan)
- `joint_trajectory_controller` (učitan, neaktivan)

**Terminal 1:**
```bash
source install/setup.bash
ros2 launch fanuc_m10ia_support fanuc_controllers.launch.py
```

#### Ručno testiranje forward_position_controller

**Terminal 2:**
```bash
source install/setup.bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]" --once
```

Rezultat: Robot trenutačno "skoči" na zadanu poziciju.

#### Praćenje stanja zglobova

**Terminal 2:**
```bash
ros2 topic echo /joint_states
```

#### Provjera statusa kontrolera

**Terminal 2:**
```bash
ros2 control list_controllers
```

Očekivani ispis:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
forward_position_controller[forward_command_controller/ForwardCommandController] active
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive
```

#### Prebacivanje između kontrolera

**Terminal 2:**
```bash
# Deaktivirati forward_position_controller i aktivirati joint_trajectory_controller
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller

# Povratak na forward_position_controller
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller
```

#### Testiranje joint_trajectory_controller

**Terminal 2:**
```bash
source install/setup.bash
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller
cd src/fanuc_m10ia_support/test
python3 test_trajectory.py
```

Rezultat: Robot glatko izvršava trajektoriju kroz definirane točke.

---

### Zadatak 3: Automatsko objavljivanje pozicija (forward_position_controller)

Launch datoteka automatski objavljuje niz testnih pozicija na `forward_position_controller`.

**NAPOMENA:** Za ovaj zadatak `forward_position_controller` mora biti aktivan!

**Terminal 1:**
```bash
source install/setup.bash
ros2 launch fanuc_m10ia_support fanuc_controllers.launch.py
```

**Terminal 2 - Provjera i aktivacija kontrolera:**
```bash
source install/setup.bash
# Provjera statusa kontrolera
ros2 control list_controllers

# Ako je joint_trajectory_controller aktivan, prebaci na forward_position_controller
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller
```

**Terminal 3 - Pokretanje automatskog testa:**
```bash
source install/setup.bash
ros2 launch fanuc_m10ia_support publish_forward_positions.launch.py
```

Rezultat: Robot automatski "skače" kroz sljedeće pozicije (svaka 2 sekunde):
- [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
- [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
- [1.0, 0.5, 0.0, 0.0, 0.0, 0.0]
- [0.5, 0.3, -0.2, 0.0, 0.5, 0.0]
- [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

---

### Zadatak 4: Automatsko objavljivanje trajektorija (joint_trajectory_controller)

Launch datoteka automatski objavljuje testnu trajektoriju na `joint_trajectory_controller`.

**NAPOMENA:** Za ovaj zadatak `joint_trajectory_controller` mora biti aktivan!

**Terminal 1:**
```bash
source install/setup.bash
ros2 launch fanuc_m10ia_support fanuc_controllers.launch.py
```

**Terminal 2 - Provjera i aktivacija kontrolera:**
```bash
source install/setup.bash
# Provjera statusa kontrolera
ros2 control list_controllers

# Deaktivacija forward_position_controller i aktivacija joint_trajectory_controller
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller

# Provjera da je promjena uspješna
ros2 control list_controllers
```

Očekivani ispis nakon promjene:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
forward_position_controller[forward_command_controller/ForwardCommandController] inactive
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
```

**Terminal 3 - Pokretanje automatskog testa:**
```bash
source install/setup.bash
ros2 launch fanuc_m10ia_support publish_trajectory.launch.py
```

Rezultat: Robot glatko izvršava kompleksnu trajektoriju definiranu u skripti.

---

## Korisne naredbe za provjeru i debugging

```bash
# Popis svih dostupnih topica
ros2 topic list

# Praćenje trenutnih pozicija zglobova
ros2 topic echo /joint_states

# Provjera dostupnih kontrolera i njihovog stanja
ros2 control list_controllers

# Popis svih hardverskih sučelja
ros2 control list_hardware_interfaces

# Provjera transformacija između frame-ova
ros2 run tf2_ros tf2_echo base_link tool0
```

## Tehnički detalji

### Mock hardware sustav

Projekt koristi `mock_components/GenericSystem` plugin koji simulira hardversko sučelje robota bez potrebe za stvarnim hardverom. Konfiguracija se nalazi u `urdf/m10ia_ros2_control.xacro`.

### Kontroleri

- **forward_position_controller**: Prima pozicijske naredbe i trenutačno ih postavlja. Nema interpolaciju.
- **joint_trajectory_controller**: Prima trajektorijske naredbe putem action servera i izvršava glatke pokrete između točaka s kontrolom brzine.

**VAŽNO:** Samo jedan od ova dva kontrolera može biti aktivan istovremeno! Prije testiranja zadatka 3 ili 4, obvezno provjeriti i po potrebi promijeniti aktivni kontroler.

### Konfiguracija

Sva konfiguracija kontrolera nalazi se u `config/fanuc_controllers.yaml` koji definira:
- Update rate sustava (100 Hz)
- Tipove kontrolera
- Zglobove kojima upravlja svaki kontroler
- Command i state sučelja

## Reference

- [ROS2 Control dokumentacija](https://control.ros.org/)
- [Example 7: Full tutorial with a 6DOF robot](https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html)
- [Mock Components](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html)

## Autor

Krešimir Hartl  
Repozitorij: https://github.com/KxHartl/projektiranje-autonomnih-sustava
