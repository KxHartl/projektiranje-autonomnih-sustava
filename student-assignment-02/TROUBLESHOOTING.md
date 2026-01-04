# Troubleshooting Guide - Stage Simulator, SLAM i RViz

## Česti Problemi i Rješenja

### 1. "No tf published" ili "frame not found" greške

#### Problem
RViz ne prikazuje nijedan podatak, a u terminalu vidite:
```
[ERROR] [tf2]: "base_link" frame does not exist
```

#### Uzrok
- Stage simulator ne publikuje transformacije
- `enforce_prefixes` u stage_launch.py je postavljen na `True`
- TF transformacije nisu pravilno mapiranja

#### Rješenje

**1. Provjerite stage_launch.py datoteku:**

Osigurajte da je `enforce_prefixes: False`:

```python
parameters=[{
    'world_file': os.path.join(...),
    'enforce_prefixes': False,  # ← VAŽNO!
    'use_stamped_velocity': False,
    'use_static_transformations': True,
}]
```

**2. Provjerite da su robot_state_publisher i static_tf_publisher uključeni:**

```bash
ros2 node list
```

Trebali biste vidjeti:
- `/stage` - Stage simulator
- `/robot_state_publisher` - Robot state publisher
- `static_transform_publisher` - Static TF publisher
- `/rviz2` - RViz
- `/slam_toolbox` - SLAM mapper

**3. Vizualizirajte TF hijerarhiju:**

```bash
ros2 run tf2_tools view_frames.py
eog frames.pdf  # ili xdg-open frames.pdf
```

Trebala bi biti sljdeća hijerarhija:
```
map
 └── odom
     └── base_link
         └── base_scan
```

**4. Ako je problem u perzistentnim cache datotekama:**

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select student_assignment_02
source install/setup.bash
```

---

### 2. LaserScan se ne prikazuje u RViz-u

#### Problem
Map prikazuje datoteke, ali nema laser scan točaka.

#### Uzrok
- Topic `/base_scan` nije dostupan
- TF transformacija `base_link` → `base_scan` nije dostupna
- Stage ne publikuje laser podatke

#### Rješenje

**1. Provjerite dostupne topike:**

```bash
ros2 topic list | grep -E "scan|laser"
```

Trebali biste vidjeti topike kao `/base_scan`.

Ako nema topika:
- Provjerite da je Stage pravilno pokrenut
- Provjerite stage launch datoteku

**2. Provjerite tip podataka:**

```bash
ros2 topic info /base_scan
```

Trebao bi biti `sensor_msgs/msg/LaserScan`.

**3. Provjerite podatke u topiku:**

```bash
ros2 topic echo /base_scan --once
```

Trebali biste vidjeti:
```
header:
  frame_id: base_scan
angle_min: -3.141592...
angle_max: 3.141592...
ranges: [0.5, 0.6, 0.7, ...]
```

**4. RViz konfiguracija LaserScan displaya:**

Provjerite u RViz-u:
- Display → LaserScan
- Topic: `/base_scan`
- Size (m): `0.05` (za manje točke)
- Color Transformer: `Intensity` ili `FlatColor`

---

### 3. Map ne prikazuje podatke

#### Problem
RViz Map display je aktivan, ali je crn ili prazan.

#### Uzrok
- SLAM toolbox se nije pokrenuo pravilno
- `/map` topic nije dostupan
- SLAM još nije kreirao mapu

#### Rješenje

**1. Provjerite dostupnost `/map` topika:**

```bash
ros2 topic list | grep map
```

Trebali biste vidjeti:
- `/map` - Aktualna mapa
- `/map_updates` - Ažuriranja mape

**2. Provjerite je li SLAM toolbox pokrenut:**

```bash
ros2 node list | grep slam
```

Trebali biste vidjeti `/slam_toolbox`.

**3. Provjerite SLAM logove:**

```bash
ros2 launch student_assignment_02 online_async_launch.py use_sim_time:=true --log-level debug
```

Tražite poruke kao:
```
[slam_toolbox]: Initialized SLAM Toolbox
[slam_toolbox]: Received scan
```

**4. Provjerite scan_topic parametar:**

U `config/mapper_params_online_async.yaml`:

```yaml
scan_topic: /base_scan  # Trebala bi biti točna!
```

Ako je Stage objavljivao na drugačijem topiku, trebate ga promijeniti ili dodati remapping u launch datoteku:

```python
remappings=[
    ('/scan', '/base_scan'),
]
```

**5. Čekajte da robot krene:**

SLAM trebau podacima da robot krene da počne mapiranje. Premjestite robota ili ga kontrolirajte:

```bash
# U drugom terminalu
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**6. Očistite SLAM cache:**

Ako je SLAM bio pokrenut s lošim parametrima:

```bash
rm -rf ~/.ros/slam_toolbox  # Očisti cache
```

---

### 4. Stage simulator se ne pokre"e ili se rušiti

#### Problem
Stage proces se odmah zaustavlja ili se gešt u startu.

#### Uzrok
- World datoteka ne postoji ili je oštećena
- Loša stage konfiguracija
- Stage ROS2 nije pravilno instaliran

#### Rješenje

**1. Provjerite da world datoteka postoji:**

```bash
ls -la ~/ros2_ws/install/student_assignment_02/share/student_assignment_02/world/
```

Trebala bi biti `map_01.world`.

**2. Provjerite Stage instalaciju:**

```bash
ros2 pkg list | grep stage
ros2 pkg prefix stage_ros2
```

Ako nije instaliran:

```bash
sudo apt-get install ros-humble-stage-ros2
```

**3. Provjerite world datoteku:**

Otvorite `world/map_01.world` i provjerite da ima valid Stage syntax.

Primjer minimalne world datoteke:

```
# Primjer map_01.world
include "$(find stage_ros2)/world/defaults.inc"

worldsize [-20 -20 20 20]

# Robot
robot (
  name "robot_0"
  pose [0 0 0 0]
  localization "odom"
  localization_origin [0 0 0 0]
)
```

**4. Pokrenite Stage s verbose izlazom:**

```bash
ros2 launch student_assignment_02 stage_launch.py stage:=true rviz:=false --log-level debug
```

---

### 5. Topic remapping ne radi

#### Problem
Topic se ne remap-uje na očekivanu vrijednost.

#### Rješenje

Provjerite da su remappings pravilno definirani u launch datoteci:

```python
remappings=[
    ('/scan', '/base_scan'),
    ('tf', 'tf'),
]
```

Provjerite topike:

```bash
ros2 topic list
```

---

### 6. A* putanja se ne prikazuje

#### Problem
Path planning čvor je pokrenut, ali putanja se ne prikazuje u RViz-u.

#### Uzrok
- Map nije dostupna
- Marker publisher nije aktiviran
- RViz Marker display nije pravilno konfiguriran

#### Rješenje

**1. Provjerite je li path_planning_node pokrenut:**

```bash
ros2 node list | grep path_planning
```

**2. Provjerite marker topic:**

```bash
ros2 topic list | grep marker
```

Trebali biste vidjeti `/visualization_marker_array`.

**3. RViz Marker konfiguracija:**

U RViz-u:
- Add Display → Marker
- Topic: `/visualization_marker_array`
- Enabled: true

**4. Provjerite logove path planning čvora:**

```bash
ros2 launch student_assignment_02 complete_mapping_launch.py --log-level debug 2>&1 | grep path_planning
```

---

### 7. SLAM Toolbox se ne sinkronizira s odom

#### Problem
Robot se može vidjeti u Stage-u, ali pozicija u SLAM-u se razlikuje.

#### Uzrok
- Odom frame nije pravilno postavljen
- TF transformacija odom → base_link nije dostupna

#### Rješenje

**1. Provjerite odom topic:**

```bash
ros2 topic info /odom
```

Trebali biste vidjeti `nav_msgs/msg/Odometry`.

**2. SLAM parametri:**

U `config/mapper_params_online_async.yaml`:

```yaml
odom_frame: odom
map_frame: map
base_frame: base_link
```

**3. Provjerite transform_publish_period:**

Za bolju sinkronizaciju, povećajte publikaciju:

```yaml
transform_publish_period: 0.02  # 50 Hz
```

---

### 8. Kolaboracija između više čvorova ne radi

#### Problem
Čvorovi se ne "vide" međusobno.

#### Rješenje

**1. Provjerite ROS_DOMAIN_ID:**

```bash
echo $ROS_DOMAIN_ID
```

Ako je različit između terminala, svi trebaju biti isti ili prazni:

```bash
unset ROS_DOMAIN_ID
```

**2. Provjerite ROS_LOCALHOST_ONLY:**

```bash
echo $ROS_LOCALHOST_ONLY
```

Za lokalnu komunikaciju, trebao bi biti postavljen na 1:

```bash
export ROS_LOCALHOST_ONLY=1
```

---

## Debugging Alati

### RViz Diagnostika

```bash
# Prikazuje sve dostupne topike
ros2 topic list

# Prikazuje TF hijerarhiju
ros2 run tf2_tools view_frames.py
eog frames.pdf

# Provjerava specifičan topic
ros2 topic info /map
ros2 topic echo /map --once

# ROS 2 bag za snimanje podataka
ros2 bag record -o session /map /base_scan /tf
```

### Logovanje

```bash
# Svi logovi
ros2 launch student_assignment_02 complete_mapping_launch.py --log-level debug

# Samo specifičnog paketa
ros2 run --launch-prefix="ros2 run rlcpp_py create_logger" student_assignment_02 path_planning_node
```

---

## Kontakt i Dodatna Pomoć

Ako problem persisti:

1. Provjerite sve TF transformacije
2. Očistite build direktoirum i ponovno kompajlirajte
3. Provjerite verzije instaliranih paketa
4. Pogledajte ROS2 dokumentaciju za specifičan paket

---

**Zadnja ažuriranja:** January 4, 2026
