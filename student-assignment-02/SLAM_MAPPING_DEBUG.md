# 🔍 SLAM MAPPING DEBUGIRANJE

## Problem 1: "Failed to compute odom pose"

### Što znači?

SLAM toolbox se ne može orijentirati u prostoru jer:
1. Nema dostupne `/base_scan` laser podataka
2. Nema dostupnog `odom → base_link` TF-a
3. Laser senzor nije pravilno konfiguriran u Stage-u

---

## Problem 2: "no map received"

RViz ne primjenjuje mapu jer SLAM toolbox još nije emitirao `/map` topic.

---

## 🔧 RJEŠENJE

### Korak 1: Provjeri da li Stage emitira laser podatke

```bash
# U novom terminalu (nakon pokretanja mapping_launch.py)
ros2 topic list

# Trebalo bi vidjeti:
# /base_scan         ← LASER PODACI
# /cmd_vel
# /ground_truth
# /odom
# /tf
# /tf_static
```

Ako **NEMA** `/base_scan`, problem je što Stage ne emitira laser podatke!

```bash
# Provjera sadržaja /base_scan
ros2 topic echo /base_scan

# Trebalo bi vidjeti:
# header:
#   stamp: ...
#   frame_id: base_scan
# angle_min: -3.14
# angle_max: 3.14
# ranges: [8.0, 8.0, 7.95, ...]
```

---

### Korak 2: Provjeri TF stablo

```bash
# Visualizacija TF transformacija
ros2 run tf2_tools view_frames.py
cat frames.pdf  # Otvori sa nekom aplikacijom

# Trebalo bi vidjeti:
# map
#  └─ odom
#      └─ base_link
#          └─ base_scan
```

Ako nema `odom` ili `base_link`, tu je problem!

```bash
# Detaljni TF debug
ros2 topic echo /tf
ros2 topic echo /tf_static
```

---

### Korak 3: Provjeri SLAM node log

```bash
# Gdje je log?
cat ~/.ros/log/latest/slam_toolbox*.log

# Trebalo bi vidjeti:
# [INFO] [async_slam_toolbox_node]: Node initialized
# [INFO] [...]: Pose transformation: ...

# LOŠE znakove:
# [ERROR] Failed to compute odom pose
# [WARN] No laser data received
```

---

## 🎯 Česti Problemi i Rješenja

### Problem A: Stage ne emitira /base_scan

**Uzrok:** World file nema laserske definicije u robots.inc

**Rješenje:**
1. Provjeri `world/include/robots.inc` - trebalo bi imati `laser` blok
2. Provjeri da je `pioneer2dx_with_laser` korišten u `map_01.world`

```bash
# Provjera
grep -i "laser" src/student_assignment_02/world/include/robots.inc
grep -i "pioneer2dx_with_laser" src/student_assignment_02/world/map_01.world
```

---

### Problem B: TF transformacije nedostaju

**Uzrok:** `robot_state_publisher` ili `static_transform_publisher` nije pokrenut

**Rješenje:**
Provjerite `stage_launch.py` - trebalo bi imati:

```python
# 1. robot_state_publisher node
robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    ...
)

# 2. static_transform_publisher node
static_tf_publisher = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    ...
)
```

---

### Problem C: SLAM ne primjenjuje laser podatke

**Uzrok:** Topic remapping nije ispravan

**Rješenje:**
Provjerite `online_async_launch.py`:

```python
remappings=[
    ('/scan', '/base_scan'),  # VAŽNO!
    ('tf', 'tf'),
    ('tf_static', 'tf_static'),
]
```

---

## ✅ Checklist - Što trebate provjeriti

```bash
# 1. Terminal gdje je pokrenuta mapping_launch.py
ros2 topic list | grep -E "base_scan|map|odom|tf"

# Trebalo bi vidjeti:
# /base_scan
# /map
# /odom
# /tf
# /tf_static
```

Ako neke od ovih nedostaju, javiti se!

```bash
# 2. Laser podaci
ros2 topic echo /base_scan | head -20

# 3. TF stablo
ros2 topic echo /tf_static
ros2 topic echo /tf

# 4. SLAM status
ros2 node list | grep slam
```

---

## 🚀 Brzo Testiranje - Minimal Setup

Ako gornji koraci ne rade, pokušajte s minimalnom konfiguracijom:

```bash
# Terminal 1: RViz bez posebne konfiguracije
rviz2

# Terminal 2: Only Stage (bez SLAM-a)
ros2 launch student_assignment_02 stage_launch.py use_sim_time:=true

# Terminal 3: Provjera topic-a
ros2 topic list
ros2 topic echo /base_scan | head -5
```

Ako `/base_scan` postoji, SLAM može raditi.

---

## 📊 Očekivani Topici During Mapping

| Topic | Type | Source | Očekivano |
|-------|------|--------|----------|
| `/base_scan` | sensor_msgs/LaserScan | Stage | ✅ Podaci svaki frame |
| `/tf` | tf2_msgs/TFMessage | Various | ✅ Kontinuirani |
| `/tf_static` | tf2_msgs/TFMessage | static_tf_publisher | ✅ Statički |
| `/odom` | nav_msgs/Odometry | Stage | ✅ Pozicija robota |
| `/map` | nav_msgs/OccupancyGrid | SLAM | ✅ Gradi se tijekom mapiranja |
| `/slam_toolbox/map` | nav_msgs/OccupancyGrid | SLAM | ✅ Ista kao /map |

---

## 🔴 Ako Problem Persists

### Opcionalno: Koristi minimal launch bez SLAM-a

```bash
# Kreiraj privremeni launch file bez SLAM-a
ros2 launch student_assignment_02 stage_launch.py use_sim_time:=true rviz:=true
```

Ako Stage i RViz rade dobro, problem je u SLAM toolbox konfiguraciji.

### Debug SLAM s više logiranja

```bash
ros2 launch student_assignment_02 mapping_launch.py --debug
```

### Registracija ROS Repozitorija

Možda SLAM toolbox nije pravilno instaliran:

```bash
sudo apt-get install ros-humble-slam-toolbox
ros2 pkg prefix slam_toolbox
```

Trebalo bi ispisati `/opt/ros/humble`

---

## 💡 Što se trebalo Desiti

### Faza 1: Stage se Pokreće
```
✅ Stage simulator startuje
✅ Robot se pojavljuje u simulaciji
✅ Laser senzor emitira /base_scan podatke
```

### Faza 2: SLAM se Inicijalizira
```
✅ SLAM toolbox startuje
✅ Čeka /base_scan podatke
✅ Počinje mapirati
```

### Faza 3: RViz Primjenjuje Mapu
```
✅ RViz prikazuje /map
✅ Mapa se ažurira kako robot ide
✅ Vidite "live map building"
```

---

**Status:** 🔧 Troubleshooting

**Ako Problem Ostaje:** Javiti output od:
```bash
ros2 topic list
ros2 topic echo /base_scan | head -5
ros2 topic echo /tf_static
```
