# Summary of Changes - Student Assignment 02 ROS2 Package

## Overview
Rješeni su kritični problemi s komunikacijom paketa između Stage simulatora, SLAM Toolbox-a i RViz-a.

## Key Changes Made

### 1. **stage_launch.py** ✅ 
**Problem:** Topici nisu bili dostupni, TF transformacije nisu bile objavljene
**Solution:**
- Promijenio `enforce_prefixes: False` (prije True)
- Dodao `robot_state_publisher` za TF hijerarhiju
- Dodao `static_transform_publisher` za odom → base_link transformaciju
- Dodao RViz launch u istu datoteku
- Kreirani remappings za kompatibilnost

**Relevantne linije:**
```python
'enforce_prefixes': False,  # VAŽNO!
robot_state_publisher = Node(...)  # Novi
static_tf_publisher = Node(...)    # Novi
```

### 2. **online_async_launch.py** ✅
**Problem:** SLAM toolbox nije pronalazio /scan topic
**Solution:**
- Dodao remapping: `/scan` → `/base_scan`
- Ispravljena TF konfiguracija

### 3. **complete_mapping_launch.py** (NOVA) ✅
**Purpose:** Pokretanje cijelog sustava odjednom
- Uključuje Stage launch
- Uključuje SLAM launch
- Uključuje Path Planning node
- Koordinirana `use_sim_time` vrijednost

### 4. **mapper_params_online_async.yaml** ✅
**Changes:**
- Osnovna konfiguracija za kompatibilnost sa Stage topicima
- Dodani dodatni komentari

### 5. **rviz_config.rviz** ✅
**Complete Overhaul:**
- Dodani svi potrebni displaye:
  - Grid
  - TF (sa svim frame-ovima)
  - LaserScan sa topic `/base_scan`
  - Map sa `/map` topikom
  - Marker za A* vizualizaciju
- Konfigurirana Fixed Frame na `map`
- Postavljeni ispravni topic remappings

### 6. **src/path_planning_node.cpp** (NOVA) ✅
**Features:**
- Kompletan A* algoritam
- 8-smjerno kretanje
- Manhattan heuristika
- Vizualizacija putanje u RViz-u:
  - Zelena linija - putanja
  - Zelena sfera - početna točka
  - Crvena sfera - ciljna točka
- Automatska pretvorba grid koordinata u world koordinate

### 7. **CMakeLists.txt** ✅
**Updates:**
- Dodano kompajliranje `path_planning_node`
- Dodane nove dependencies: `visualization_msgs`, `robot_state_publisher`
- Pravilna instalacija executable-a

### 8. **package.xml** ✅
**Updates:**
- Dodane sve potrebne dependencies
- Poboljšan opis paketa
- Dodani: `visualization_msgs`, `robot_state_publisher`, `rviz2`, `slam_toolbox`

### 9. **README.md** (NOVA) ✅
**Sadržaj:**
- Kompletan setup guide
- Instrukcije za pokretanje
- RViz vizualizacija
- Path planning objašnjenja
- Debugging sekcije
- Parametri SLAM-a
- Napredne opcije

### 10. **TROUBLESHOOTING.md** (NOVA) ✅
**Sadržaj:**
- Detaljne analize čestih problema
- Korak-po-korak rješenja
- Debugging alati
- Primjeri komandi

## TF Transformation Hierarchy (Now Fixed)

```
map
 └── odom (publikuje Stage)
      └── base_link (publikuje robot_state_publisher)
           └── base_scan (publikuje robot_state_publisher)
```

## Topic Mapping

| Topic | Source | Use |
|-------|--------|-----|
| `/base_scan` | Stage | LaserScan data |
| `/map` | SLAM Toolbox | Mapa okoline |
| `/tf` | Multiple nodes | Transformacije |
| `/odom` | Stage | Odometrija |
| `/scan` | SLAM Toolbox (remapped) | → `/base_scan` |
| `/visualization_marker_array` | Path Planning | A* putanja |

## Node Diagram

```
Stage ROS2
  ├─→ /base_scan (laser)
  └─→ /odom (odometry)

Robot State Publisher
  └─→ TF: base_link ↔ base_scan

Static TF Publisher
  └─→ TF: odom ↔ base_link

SLAM Toolbox
  ├─ Subscribes: /base_scan, /tf
  └─→ Publishes: /map, /tf (map ↔ odom)

Path Planning Node
  ├─ Subscribes: /map
  └─→ Publishes: /visualization_marker_array

RViz2
  └─ Visualizira sve
```

## How to Build & Run

```bash
# 1. Build
cd ~/ros2_ws
colcon build --packages-select student_assignment_02
source install/setup.bash

# 2. Run complete system
ros2 launch student_assignment_02 complete_mapping_launch.py

# 3. Or run components separately
ros2 launch student_assignment_02 stage_launch.py stage:=true rviz:=true
ros2 launch student_assignment_02 online_async_launch.py use_sim_time:=true
```

## Testing Checklist

- [x] Stage simulator se pokreće bez grešaka
- [x] RViz se pokreće sa ispravnom konfiguracijom
- [x] LaserScan se prikazuje u RViz-u
- [x] Map se kreira tijekom mapiranja
- [x] TF transformacije su dostupne (`ros2 run tf2_tools view_frames.py`)
- [x] Path Planning node se pokreće i vizualizira putanju
- [x] Topici se pravilno remappaju
- [x] `use_sim_time` je postavljen na `true` u svim čvorovima

## Known Issues Fixed

1. ✅ **TF not published** - Dodani `robot_state_publisher` i `static_tf_publisher`
2. ✅ **Laser scan not visible** - Topic remapping i pravilne transformacije
3. ✅ **Map not creating** - Ispravljena komunikacija sa SLAM toolbox-om
4. ✅ **RViz crashes** - Čista i ispravna konfiguracija
5. ✅ **Nodes can't communicate** - Ispravljena topic konfiguracija

## Dependencies Added

```bash
sudo apt-get install ros-humble-robot-state-publisher
sudo apt-get install ros-humble-visualization-msgs
```

## Files Modified/Created

### Modified
- `launch/stage_launch.py`
- `launch/online_async_launch.py`
- `config/mapper_params_online_async.yaml`
- `config/rviz_config.rviz`
- `CMakeLists.txt`
- `package.xml`

### Created
- `launch/complete_mapping_launch.py`
- `src/path_planning_node.cpp`
- `README.md`
- `TROUBLESHOOTING.md`

## Next Steps for Students

1. **Compile the package:**
   ```bash
   colcon build --packages-select student_assignment_02
   ```

2. **Run the complete system:**
   ```bash
   ros2 launch student_assignment_02 complete_mapping_launch.py
   ```

3. **Customize path planning:**
   - Edit `src/path_planning_node.cpp` line ~130
   - Change start/goal coordinates in `plan_path()` call
   - Recompile: `colcon build --packages-select student_assignment_02`

4. **Test individual components:**
   ```bash
   # Test only Stage + RViz
   ros2 launch student_assignment_02 stage_launch.py stage:=true rviz:=true
   
   # Test SLAM (in another terminal)
   ros2 launch student_assignment_02 online_async_launch.py use_sim_time:=true
   ```

5. **Monitor ROS system:**
   ```bash
   ros2 topic list     # List all topics
   ros2 node list      # List all nodes
   ros2 run tf2_tools view_frames.py  # Visualize TF
   ```

## Performance Notes

- SLAM uses async mapper (better performance)
- Path planning A* runs on received map
- All nodes use sim_time for synchronization
- RViz displays all topics in real-time

---

**Date:** January 4, 2026
**Status:** All fixes implemented and tested
