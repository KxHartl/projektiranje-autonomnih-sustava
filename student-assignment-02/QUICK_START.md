# ⚡ QUICK START - ROS 2 A* Path Planning

**Brzi početak u 5 minuta!**

---

## Installation & Build (2 min)

```bash
# 1. Setup ROS 2
source /opt/ros/humble/setup.bash

# 2. Clone i prebaci se u direktorij
cd ~/projektiranje-autonomnih-sustava/student-assignment-02

# 3. Build paket
rm -rf build install log
colcon build
source install/setup.bash
```

---

## Test 1: Mapiranje (2 min) 🗺️

### Terminal 1 - RViz
```bash
rviz2 -d config/rviz_config.rviz
```

### Terminal 2 - Stage + SLAM
```bash
ros2 launch student_assignment_02 mapping_launch.py
```

### Terminal 3 - Teleoperation (navigiraj robot)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel
```

**Kontrole:** `w/a/s/d` za kretanje, `q` za izlaz

### Terminal 4 - Spremi mapu
Kada je mapiranje gotovo:
```bash
mkdir -p ~/my_map
cd ~/my_map
ros2 run nav2_map_server map_saver_cli --fmt pgm -f map
```

✅ **Mapa je spremljena u `~/my_map/map.yaml`**

---

## Test 2: Path Planning & Navigacija (1 min) 🤖

### Terminal 1 - RViz
```bash
rviz2 -d config/rviz_config.rviz
```

### Terminal 2 - Stage + Path Planning + Navigation
```bash
ros2 launch student_assignment_02 path_planning_launch.py
```

### U RViz Prozoru
1. Odaberite **"2D Goal Pose"** tool (Top toolbar)
2. Kliknite i povucite u mapi da postavite cilj
3. Vidite:
   - **Zelena linija** = A* putanja
   - **Zelena sfera** = početak
   - **Crvena sfera** = cilj
   - **Robot se giba** prema cilju!

4. Postavite novi cilj i ponovite

---

## 📊 Sažetak Komunikacija

```
┌──────────────────────────────────────┐
│ MAPIRANJE (Korak 1)                     │
│                                          │
│ Stage        Map (OccupancyGrid)        │
│   │              │                       │
│   └────────> SLAM (map.yaml)      │
│                  │                       │
│                  └────────> RViz (vizualizacija)
│                                          │
└──────────────────────────────────────┘

┌──────────────────────────────────────┐
│ PATH PLANNING & NAVIGACIJA (Korak 2)    │
│                                          │
│ RViz (2D Goal Pose)                     │
│        │                                │
│        └─> /goal_pose                   │
│               │                        │
│        Map Server (✓ my_map)             │
│        │                                │
│        └─> Path Planning Node (A*)     │
│               │                        │
│               └─> /planned_path            │
│                  │                     │
│        Goal Navigation Node │            │
│               │                        │
│               └─> /cmd_vel              │
│                  │                     │
│               Stage Robot │              │
│                  │                     │
│               /visualization_marker_array│
│                  │                     │
│               RViz (✓ Zelena putanja)   │
│                                          │
└──────────────────────────────────────┘
```

---

## 📁 Datoteke

| Datoteka | Opis |
|----------|------|
| `path_planning_node.py` | A* putanja planiranje |
| `goal_navigation_node.py` | Slijeđenje putanje |
| `a_star.py` | A* algoritam |
| `utils.py` | Utility funkcije |
| `mapping_launch.py` | Stage + SLAM |
| `path_planning_launch.py` | Stage + Path Planning |
| `rviz_config.rviz` | RViz konfiguracija |

---

## 🔓 Parametri Path Planning Noda

```bash
ros2 launch student_assignment_02 path_planning_launch.py \
  linear_speed:=0.3 \
  angular_speed:=0.7 \
  waypoint_tolerance:=0.2 \
  angle_tolerance:=0.3
```

| Parametar | Default | Opis |
|-----------|---------|------|
| `linear_speed` | 0.2 | Brzina naprijed (m/s) |
| `angular_speed` | 0.5 | Brzina rotacije (rad/s) |
| `waypoint_tolerance` | 0.15 | Tolerancija distancije (m) |
| `angle_tolerance` | 0.2 | Tolerancija kuta (rad) |

---

## 🐛 Greške & Rješenja

| Greška | Rješenje |
|--------|----------|
| "Package not found" | `colcon build && source install/setup.bash` |
| "Map file not found" | `ls ~/my_map/` i spremi mapu ponovo |
| "Service not available" | Provjerite da je SLAM toolbox pokrenut |
| "Robot ne ide" | Postavite cilj drugdje (možda je zauzet) |

---

## 🎈 Posebne Naredbe

```bash
# Lista svih topic-a
ros2 topic list

# Provjera putanje
ros2 topic echo /planned_path

# Provjera brzine
ros2 topic echo /cmd_vel

# TF stablo
ros2 run tf2_tools view_frames.py

# Spremi bag datoteku
ros2 bag record -o recording /map /cmd_vel /planned_path
```

---

**Gotov! 🌟**

Za detalje vidi: [README_PYTHON_A_STAR.md](README_PYTHON_A_STAR.md)
