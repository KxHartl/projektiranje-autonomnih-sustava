# 🤖 STUDENT ASSIGNMENT 02 - A* Path Planning sa Mapiranjem

**Mapiranje (SLAM) → Lokalizacija (AMCL) → Path Planning (A*)**

Kompletna implementacija autonomous robotike s SLAM mapiranjem, lokalizacijom i A* algoritmom za planiranje putanje.

---

## 📋 Sadrţaj

- [Preduvjeti](#preduvjeti)
- [Instalacija](#instalacija)
- [Korak 1: Mapiranje](#korak-1-mapiranje)
- [Korak 2: Lokalizacija](#korak-2-lokalizacija)
- [Korak 3: A* Path Planning](#korak-3-a-path-planning)
- [Parametri](#parametri)
- [Troubleshooting](#troubleshooting)

---

## 📦 Preduvjeti

Trebate:
- **ROS 2 Humble**
- **Stage ROS2** simulator
- **SLAM Toolbox**
- **Nav2/AMCL**
- **Python 3.10+**

Proveravanje:
```bash
ros2 --version
ros2 pkg list | grep stage
ros2 pkg list | grep slam
```

---

## 🔧 Instalacija

### 1. Clone i Build

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02

# Clean build
rm -rf build/ install/ log/

# Build
colcon build --packages-select student_assignment_02 --symlink-install

# Source
source install/setup.bash
```

### 2. Provjera Čvorova

```bash
ros2 run student_assignment_02 a_star_path_planner --help
```

Trebalo bi vidjeti help poruku bez greške.

---

## 🗺️ KORAK 1: Mapiranje

**Cilj**: Mapirati okruženje pomoću SLAM Toolbox-a

### 1.1 Pokrenite Stage Simulator

```bash
# Terminal 1
ros2 launch stage_ros2 stage.launch.py
```

### 1.2 Pokrenite SLAM Mapping

```bash
# Terminal 2
ros2 launch student_assignment_02 mapping_complete_launch.py
```

RViz će prikazati mapu kako se gradi.

### 1.3 Upravljanje Robotom

```bash
# Terminal 3
ros2 run turtlebot3_teleop teleop_keyboard
```

**Upravljanje**: `w`=naprijed, `s`=unazad, `a`=lijevo, `d`=desno, `x`=stop

### 1.4 Spremi Mapu

Provjerite koji je broj zadnje mape:
```bash
ls ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/mapped_maps/
```

Ako je zadnja `map_05`, spremi kao `map_06`:

```bash
# Terminal 4 - Zamijeni USERNAME s vašim korisničkim imenom!
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name:
    data: '/home/USERNAME/FSB/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/mapped_maps/map_06/map_06'"
```

**GOTOVO MAPIRANJE!** ✅

---

## 📍 KORAK 2: Lokalizacija

**Cilj**: Lokalizirati robota s AMCL koristeći spremljenu mapu

### 2.1 Pokrenite Stage (ako nije)

```bash
# Terminal 1
ros2 launch stage_ros2 stage.launch.py
```

### 2.2 Pokrenite Lokalizaciju

**Važno**: Zamijeni `map_06` s brojem vaše mape!

```bash
# Terminal 2
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_06
```

RViz će prikazati:
- Mapu
- Robot poziciju
- Particle filter (crvene strelice)

### 2.3 Inicijalizacija

```bash
# Terminal 3 - Pokrenite robota da se lokalizira
ros2 run turtlebot3_teleop teleop_keyboard
```

**Važno**: Pokrenite robota (`w` tipka) dok se crvene strelice ne okrenu u zelene! ✅

To znači da je robot lokaliziran.

### 2.4 Provjera TF

```bash
# Terminal 4
ros2 run tf2_tools view_frames
```

Trebalo bi vidjeti:
```
map → odom → base_link
```

**GOTOVO LOKALIZACIJA!** ✅

---

## 🎯 KORAK 3: A* Path Planning

**Cilj**: Planirati putanju s A* algoritmom s inflation bufferom

### 3.1 Pokrenite Stage + Lokalizaciju

```bash
# Terminal 1: Stage
ros2 launch stage_ros2 stage.launch.py

# Terminal 2: Lokalizacija (zamijeni map_06!)
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_06

# Terminal 3 (Opciono): Teleop
ros2 run turtlebot3_teleop teleop_keyboard
```

### 3.2 Pokrenite A* Path Planner

```bash
# Terminal 4
ros2 launch student_assignment_02 a_star_path_planner.launch.py
```

Trebali biste vidjeti:
```
[INFO] A* Path Planner Node: Started
[INFO] Inflation distance: 0.2m
[INFO] Koristi base_link za početnu točku
```

### 3.3 Pokrenite RViz

```bash
# Terminal 5
ros2 run rviz2 rviz2
```

### 3.4 RViz Konfiguracija

1. **Fixed Frame**: Postavite na `map`
2. **Add** → **By topic**:
   - `/map` (OccupancyGrid)
   - `/planned_path` (Path)
   - `/path_planning_visualization` (Marker)
   - `/inflation_buffer_visualization` (Marker)

### 3.5 Postavljanje Cilja

1. Kliknite **2D Goal Pose** tool (zeleni strelica)
2. Kliknite na mapu gdje želite cilj
3. **Putanja se planira automatski!**

**Trebali biste vidjeti**:
- 🟢 **Zelena linija** = Putanja
- 🟠 **Narančasti kubici** = Buffer (0.2m od zidova)
- 🔘 **Sive sfere** = Istraživane stanice
- 🟡 **Žute sfere** = Čelna fronta

**GOTOVO PATH PLANNING!** ✅

---

## ⚙️ Parametri

### A* Path Planner

```bash
ros2 launch student_assignment_02 a_star_path_planner.launch.py \
    max_iterations:=50000 \
    inflation_distance_m:=0.2 \
    allow_diagonal:=true
```

| Parametar | Default | Opis |
|-----------|---------|------|
| `max_iterations` | 50000 | Max broja A* iteracija |
| `inflation_distance_m` | 0.2 | Buffer od zidova (m) |
| `allow_diagonal` | true | Dijagonalno kretanje |

### Lokalizacija

```bash
ros2 launch student_assignment_02 localization_complete_launch.py \
    map_name:=map_06 \
    initial_pose_x:=0.5 \
    initial_pose_y:=0.5
```

---

## 🛠️ Troubleshooting

### "Command not found" za čvorove

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
colcon build --packages-select student_assignment_02 --symlink-install
source install/setup.bash
```

### Putanja nije pronađena

1. Provjeri da je robot lokaliziran (crvene strelice → zelene)
2. Provjeri da je cilj u slobodnoj zoni
3. Povećaj `max_iterations`:
   ```bash
   ros2 launch student_assignment_02 a_star_path_planner.launch.py max_iterations:=100000
   ```

### RViz ne prikazuje mapu

1. Postavite Fixed Frame na `map`
2. Dodajte `/map` kao OccupancyGrid
3. Provjeri `/map` topic:
   ```bash
   ros2 topic echo /map --once | head -20
   ```

### base_link transformacija nedostaje

```bash
# Provjeri TF
ros2 run tf2_tools view_frames

# Ako base_link nedostaje, robot nije lokaliziran
# Pokrenite robota s teleop dok se ne lokalizira
```

---

## 📊 Monitoring

```bash
# Svi čvorovi
ros2 node list

# Svi topic-i
ros2 topic list

# TF transformacije
ros2 run tf2_ros tf2_echo map base_link

# Ispis putanje
ros2 topic echo /planned_path
```

---

## 📁 Direktorij Struktura

```
student-assignment-02/
├── src/student_assignment_02/
│   ├── student_assignment_02/
│   │   ├── a_star_path_planner.py
│   │   ├── map_republisher.py
│   │   └── ...
│   ├── launch/
│   │   ├── mapping_complete_launch.py
│   │   ├── localization_complete_launch.py
│   │   └── a_star_path_planner.launch.py
│   ├── mapped_maps/
│   │   ├── map_01/
│   │   ├── map_06/        ← NOVA MAPA
│   │   └── ...
│   ├── config/
│   └── setup.py
├── README.md              ← OVI FAJL
└── ...
```

---

## 🚀 Brzi Start (Ako Ste Spremi)

Ako ste već mapirali:

```bash
# Terminal 1
ros2 launch stage_ros2 stage.launch.py

# Terminal 2 (zamijeni map_06!)
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_06

# Terminal 3
ros2 launch student_assignment_02 a_star_path_planner.launch.py

# Terminal 4
ros2 run rviz2 rviz2

# U RViz: 2D Goal Pose → kliknite na mapu
```

---

## 📚 Dodatne Upute

### Spremi Novu Mapu

```bash
# Provjeri zadnji broj
ls ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/mapped_maps/

# Spremi (zamijeni USERNAME i broj)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name:
    data: '/home/USERNAME/FSB/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/mapped_maps/map_07/map_07'"
```

### Koristi Drugu Mapu

```bash
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_05
```

---

## 🎯 Ključne Mogućnosti

✅ **SLAM Mapiranje** - Automatsko mapiranje okruženja  
✅ **AMCL Lokalizacija** - Precizna lokalizacija robota  
✅ **A* Path Planning** - Optimalna putanja  
✅ **Inflation Buffer** - 0.2m sigurna distanca od zidova  
✅ **Base Link Tracking** - Automatsko čitanje robot pozicije  
✅ **RViz Vizualizacija** - Potpuna vizualizacija pretraživanja  
✅ **Dynamic Goal Pose** - Postavljanje cilja iz RViza  

---

## 📖 Literatura

- [ROS 2 Dokumentacija](https://docs.ros.org/en/humble/)
- [SLAM Toolbox](https://github.com/StanleyInnovation/slam_toolbox)
- [Navigation2](https://navigation.ros.org/)
- [A* Algoritam](https://en.wikipedia.org/wiki/A*_search_algorithm)

---

## 👨‍💻 Autor

**Kresimir Hartl** (KxHartl)  
FSB, Zagreb  
Sjecanj 2026.

---

**Status**: ✅ GOTOVO  
**Verzija**: 1.0.0
