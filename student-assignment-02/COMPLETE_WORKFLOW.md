# 🚀 STUDENT ASSIGNMENT 02 - Kompletni Workflow

**Mapiranje → Lokalizacija → A* Path Planning**

Ovaj README vodit će vas kroz sve korake od instalacije do pokretanja A* path planera.

---

## 📋 Sadržaj

1. [Preduvjeti](#preduvjeti)
2. [Instalacija](#instalacija)
3. [Korak 1: Mapiranje](#korak-1-mapiranje)
4. [Korak 2: Lokalizacija](#korak-2-lokalizacija)
5. [Korak 3: A* Path Planning](#korak-3-a-path-planning)
6. [Troubleshooting](#troubleshooting)

---

## 📦 Preduvjeti

Trebate:
- **ROS 2 Humble** (instaliran)
- **Stage ROS2** (`ros2 launch stage_ros2 stage.launch.py` radi)
- **SLAM Toolbox** (`slam_toolbox` paket)
- **Nav2** (za AMCL lokalizaciju)
- **Python 3.10+**

### Provjera Instalacije

```bash
# Provjeri ROS 2
ros2 --version
# Trebalo bi: ROS 2 Humble ...

# Provjeri Stage
ros2 pkg list | grep stage
# Trebalo bi: stage_ros2

# Provjeri SLAM Toolbox
ros2 pkg list | grep slam
# Trebalo bi: slam_toolbox
```

---

## 💻 Instalacija

### 1. Clone Repository

```bash
cd ~/FSB/projektiranje-autonomnih-sustava
git clone https://github.com/KxHartl/projektiranje-autonomnih-sustava.git
# Ili ako već postoji:
cd student-assignment-02
```

### 2. Build Paket

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02

# Clean build (važno!)
rm -rf build/ install/ log/

# Build paket
colcon build --packages-select student_assignment_02 --symlink-install

# Source setup
source install/setup.bash
```

### 3. Provjera Čvorova

```bash
# Trebali biste vidjeti svi 4 čvora
ros2 run student_assignment_02 a_star_path_planner --help
ros2 run student_assignment_02 map_republisher --help
```

Ako se javlja greška "Command not found" - vratite se na korak 2!

---

## 🗺️ KORAK 1: Mapiranje

### 1.1 Pokrenite Stage Simulator

```bash
# Terminal 1
ros2 launch stage_ros2 stage.launch.py
```

Trebali biste vidjeti:
- Simulator s robotom
- Mapa s prepreke
- Robot u inicijalni poziciji

### 1.2 Pokrenite Mapping

```bash
# Terminal 2 (iz student-assignment-02 direktorija)
ros2 launch student_assignment_02 mapping_complete_launch.py
```

Trebali biste vidjeti:
- RViz s mapom
- SLAM Toolbox koji gradi mapu
- Robot pozicija u RViz-u

### 1.3 Upravljanje Robotom

```bash
# Terminal 3
ros2 run turtlebot3_teleop teleop_keyboard
```

**Upravljanje:**
- `w` = Naprijed
- `s` = Unazad
- `a` = Lijevo
- `d` = Desno
- `x` = Zaustavi
- Shift + wasd = Brži pokret

**Cilj**: Pripremite sveobuhvatnu mapu cijelog okruženja

### 1.4 Spremi Mapu

```bash
# Terminal 4
# Prvo provjeri koji je zadnji broj mape
ls -la ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/mapped_maps/

# Npr. ako zadnja mapa je map_05, spremi kao map_06
# Spremi mapu
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name:
    data: '/home/USERNAME/FSB/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/mapped_maps/map_06/map_06'"
```

**Važno**: Zamijeni `USERNAME` sa tvojim korisničkim imenom!

### 1.5 Provjera Spremljene Mape

```bash
ls -la ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/mapped_maps/map_06/

# Trebalo bi biti:
# - map_06.pgm (slika mape)
# - map_06.yaml (metapodaci)
```

**GOTOVO MAPIRANJE!** ✅

---

## 🧭 KORAK 2: Lokalizacija

### 2.1 Pokrenite Stage (ako nije pokrenut)

```bash
# Terminal 1
ros2 launch stage_ros2 stage.launch.py
```

### 2.2 Pokrenite Lokalizaciju S Mapom

```bash
# Terminal 2
# Zamijeni map_06 s brojem vaše mape!
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_06
```

Trebali biste vidjeti:
- RViz s mapom koju ste mapirali
- Robot pozicija
- Particle filter vizualizacija (crvene strelice)
- "/tf" informacije

### 2.3 Inicijalizacija Pozicije

```bash
# Terminal 3 - Upravljanje robotom
ros2 run turtlebot3_teleop teleop_keyboard
```

**Važno**: Robot se mora "lokalizirati":
1. Pokrenite robota `w` tipkom
2. Promatrajte RViz - crvene strelice trebaju postati zelene ✅
3. To znači da je robot lokaliziran!

### 2.4 Provjera TF Tree-a

```bash
# Terminal 4 - Provjeri da li je transformacija dostupna
ros2 run tf2_tools view_frames
```

Trebalo bi vidjeti:
```
map
 └─ odom
    └─ base_link
```

**GOTOVO LOKALIZACIJA!** ✅

---

## 🤖 KORAK 3: A* Path Planning

### 3.1 Pokrenite Stage + Lokalizaciju

```bash
# Terminal 1 - Stage
ros2 launch stage_ros2 stage.launch.py

# Terminal 2 - Lokalizacija (s vašom mapom!)
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_06

# Terminal 3 - Upravljanje (ako trebate)
ros2 run turtlebot3_teleop teleop_keyboard
```

### 3.2 Pokrenite A* Path Planner

```bash
# Terminal 4
ros2 launch student_assignment_02 a_star_path_planner.launch.py
```

Trebali biste vidjeti:
```
[INFO] [a_star_path_planner]: A* Path Planner Node: Started
[INFO] [a_star_path_planner]: Max iterations: 50000
[INFO] [a_star_path_planner]: Inflation distance: 0.2m (0.2m = 20cm)
[INFO] [a_star_path_planner]: Koristi base_link za početnu točku
```

### 3.3 Pokrenite RViz

```bash
# Terminal 5
ros2 run rviz2 rviz2
```

### 3.4 Konfiguracija RViz-a

**Fixed Frame**: `map`

**Add > By display type:**
1. **Map** → `/map`
2. **Path** → `/planned_path`
3. **Marker** → `/path_planning_visualization` (istraživane stanice - sive)
4. **Marker** → `/planning_frontier` (čelna fronta - žute)
5. **Marker** → `/inflation_buffer_visualization` (buffer - narančasti kubici)

### 3.5 Postavljanje Goal-a

1. Kliknite "2D Goal Pose" tool (zeleni strelica u toolbaru)
2. Kliknite na mapu gdje želite da robot ide
3. **Putanja se planira automatski!**

Trebali biste vidjeti:
- 🟢 **Zelena linija** = Planirana putanja
- 🟠 **Narančasti kubici** = Inflation buffer (0.2m od zidova)
- 🟤 **Sive sfere** = Istraživane stanice
- 🟡 **Žute sfere** = Čelna fronta

**GOTOVO A* PATH PLANNING!** ✅

---

## 📁 Direktorij Struktura Mapa

```
~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/
src/student_assignment_02/
mapped_maps/
├── map_01/
│   ├── map_01.pgm
│   └── map_01.yaml
├── map_02/
│   ├── map_02.pgm
│   └── map_02.yaml
├── map_06/    ← NOVA MAPA
│   ├── map_06.pgm
│   └── map_06.yaml
└── ...
```

---

## 🚀 Brzi Start (Ako Ste Spremni)

Ako ste već mapirali i spremili mapu:

```bash
# Terminal 1: Stage
ros2 launch stage_ros2 stage.launch.py

# Terminal 2: Lokalizacija (zamijeni map_06 s vašom mapom)
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_06

# Terminal 3: A* Path Planner
ros2 launch student_assignment_02 a_star_path_planner.launch.py

# Terminal 4: RViz
ros2 run rviz2 rviz2

# U RViz-u: "2D Goal Pose" → Kliknite na mapu
```

**Gotovo!** 🎉

---

## ⚙️ Parametri

### A* Path Planner Parametri

```bash
ros2 launch student_assignment_02 a_star_path_planner.launch.py \
    max_iterations:=50000 \
    inflation_distance_m:=0.2 \
    allow_diagonal:=true
```

| Parametar | Default | Opis |
|-----------|---------|------|
| `max_iterations` | 50000 | Maksimalno broja A* iteracija |
| `inflation_distance_m` | 0.2 | Buffer od zidova (metri) |
| `allow_diagonal` | true | Dijagonalno kretanje |
| `inflation_cost_threshold` | 60 | Threshold za "opasnu" zonu |

### Lokalizacija Parametri

```bash
ros2 launch student_assignment_02 localization_complete_launch.py \
    map_name:=map_06 \
    initial_pose_x:=0.5 \
    initial_pose_y:=0.5
```

---

## 🔍 Troubleshooting

### Problem: "Command not found" za čvorove

```bash
# Rješenje:
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
colcon build --packages-select student_assignment_02 --symlink-install
source install/setup.bash
```

### Problem: Mapa se ne sprema

```bash
# Provjeri da li je SLAM Toolbox pokrenut
ros2 topic list | grep slam

# Trebalo bi: /slam_toolbox/...

# Ako ne postoji, ponovno pokrenite mapping_complete_launch.py
```

### Problem: "base_link" transformacija nije dostupna

```bash
# Provjeri TF tree
ros2 run tf2_tools view_frames

# Ako base_link nedostaje, robot nije lokaliziran
# Pokrenite robota u RViz-u dok se ne lokalizira
```

### Problem: Putanja nije pronađena

1. **Provjeri da je start stanica validna**: Robot treba biti u slobodnoj zoni
2. **Provjeri da je cilj validan**: Kliknite u slobodnu zonu
3. **Povećaj `max_iterations`**:
   ```bash
   ros2 launch student_assignment_02 a_star_path_planner.launch.py max_iterations:=100000
   ```
4. **Provjerite `/map` topic**:
   ```bash
   ros2 topic echo /map --once | head -20
   ```

### Problem: RViz ne prikazuje mapu

1. **Postavite Fixed Frame na `map`**: RViz → Fixed Frame dropdown → "map"
2. **Add marker za `/map` topic**: Add → By topic → /map → OccupancyGrid
3. **Provjeri `/map` topic**:
   ```bash
   ros2 topic list | grep map
   ```

---

## 📊 Monitoring

### Monitoring Čvorova

```bash
# Provjera svih aktivnih čvorova
ros2 node list

# Trebalo bi:
# /a_star_path_planner
# /amcl
# /stage_ros2
# /rviz2
# ...
```

### Monitoring Topic-a

```bash
# Sve dostupne topic-e
ros2 topic list

# Informacije o topic-u
ros2 topic info /planned_path

# Ispis sadržaja (live)
ros2 topic echo /planned_path
```

### Monitoring TF Transformacija

```bash
# Provjera dostupnih transformacija
ros2 run tf2_tools view_frames

# Ispis specifične transformacije
ros2 run tf2_ros tf2_echo map base_link
```

---

## 📚 Dodatne Upute

### Promjena Mape

Ako trebate koristiti drugu mapu (npr. map_05):

```bash
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_05
```

### Spremi Novu Mapu

Ako ste mapirali novu mapu:

```bash
# Prvo provjeri task broj
ls -la ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/mapped_maps/

# Spremi mapu (zamijeni USERNAME i MAP_NUM)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name:
    data: '/home/USERNAME/FSB/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/mapped_maps/map_MAP_NUM/map_MAP_NUM'"
```

### Kalibracija Robotskog Kretanja

Ako robot nije precizno lokaliziran:

```bash
# Prosljeđi kalibraciju u localization_complete_launch.py
ros2 launch student_assignment_02 localization_complete_launch.py \
    map_name:=map_06 \
    initial_pose_x:=0.5 \
    initial_pose_y:=0.5 \
    initial_pose_a:=0.0
```

---

## 🎯 Sažetak Workflow-a

```
┌─────────────────────────────────────────────────┐
│ 1. MAPIRANJE (mapping_complete_launch.py)       │
│    - Robotom kruži po okruženju                  │
│    - SLAM gradi mapu                             │
│    - Spremi mapu u mapped_maps/map_XX/           │
└─────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────┐
│ 2. LOKALIZACIJA (localization_complete_launch.py)│
│    - Robot se lokalizira s AMCL                  │
│    - Particle filter poravnava s mapom           │
│    - TF tree je dostupan                         │
└─────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────┐
│ 3. PATH PLANNING (a_star_path_planner.launch.py)│
│    - A* planira putanju                          │
│    - Inflation buffer čuva distancu od zidova   │
│    - Vizualizacija u RViz-u                      │
└─────────────────────────────────────────────────┘
```

---

## 📞 Kontakt & Podrška

Ako trebate pomoć:

1. **GitHub Issues**: https://github.com/KxHartl/projektiranje-autonomnih-sustava/issues
2. **ROS 2 Dokumentacija**: https://docs.ros.org/en/humble/
3. **SLAM Toolbox**: https://github.com/StanleyInnovation/slam_toolbox
4. **Navigation2**: https://navigation.ros.org/

---

**Verzija**: 1.0.0  
**Datum**: 7. siječnja 2026.  
**Autor**: Kresimir Hartl (KxHartl)  
**Status**: ✅ GOTOVO
