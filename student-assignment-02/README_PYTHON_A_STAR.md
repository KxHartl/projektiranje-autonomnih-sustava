# 🤖 ROS 2 Autonomous Path Planning - A* Algorithm

ROS 2 Humble Python paket za autonomnu navigaciju robota s A* path planning algoritmom i SLAM mapiranjem.

---

## 📋 Sadržaj

1. [Instalacija](#-instalacija)
2. [Build](#-build)
3. [Korak 1: Mapiranje](#-korak-1-mapiranje)
4. [Korak 2: Path Planning & Navigacija](#-korak-2-path-planning--navigacija)
5. [Struktura Projekta](#-struktura-projekta)
6. [Troubleshooting](#-troubleshooting)

---

## 💻 Instalacija

### Preduvjeti

```bash
# ROS 2 Humble instaliran
# Ubuntu 22.04 ili starija
# Python 3.10+
# git
```

### Preuzimanje Projekta

```bash
cd ~
git clone https://github.com/KxHartl/projektiranje-autonomnih-sustava.git
cd projektiranje-autonomnih-sustava/student-assignment-02
```

### Instalacija Dependency-ja

```bash
# Aktiviraj ROS 2 Humble
source /opt/ros/humble/setup.bash

# Instalacija potrebnih paketa
sudo apt-get update
sudo apt-get install -y \
  ros-humble-stage-ros2 \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard \
  python3-numpy
```

---

## 🔨 Build

```bash
cd ~/projektiranje-autonomnih-sustava/student-assignment-02

# Očisti prethodne build-ove
rm -rf build install log

# Build paket
colcon build

# Učitaj install setup
source install/setup.bash
```

**Očekivani Output:**
```
Starting >>> student_assignment_02
Finished <<< student_assignment_02 [6.23s]

Summary: 1 package finished [6.23s]
```

---

# 🗺️ KORAK 1: MAPIRANJE

Mapiramo okruženje sa SLAM toolbox-om.

## 1.1 Pokreni RViz (Prvi Terminal)

```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/projektiranje-autonomnih-sustava/student-assignment-02/config/rviz_config.rviz
```

**RViz Prozor:**
- Prikazuje mapu kako se gradi
- Prikazuje poziciju robota
- Laser scan vizualizacija

---

## 1.2 Pokreni Stage + SLAM (Drugi Terminal)

```bash
cd ~/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash
ros2 launch student_assignment_02 mapping_launch.py
```

**Stage Simulator:**
- Prikazuje robota u okruženju
- Prikazuje laser sensore
- Pokreće SLAM toolbox

**Što se dešava:**
- Stage simulira robota i laser senzore
- SLAM toolbox mapira okruženje
- RViz ažurira mapu u realnom vremenu

---

## 1.3 Navigacija Robota - Teleop (Treći Terminal)

```bash
source /opt/ros/humble/setup.bash
cd ~/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel
```

**Upravljanje:**
```
Reading from keyboard
Publishing to /cmd_vel

Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

c/v : increase/decrease only linear
,/. : increase/decrease only angular

q to quit
```

**Što trebate učiniti:**
1. Pritisnite `w` da se robot pomakne naprijed
2. Pritisnite `a`/`d` za rotaciju
3. Navigirajte robot kroz cijelo okruženje
4. **Važno:** Robot mora posjetiti sve dijelove mape!
5. Čini se li mapa dobra? Nastavite dalje (Korak 1.4)

---

## 1.4 Spremi Mapu (Četvrti Terminal)

Kada je mapiranje dovršeno:

```bash
source /opt/ros/humble/setup.bash

# Kreiraj direktorij za mapu
mkdir -p ~/my_map

# Spremi mapu
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "map_url:$HOME/my_map"
```

**Output:**
```
waiting for service to become available...
service not available, waiting again...
service call failed to wait for service
```

**Alternativa - Koristi map_saver iz save_map noda:**

```bash
cd ~/my_map
ros2 run nav2_map_server map_saver_cli --fmt pgm -f map
```

**Provjera Mape:**
```bash
ls -la ~/my_map/
```

**Trebate vidjeti:**
```
map.pgm    (bitmap mape)
map.yaml   (metapodaci)
map.txt    (dodatni podaci)
```

**Zatvori sve Terminal prozore** (Ctrl+C)

---

# 🎯 KORAK 2: PATH PLANNING & NAVIGACIJA

Koristimo spravljenu mapu za A* path planning i navigaciju.

## 2.1 Pokreni RViz (Prvi Terminal)

```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/projektiranje-autonomnih-sustava/student-assignment-02/config/rviz_config.rviz
```

**RViz Prozor:**
- Prikazuje spravljenu mapu
- Prikazuje robota
- Prikazuje planiranu putanju (zelena linija)
- Prikazuje početak (zelena sfera) i cilj (crvena sfera)

---

## 2.2 Pokreni Stage + Path Planning + Navigation (Drugi Terminal)

```bash
cd ~/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

# Default mapa ~/my_map
ros2 launch student_assignment_02 path_planning_launch.py

# Ili sa custom mapom:
ros2 launch student_assignment_02 path_planning_launch.py map_file:=~/my_map/map
```

**Što se dešava:**
- Stage simulira robota
- Map Server učitava spravljenu mapu (`~/my_map/map.yaml`)
- Path Planning Node čeka na /goal_pose
- Goal Navigation Node je spreman za slijeđenje putanje

**Expected Log Output:**
```
[map_server-1]: Info: Read map from [/home/user/my_map/map.pgm]...
[path_planning_node-X]: Path Planning Node initialized
[goal_navigation_node-X]: Goal Navigation Node initialized
```

---

## 2.3 Postavite Cilj u RViz (RViz Prozor)

### Odaberite "2D Goal Pose" Tool

1. U RViz Top Toolbar pronađite **"2D Goal Pose"** tool (crvena strelica)
2. Kliknite na ikonu

### Postavite Cilj u Mapi

1. **Kliknite** na poziciju u mapi gdje želite da robot ide
2. **Povucite** da postavite smjer (strelica)
3. **Pustite** mišu

**Što se dešava:**
- A* algoritam počinje planirati putanju
- Vidite log u Terminal prozoru:
  ```
  [path_planning_node-X]: === [Plan 1] ===
  [path_planning_node-X]: ✓ Path found! Length: 47 nodes
  ```
- Vidite **zelenu liniju** s putanjom
- Vidite **zelenu sferu** (početak) i **crvenu sferu** (cilj)
- Robot počinje slijediti putanju!

---

## 2.4 Praćenje Navigacije

### U RViz:
- Vidite robota kako se giba po putanji
- Vidite laser scan (crveni zrake)
- Vidite orijentaciju robota (strelica)

### U Terminal:
```
[goal_navigation_node-X]: ✅ Waypoint 10 reached (0.05 m)
[goal_navigation_node-X]: ✅ Waypoint 20 reached (0.08 m)
[goal_navigation_node-X]: ✅ Goal reached! Movement finished.
```

### Ponovite Test

1. Postavite novi cilj s "2D Goal Pose" tool
2. Vidite kako se planira nova putanja
3. Robot ide do novog cilja

---

## 📊 Testiranje Parametara

### Promijenite brzinu robota:

```bash
ros2 launch student_assignment_02 path_planning_launch.py \
  linear_speed:=0.3 \
  angular_speed:=0.7 \
  waypoint_tolerance:=0.2
```

### Dostupni Parametri:
- **linear_speed**: Brzina naprijed (default: 0.2 m/s)
- **angular_speed**: Brzina rotacije (default: 0.5 rad/s)
- **waypoint_tolerance**: Tolerancija distancije (default: 0.15 m)
- **angle_tolerance**: Tolerancija kuta (default: 0.2 rad)

---

# 📁 Struktura Projekta

```
student-assignment-02/
├── src/student_assignment_02/
│   ├── student_assignment_02/          # Python paket
│   │   ├── __init__.py
│   │   ├── path_planning_node.py       # A* putanja planiranje
│   │   ├── goal_navigation_node.py     # Slijeđenje putanje
│   │   ├── a_star.py                   # A* algoritam
│   │   └── utils.py                    # Utility funkcije
│   ├── launch/
│   │   ├── stage_launch.py             # Samo Stage simulator
│   │   ├── mapping_launch.py           # Stage + SLAM (mapiranje)
│   │   ├── path_planning_launch.py     # Stage + Path Planning
│   │   └── online_async_launch.py      # SLAM toolbox
│   ├── config/
│   │   └── rviz_config.rviz            # RViz konfiguracija
│   ├── world/
│   │   └── map_01.world                # Stage simulator svijet
│   ├── setup.py                        # Python build config
│   ├── pyproject.toml                  # Modern Python config
│   └── package.xml                     # ROS 2 metadata
├── data/
│   └── maps/                           # Spravljene mape
├── README.md                           # Originalni README
└── README_PYTHON_A_STAR.md             # Ovaj README
```

---

# 🐛 Troubleshooting

## Problem: "Package not found"

```
[ERROR] Package 'student_assignment_02' not found
```

**Rješenje:**
```bash
cd ~/projektiranje-autonomnih-sustava/student-assignment-02
colcon build
source install/setup.bash
```

---

## Problem: "Map file not found"

```
[ERROR] Failed to load map from [/home/user/my_map/map.yaml]
```

**Rješenje:**
1. Provjerite da postoji mapa:
   ```bash
   ls -la ~/my_map/
   ```
2. Ako ne postoji, ponovite Korak 1 (mapiranje)
3. Koristite puni path:
   ```bash
   ros2 launch student_assignment_02 path_planning_launch.py map_file:=$HOME/my_map/map
   ```

---

## Problem: "Map server not available"

```
[ERROR] Service '/map_saver/save_map' not available
```

**Rješenje:**
1. Provjerite da je SLAM toolbox pokrenut
2. Koristite alternativu:
   ```bash
   ros2 run nav2_map_server map_saver_cli --fmt pgm -f ~/my_map/map
   ```

---

## Problem: "Robot ne ide prema cilju"

**Mogućnosti:**
1. Putanja nije pronađena (cilj je u prepreci)
2. RViz nije izbrisao stare markere

**Rješenje:**
1. Postavite cilj drugdje
2. Restartaj sve prozore

---

## Problem: "RViz config ne učitava se"

```
source /opt/ros/humble/setup.bash
rviz2 -d ~/projektiranje-autonomnih-sustava/student-assignment-02/config/rviz_config.rviz
```

Ako ne radi, kreirajte ručno:
1. Otvorite RViz bez config-a:
   ```bash
   rviz2
   ```
2. Dodajte displays (left panel → Add):
   - Grid
   - Map
   - LaserScan
   - Path
   - MarkerArray
   - TF
3. Spremi kao `rviz_config.rviz`

---

## Problem: "Stage se ne pokreće"

```
[ERROR] Could not load stage_ros2
```

**Rješenje:**
```bash
sudo apt-get install ros-humble-stage-ros2
```

---

# 📝 ROS 2 Naredbe

### Debug naredbe

```bash
# Popis svih topic-a
ros2 topic list

# Popis svih node-a
ros2 node list

# Popis svih service-a
ros2 service list

# Provjera topica
ros2 topic echo /planned_path
ros2 topic echo /cmd_vel

# TF tree
ros2 run tf2_tools view_frames.py
```

### Spremi bag datoteku (recording)

```bash
ros2 bag record -o my_recording /map /cmd_vel /planned_path
```

### Reproduciraj bag datoteku

```bash
ros2 bag play my_recording
```

---

# 🔗 Linkovi

- **A* Algoritam**: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py
- **ROS 2 Humble**: https://docs.ros.org/en/humble/
- **Stage Simulator**: http://playerstage.sourceforge.net/
- **SLAM Toolbox**: https://github.com/SteveMacenski/slam_toolbox

---

# ✅ Checklist - Prije nego što krenete

- [ ] ROS 2 Humble instaliran: `source /opt/ros/humble/setup.bash`
- [ ] Dependency-ji instalirani: `sudo apt-get install ros-humble-stage-ros2 ...`
- [ ] Project kloniran: `git clone ...`
- [ ] Build gotov: `colcon build && source install/setup.bash`
- [ ] Mapiranje dovršeno: `~/my_map/map.yaml` postoji
- [ ] RViz config pronađen: `~/student-assignment-02/config/rviz_config.rviz`

---

# 📞 Zahtjev za Pomoć

Ako nešto ne radi:

1. Provjerite log poruke: Terminal prozor gdje ste pokrenuli launch
2. Provjerite ROS 2 je aktivan: `echo $ROS_DISTRO` (trebalo bi biti "humble")
3. Provjerite path-ove: `echo $PATH | grep ros`
4. Pokrenite sve ponovno od početka

---

**Autori:** AI Assistant

**Datum:** 7. Siječnja 2026.

**Verzija:** 1.0

**Status:** ✅ Production Ready
