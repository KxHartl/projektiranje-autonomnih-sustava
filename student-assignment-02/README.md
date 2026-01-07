# 🤖 STUDENT ASSIGNMENT 02 - A* Path Planning

**Mapiranje (SLAM) → Lokalizacija (AMCL) → Path Planning (A*)**

Kompletna implementacija autonomous robotike. Ovaj README vodi vas korak po korak od početka do kraja.

> ⚠️ **VAŽNO**: Pažljivo slijedite sve korake redom. Ako nešto ne radi, pogledajte [Troubleshooting](#troubleshooting).

---

## 📋 Sadržaj

1. [Preduvjeti](#preduvjeti)
2. [Instalacija](#instalacija--prvi-put-)
3. [Korak 1: Mapiranje](#korak-1-mapiranje)
4. [Korak 2: Lokalizacija](#korak-2-lokalizacija)
5. [Korak 3: A* Path Planning](#korak-3-a-path-planning)
6. [Parametri](#parametri)
7. [Troubleshooting](#troubleshooting)

---

## 📦 Preduvjeti

Provjerite da imate:

```bash
ros2 --version
```

Trebalo bi vidjeti: `ROS 2 Humble ...`

Ako nemate ROS 2 Humble instaliran, [pratite službenu instalaciju](https://docs.ros.org/en/humble/Installation.html).

---

## 🔧 Instalacija (PRVI PUT)

### Korak 1: Clone Repository

```bash
# Kreiraj direktorij gdje će biti projekt
mkdir -p ~/zadaca_02_ws
cd ~/zadaca_02_ws

# Clone repository
git clone https://github.com/KxHartl/projektiranje-autonomnih-sustava.git

# Idi u folder zadaće
cd projektiranje-autonomnih-sustava/student-assignment-02
```

**Trebalo bi biti**:
```
~/zadaca_02_ws/
└── projektiranje-autonomnih-sustava/
    └── student-assignment-02/
        ├── src/
        ├── launch/
        ├── README.md
        └── ...
```

### Korak 2: Build Projekt

```bash
# Sigurno si u student-assignment-02 direktoriju
pwd
# Trebalo bi: .../student-assignment-02

# Očisti stare build datoteke
rm -rf build/ install/ log/

# Gradi projekt
colcon build --symlink-install
```

### Korak 3: Source Setup

```bash
# Učitaj environment
source install/setup.bash

**✅ INSTALACIJA GOTOVA!**

---

## 🗺️ KORAK 1: Mapiranje

**Cilj**: Mapirati okruženje pomo­ću SLAM Toolbox-a

### 1.2 Terminal 2: SLAM Mapping

Otvori novi terminal:

```bash
cd ~/zadaca_02_ws/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 launch student_assignment_02 mapping_complete_launch.py
```

**Trebali biste vidjeti**: RViz prozor s mapom kako se gradi.

### 1.2 Terminal 2: Upravljanje Robotom

Otvori novi terminal:

```bash
cd ~/zadaca_02_ws/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 run turtlebot3_teleop teleop_keyboard
```

**Trebali biste vidjeti**: Poruka "Publishing twist"

**Cilj**: Voziš robota po čitavom okruženju da SLAM mapira sve.

**Savjet**: Vozite u U-obliku, pokrivajući sve zidove i kutove.

### 1.3 Terminal 3: Spremi Mapu

Otvori novi terminal:

```bash
cd ~/zadaca_02_ws/projektiranje-autonomnih-sustava/student-assignment-02

# Provjeri koji je broj zadnje mape
ls src/student_assignment_02/mapped_maps/
```

Vidjećete nešto kao:
```
map_01  map_02  map_03  map_04
```

Ako je zadnja `map_04`, spremi kao `map_05`.

**Sada spremi mapu** (zamijeni USERNAME sa tvojim korisničkim imenom - provjeri s `whoami`):

```bash
# Prvo provjeri svoje korisničko ime
whoami
```

Ako se javlja `hartl`, onda komanda je:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name:
    data: '/home/hartl/zadaca_02_ws/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/mapped_maps/map_05/map_05'"
```

> ⚠️ **VAŽNO**: Zamijeni `hartl` sa rezultatom `whoami` komande!

**Trebali biste vidjeti**: Poruka "Map saved successfully"

### 1.4 Provjera Sprema Mape

U istom terminalu:

```bash
ls -la src/student_assignment_02/mapped_maps/map_05/
```

Trebalo bi biti:
```
map_05.pgm   (slika mape)
map_05.yaml  (metapodaci)
```

**Ako vidite ove datoteke: ✅ MAPIRANJE GOTOVO!**

### 1.5 Zaustavi Simulatore

U sve terminale, pritisnite **CTRL+C** da zaustaviš sve.

---

## 📍 KORAK 2: Lokalizacija

**Cilj**: Lokalizirati robota s AMCL koristeći spremljenu mapu

**Što trebate**: 3-4 otvorena terminala

### 2.1 Terminal 1: Stage Simulator

```bash
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 launch stage_ros2 stage.launch.py
```

### 2.2 Terminal 2: Lokalizacija

Otvori novi terminal:

```bash
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

# Zamijeni map_05 s brojem vaše mape ako je drugačiji!
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_05
```

**Trebali biste vidjeti**: RViz s mapom i crvenim strelicama (particle filter).

### 2.3 Terminal 3: Upravljanje Robotom

Otvori novi terminal:

```bash
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 run turtlebot3_teleop teleop_keyboard
```

**VA­ŽNO**: Pokrenite robota s **W** tipkom nekoliko puta!

**Trebalo bi se desiti**: Crvene strelice se počinju okretati u zelene. To znači da se robot lokalizira!

Vozite robota malo više dok se sve strelice ne oklone zelenim.

### 2.4 Provjera TF Transformacija

Otvori novi terminal:

```bash
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 run tf2_tools view_frames
```

Provjerite je li mapa → odom → base_link dostupna:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Trebalo bi vidjeti koordinate bez greške.

**✅ LOKALIZACIJA GOTOVA!**

### 2.5 Zaustavi Simulatore

U svim terminalima, pritisnite **CTRL+C**.

---

## 🎯 KORAK 3: A* Path Planning

**Cilj**: Planirati putanju s A* algoritmom

**Što trebate**: 5 otvorenih terminala

### 3.1 Terminal 1: Stage Simulator

```bash
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 launch stage_ros2 stage.launch.py
```

### 3.2 Terminal 2: Lokalizacija

Otvori novi terminal:

```bash
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

# Zamijeni map_05 s brojem vaše mape!
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_05
```

### 3.3 Terminal 3: A* Path Planner

Otvori novi terminal:

```bash
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 launch student_assignment_02 a_star_path_planner.launch.py
```

**Trebali biste vidjeti**:
```
[INFO] A* Path Planner Node: Started
[INFO] Inflation distance: 0.2m
[INFO] Koristi base_link za početnu točku
```

### 3.4 Terminal 4: RViz

Otvori novi terminal:

```bash
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 run rviz2 rviz2
```

**RViz prozor će se otvoriti.**

### 3.5 RViz Konfiguracija

**U RViz-u**:

1. **Gornji lijevo**, kliknite na **Fixed Frame** dropdown
2. Izaberite **map**

3. Kliknite **Add** (gumb dolje lijevo)
4. Kliknite **By topic**
5. Dodaj ove topic-e:
   - `/map` → OccupancyGrid ✅
   - `/planned_path` → Path ✅
   - `/path_planning_visualization` → Marker ✅
   - `/inflation_buffer_visualization` → Marker ✅

**Trebali biste vidjeti mapu i inflation buffer (narančasti kubici).**

### 3.6 Postavljanje Cilja

**U RViz-u**:

1. U gornjoj toolbaru, kliknite na **2D Goal Pose** (zeleni strelica s crvendim ciljnom točkom)
2. Kliknite na mapu gdje trebate cilj
3. Povucite miš malo da postavite smjer
4. Otpustite miš

**Trebalo bi se desiti**: 
- 🟢 **Zelena linija** se pojavljuje (to je vaša putanja!)
- 🟠 **Narančasti kubici** pokazuju buffer (0.2m od zidova)
- 🔴 **Sive sfere** pokazuju istraživane stanice
- 🟡 **Žute sfere** pokazuju čelnu frontu algoritma

**Trebali biste vidjeti poruku u terminalu 3**:
```
[INFO] Path found! Length: XX nodes
[INFO] Path planning took X.XX seconds
```

**✅ A* PATH PLANNING GOTOVO!**

### 3.7 Pokušajte Više Cilja

Možete kliknuti na različite lokacije na mapi i svaki put će se putanja replanirati!

---

## ⚙️ Parametri

### Promjena Mape

Ako trebate koristiti drugačiju mapu (npr. map_04):

```bash
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_04
```

### A* Path Planner - Custom Parametri

```bash
# Primjer: Manji buffer (0.15m umjesto 0.2m)
ros2 launch student_assignment_02 a_star_path_planner.launch.py inflation_distance_m:=0.15

# Primjer: Više iteracija
ros2 launch student_assignment_02 a_star_path_planner.launch.py max_iterations:=100000

# Primjer: Bez dijagonalnog kretanja
ros2 launch student_assignment_02 a_star_path_planner.launch.py allow_diagonal:=false
```

---

## 🔍 Troubleshooting

### Problem: "Command not found" za `ros2 launch`

**Rješenje**: Trebate source-ati setup:

```bash
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash
```

### Problem: "CMake Error" tijekom build-a

**Rješenje**: Očisti i pokušaj ponovno:

```bash
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build/ install/ log/
colcon build --packages-select student_assignment_02 --symlink-install
```

### Problem: Mapa se ne sprema

**Trebalo bi**: Vidjeti poruku "Map saved successfully"

**Ako ne vidite**: Provjerite je li SLAM Toolbox pokrenut:

```bash
ros2 topic list | grep slam
```

Trebalo bi vidjeti `/slam_toolbox/...` topic-e.

### Problem: Lokalizacija se ne javlja

**Trebalo bi**: Crvene strelice postanu zelene nakon što vozite robota.

**Ako ne postaju**: Vozite robota više (W tipka) duže vremenske.

### Problem: Putanja nije pronađena

**Trebalo bi**: Trebali biste vidjeti zelenu liniju u RViz-u.

**Ako ne vidite**:
1. Provjerite da je start pozicija validna (robot u slobodnoj zoni)
2. Provjerite da je cilj validan (kliknite u slobodnu zonu)
3. Pove­čajte `max_iterations`:
   ```bash
   ros2 launch student_assignment_02 a_star_path_planner.launch.py max_iterations:=100000
   ```

### Problem: RViz ne prikazuje mapu

**Trebalo bi**: Vidjeti mapu kao grid.

**Ako ne vidite**:
1. Provjerite da je Fixed Frame postavljen na `map`
2. Dodajte `/map` kao OccupancyGrid
3. Provjerite je li `/map` topic dostupan:
   ```bash
   ros2 topic echo /map --once | head -20
   ```

### Problem: Transformacija `base_link` ne postoji

**Trebalo bi**: `ros2 run tf2_ros tf2_echo map base_link` pokazuje koordinate.

**Ako se javlja greška**:
1. Provjerite je li lokalizacija pokrenut
2. Vozite robota više
3. Provjerite TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Problem: Trebam stari kod/mapu

**Stari kod**: GitHub history:
```bash
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava
git log --oneline
```

**Stara mapa**: U `src/student_assignment_02/mapped_maps/` direktoriju su sve mape.

---

## 📊 Monitoring & Debug

### Provjera Svih Aktivnih Čvorova

```bash
ros2 node list
```

Trebalo bi vidjeti:
```
/a_star_path_planner
/amcl
/stage_ros2
/rviz2
...
```

### Provjera Svih Topic-a

```bash
ros2 topic list
```

### Ispis Putanje (Live)

```bash
ros2 topic echo /planned_path
```

### Ispis Mape (Live)

```bash
ros2 topic echo /map --once
```

---

## 📁 Direktorij Struktura

```
~/autonomni-robotika/
└── projektiranje-autonomnih-sustava/
    └── student-assignment-02/
        ├── src/student_assignment_02/
        │   ├── student_assignment_02/
        │   │   ├── a_star_path_planner.py     ← Glavni kod
        │   │   ├── map_republisher.py
        │   │   └── ...
        │   ├── launch/
        │   │   ├── mapping_complete_launch.py       ← Mapiranje
        │   │   ├── localization_complete_launch.py  ← Lokalizacija
        │   │   └── a_star_path_planner.launch.py    ← A* Planer
        │   ├── mapped_maps/
        │   │   ├── map_01/
        │   │   ├── map_05/        ← NOVA MAPA
        │   │   └── ...
        │   ├── config/
        │   └── setup.py
        ├── README.md              ← OVI FAJL
        └── ...
```

---

## 🚀 Brzi Start (Nakon Instalacije)

Ako ste završili instalaciju i mapiranje:

```bash
# Terminal 1
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash
ros2 launch stage_ros2 stage.launch.py

# Terminal 2
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash
ros2 launch student_assignment_02 localization_complete_launch.py map_name:=map_05

# Terminal 3
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash
ros2 launch student_assignment_02 a_star_path_planner.launch.py

# Terminal 4
cd ~/autonomni-robotika/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash
ros2 run rviz2 rviz2

# U RViz: Postavite cilj s 2D Goal Pose
```

---

## 📚 Dodatne Informacije

### ROS 2 Dokumentacija
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

### Algoritmi
- [A* Search](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
- [AMCL Lokalizacija](https://wiki.ros.org/amcl)

### Simulatori
- [Stage ROS2](https://github.com/ros-simulation/stage_ros2)

---

## 👨‍💻 Autor

**Kresimir Hartl** (KxHartl)  
Fakultet Strojarstva i Brodogradnje, Zagreb  
Sječanj 2026.

---

## 📞 Podrška

Ako trebate pomoć:

1. Provjerite [Troubleshooting](#troubleshooting) sekciju
2. Provjerite [ROS 2 dokumentaciju](https://docs.ros.org/en/humble/)
3. Otvorite GitHub issue: [Issues](https://github.com/KxHartl/projektiranje-autonomnih-sustava/issues)

---

**Status**: ✅ GOTOVO  
**Verzija**: 1.1.0  
**Datum**: 7. siječnja 2026.  
**Zadnja Izmjena**: 7. siječnja 2026.
