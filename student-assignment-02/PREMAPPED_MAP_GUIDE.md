# 🗻️ Korištenje Unaprijed Mapiranih Mapa

## ✨ Novo

Možete sada:
- ✅ Pokrenuti Stage s **unaprijed mapiranom mapom**
- ✅ Bez čekanja na SLAM mapiranje
- ✅ Odmah početi s A* planiranjem putanja
- ✅ Isprobavati navigaciju bez mapiranja

---

## 📂 Gdje je Vaša Mapa?

Vaša mapa je sprema u:
```
~/my_map/
├── map.pgm       (slike mape)
├── map.yaml      (konfiguracija)
└── map.txt       (metapodaci)
```

---

## 🚀 BRZO POKRETANJE - 2 OPCIJE

### 🔵 OPCIJA 1: Samo Stage + Mapa (bez A* planiranja)

**Za brz pregled mape:**

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 launch student_assignment_02 navigation_with_map_launch.py
```

**Što se pokreće:**
- Stage simulator
- Robot State Publisher
- Map Server (učitava vašu mapu)

**Trebate otvoriti:**
```bash
rviz2
```

Dodajte displays: Grid, TF, LaserScan, Map

---

### 🟢 OPCIJA 2: Stage + Mapa + A* Path Planning + Navigation (KOMPLETO!)

**Za puno testiranje:**

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 launch student_assignment_02 path_planning_with_map_launch.py
```

**Što se pokreće automatski:**
- Stage simulator
- Robot State Publisher
- Map Server (vaša mapa)
- Path Planning Node (A* algoritam)
- Goal Navigation Node (gibanje robota)

**Trebate otvoriti samo:**
```bash
rviz2
```

**Onda:**
1. Dodajte displays: Grid, TF, LaserScan, Map, Path, MarkerArray
2. Kliknite "2D Goal Pose" tool
3. Gledajte kako robot planira i ide do cilja!

---

## 🎯 Usporedba Launch File-ova

| Feature | `complete_mapping_launch.py` | `navigation_with_map_launch.py` | `path_planning_with_map_launch.py` |
|---------|---------------------------|-------------------------------|--------------------------------------|
| Stage simulator | ✅ | ✅ | ✅ |
| SLAM mapiranje | ✅ (ČEKA!) | ❌ | ❌ |
| Koristi unaprijed mapiranu mapu | ❌ | ✅ | ✅ |
| Path Planning Node | ❌ | ❌ | ✅ |
| Navigation Node | ❌ | ❌ | ✅ |
| Vrijeme pokretanja | ~30 sekundi | ~3 sekunde | ~3 sekunde |
| Idealno za | Mapiranje novog svijeta | Brz pregled mape | Testiranje A* + navigacije |

---

## 🔧 Konfiguracija

### Promjena Putanje do Mape

Ako je vaša mapa drugdje:

```bash
ros2 launch student_assignment_02 path_planning_with_map_launch.py \
  map_path:=/path/to/your/map
```

**Primjer:**
```bash
ros2 launch student_assignment_02 path_planning_with_map_launch.py \
  map_path:/home/khartl/moje_mape/grad
```

### Promjena Parametara Navigacije

Ako trebate drugačije parametre:

```bash
ros2 run student_assignment_02 goal_navigation_node \
  --ros-args \
  -p linear_speed:=0.3 \
  -p angular_speed:=0.8 \
  -p waypoint_tolerance:=0.2
```

---

## 📊 Što Trebate Imati u `~/my_map/`

### Minimalno (obavezno):
```
~/my_map/
├── map.yaml    ← Konfiguracija
└── map.pgm     ← Slika mape
```

### `map.yaml` Primjer:
```yaml
image: map.pgm
resolution: 0.05      # 5 cm po grid cell
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### Provjera Mape

Ako launch file ne radi:

```bash
# Provjerite datoteke
ls -lh ~/my_map/

# Trebalo bi vidjeti:
# -rw-r--r-- 1 khartl khartl  50K Jan  5 10:00 map.pgm
# -rw-r--r-- 1 khartl khartl 150B Jan  5 10:00 map.yaml
```

---

## ❌ Česti Problemi

### Problem 1: "map.yaml not found"

**Rješenje:**
```bash
# Provjerite putanju
cat ~/my_map/map.yaml

# Trebalo bi vidjeti sadržaj yaml fajla
```

Ako ne postoji, trebate ponovno sprema mapu:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map/map
```

### Problem 2: "Map is empty or invalid"

**Provjera:**
```bash
# Provjerite je li .pgm datoteka OK
file ~/my_map/map.pgm

# Trebalo bi vidjeti:
# map.pgm: Netpbm image data, size = 200 x 200, ...
```

Ako je krivo, trebate ponovno mapirati.

### Problem 3: "Map offset is wrong"

**Uredite map.yaml:**
```yaml
origin: [0.0, 0.0, 0.0]  # Trebalo bi biti na (0,0) ili s offsetom ako trebate
```

---

## 🚀 Workflow

### Scenarij 1: Samo Pregled Mape

```bash
# Terminal 1
ros2 launch student_assignment_02 navigation_with_map_launch.py

# Terminal 2
rviz2
```

### Scenarij 2: Test A* Planiranja

```bash
# Terminal 1
ros2 launch student_assignment_02 path_planning_with_map_launch.py

# Terminal 2
rviz2

# U RViz-u: Kliknite 2D Goal Pose → Trebate vidjeti zelenu liniju putanje
```

### Scenarij 3: Test Cijele Navigacije (A* + Robot Gibanje)

```bash
# Terminal 1
ros2 launch student_assignment_02 path_planning_with_map_launch.py

# Terminal 2
rviz2

# U RViz-u:
# 1. Kliknite 2D Goal Pose
# 2. Trebate vidjeti zelenu liniju
# 3. Robot će početi ići prema cilja
```

---

## 📚 Topics Koji su Dostupni

| Topic | Tip | Opis |
|-------|-----|------|
| `/map` | OccupancyGrid | Mapa iz Map Server-a |
| `/tf` | TFMessage | Transformacije |
| `/base_scan` | LaserScan | Senzori (Stage simulator) |
| `/cmd_vel` | Twist | Brzina robota (samo s navigation node-om) |
| `/goal_pose` | PoseStamped | 2D Goal Pose iz RViz-a |
| `/planned_path` | Path | A* putanja (samo s path planning node-om) |

---

## ✅ Checklist za Brz Start

- [ ] `~/my_map/map.yaml` postoji
- [ ] `~/my_map/map.pgm` postoji
- [ ] Build je uspješan (`colcon build`)
- [ ] Pokrenuli ste launch file
- [ ] RViz prikazuje mapu
- [ ] Možete kliknuti 2D Goal Pose (ako koristite path_planning_with_map_launch.py)
- [ ] Putanja se pojavljuje (zelena linija)
- [ ] Robot se giba (ako koristite path_planning_with_map_launch.py)

---

## 💡 Savjet

Ako trebate **brzo testirati** navigaciju:

```bash
# Pokrenite sve u jednom comandu
ros2 launch student_assignment_02 path_planning_with_map_launch.py & sleep 2 && rviz2
```

Sada:
1. Čekajte da se Stage pokrene (~3 sekunde)
2. RViz će se otvoriti
3. Dodajte displays
4. Kliknite 2D Goal Pose
5. Gledajte rezultat!

---

## 🎓 Razlike Između Launch File-ova

### `complete_mapping_launch.py`
```
Stage → SLAM → Mapa se gradi → A* (ako pokrenete node-ove)
Vrijeme čekanja: ~30 sekundi
Koristi se za: Mapiranje novog okoliša
```

### `navigation_with_map_launch.py`
```
Stage → Unaprijed mapirana mapa → Samo mapa u RViz-u
Vrijeme čekanja: ~3 sekunde
Koristi se za: Brz pregled mape
```

### `path_planning_with_map_launch.py`
```
Stage → Unaprijed mapirana mapa → A* → Navigation
Vrijeme čekanja: ~3 sekunde
Koristi se za: Testiranje A* i navigacije
```

---

**Status:** ✅ Unaprijed mapirana mapa je sada dostupna!

**Zadnja ažuriranja:** 5. siječnja 2026.
