# ROS2 A* Path Planning - Assignment 02

ROS 2 Python paket za automatizirane sustave koji implementira **A* algoritam** za planiranje optimalne putanje s vizualizacijom pretraživanja u RViz-u.

## Pregled

Ovaj paket radi s **Stage simulatorom** i **SLAM Toolbox**-om kako bi:
1. ✅ Primio mapu iz Stage simulatora
2. ✅ Planirao optimalnu putanju korištenjem A* algoritma
3. ✅ Vizualizirao proces pretraživanja (istraživane stanice i frontu)
4. ✅ Objavio planiranu putanju kao `nav_msgs/Path`
5. ✅ **NOVO!** Slušao na `/goal_pose` iz RViza (2D Goal Pose tool)

## Čvorovi u paketu

| Čvor | Opis |
|------|------|
| `map_republisher` | Kontinuirano objavljuje `/map` s Transient Local QoS |
| **`a_star_path_planner`** | **GLAVNI** - A* planiranje putanje s vizualizacijom |
| `path_planning_node` | (Postojeći) |
| `goal_navigation_node` | (Postojeći) |

## A* Path Planner Čvor

### Karakteristike
- 🎯 **Optimalna putanja** - Garantira najkraću putanju
- 👁️ **Vizualizacija** - Prikaz svih istraživanih stanica i fronte u RViz-u
- ✨ **NOVO!** **Dinamiki Goal** - Kliknite na mapu u RViz-u da postavite cilj
- ⚙️ **Parametrizabilna** - Start/goal koordinate, dozvola dijagonala
- 🚀 **Brza** - A* se izvršava u <200ms za 200x200 mape
- 4️⃣ **4-povezanost + dijagonale** - Podrška za oba tipa kretanja

### ROS Interfejsi

**Subscriptions:**
- `/map` (nav_msgs/OccupancyGrid) - Mapa iz simulatora
- **`/goal_pose`** (geometry_msgs/PoseStamped) - **NOVO!** Goal iz RViza (2D Goal Pose tool)

**Publications:**
- `/planned_path` (nav_msgs/Path) - Planirana putanja
- `/path_planning_visualization` (visualization_msgs/MarkerArray) - Istraživane stanice
- `/planning_frontier` (visualization_msgs/MarkerArray) - Čelna fronta

**Parametri:**
- `goal_x`, `goal_y` - Koordinate cilja (m) - presloba se ako je goal postavljen iz RViza
- `start_x`, `start_y` - Koordinate početka (m)
- `allow_diagonal` - Dozvoli dijagonalno kretanje (bool)
- `inflation_radius` - Inflation radius oko prepreka (int)

## 🚀 Brzi Početak

### Instalacija

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build/ install/ log/  # Očisti stare build-e
colcon build --packages-select student_assignment_02 --symlink-install
source install/setup.bash
```

**Ili koristite skriptu:**
```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
chmod +x build_and_run.sh
./build_and_run.sh build
```

### Pokretanje

**Najjednostavnije - sve u 4 terminala:**

**Terminal 1 - Stage simulator:**
```bash
ros2 launch stage_ros2 stage.launch.py
```

**Terminal 2 - Map republisher:**
```bash
ros2 run student_assignment_02 map_republisher
```

**Terminal 3 - A* Path Planner:**
```bash
ros2 run student_assignment_02 a_star_path_planner
```

**Terminal 4 - RViz:**
```bash
ros2 run rviz2 rviz2
```

### Postavljanje Cilja iz RViza

1. **U RViz-u**, kliknite na "2D Goal Pose" tool (zeleni strelica u gornjoj traci)
2. **Kliknite na bilo gdje na mapi** gdje želite da bude cilj
3. **Čvor će automatski replanirati putanju**

🌟 **Nema potrebe za restartanjem!** Putanja se replanira svakim klikom.

> Detaljnije upute: [QUICK_START.md](./QUICK_START.md)

## Datoteke u paketu

```
student-assignment-02/
├── src/student_assignment_02/
│   ├── student_assignment_02/
│   │   ├── map_republisher.py                    (Postojeći)
│   │   ├── a_star_path_planner.py               ✨ GLAVNI - Sada s goal_pose
│   │   ├── path_planning_node.py                (Postojeći)
│   │   └── goal_navigation_node.py              (Postojeći)
│   │
│   ├── launch/
│   │   └── a_star_path_planner.launch.py        ✨ Launch datoteka
│   │
│   ├── config/
│   │   ├── a_star_path_planner_examples.yaml    ✨ Primjeri konfiguracije
│   │   └── a_star_path_planning.rviz            (RViz konfiguracija)
│   │
│   ├── setup.py                                 (Ažurirano)
│   └── package.xml
│
├── build_and_run.sh                           ✨ NOVI - Build skripta
├── A_STAR_PATH_PLANNER.md                       ✨ Tehnenska dokumentacija
├── QUICK_START.md                               ✨ Brza uputa
├── README_A_STAR.md                             (Ovaj fajl)
└── README.md                                    (Postojeći)
```

## A* Algoritam

### Kako Radi

1. **Open set** - Svi kandidati čvorovi za ekspanziju (inicijalno samo start)
2. **Closed set** - Istraživani čvorovi
3. **g(n)** - Cijena puta do čvora n
4. **h(n)** - Heuristička procjena do cilja (Euklidska distanca)
5. **f(n) = g(n) + h(n)** - Ukupna procjena

### Heuristika
```
h(n) = sqrt((n.x - goal.x)² + (n.y - goal.y)²)
```

Ova heuristika je **dopustiva** što znači da nikada ne precjenjuje, garantirujući optimalnost.

### Vremenska Složenost
- **O((V + E) log V)** gdje je V broj čvorova, E broj bridova

## Vizualizacija u RViz-u

### Što se Prikazuje

- 🔴 **Sive sfere** - Istraživani čvorovi (svaki 5.)
- 🟨 **Žute sfere** - Čelna fronta pretraživanja
- 🟩 **Zelena linija** - Finalna putanja
- ⬛ **Crne/tamne polje** - Prepreke

## Primjer Izvršavanja

### Scenario 1: S Parametrima
```bash
# Terminal 1: Stage
ros2 launch stage_ros2 stage.launch.py world:=simple_world.world

# Terminal 2: Map republisher
ros2 run student_assignment_02 map_republisher

# Terminal 3: A* s parametrima
ros2 launch student_assignment_02 a_star_path_planner.launch.py \
    goal_x:=5.0 \
    goal_y:=5.0

# Terminal 4: RViz
ros2 run rviz2 rviz2
```

### Scenario 2: Dinamički Goal iz RViza
```bash
# Terminal 1: Stage
ros2 launch stage_ros2 stage.launch.py

# Terminal 2: Map republisher
ros2 run student_assignment_02 map_republisher

# Terminal 3: A* (bez parametara - čeka goal iz RViza)
ros2 run student_assignment_02 a_star_path_planner

# Terminal 4: RViz
ros2 run rviz2 rviz2

# Sada kliknite na mapu u RViz-u da postavite goal!
```

**Očekivani izlaz:**
```
[INFO] A* Path Planner Node: Started
[INFO] Slusa na /goal_pose za dinamicki goal (RViz 2D Goal Pose)
[INFO] Mapa primljena: 200x200, rezolucija: 0.050 m/stanica

# Kada kliknete na mapu:
[INFO] Nova goal pose primljena iz RViza: (5.25, 6.75)
[INFO] Planiranje putanje od (0, 0) do (105, 135)
[INFO] A* završio u 1892 iteracija, istraživao 1243 stanica
[INFO] Putanja pronađena! Dužina: 72 stanica
```

## Tehnike Specifikacije

| Parametar | Vrijednost |
|-----------|----------|
| **Heuristika** | Euklidska distanca (dopustiva) |
| **Kretanje** | 4-povezanost + opciono dijagonale |
| **Trošak** | Ortogonalno: 1, Dijagonalno: √2 |
| **Max iteracija** | 10,000 |
| **Vrijeme izvršavanja** | <200ms za 200x200 mape |
| **QoS** | Transient Local (kao /map) |
| **ROS verzija** | ROS 2 (Iron, Humble) |
| **Python verzija** | 3.8+ |

## Mogućnosti za Proširenje

1. ✅ **Dinamičko planiranje** - Ako se mapa promijeni (vec implementirano!)
2. ✅ **Path smoothing** - Post-obrada putanje za glatnu navigaciju
3. ✅ **Bidirectional A*** - Pretraživanje s obje strane
4. ✅ **Jump Point Search** - Ubrzanje za uniformne grafe
5. ✅ **Multi-resolution planning** - Prvo grubo, zatim fino planiranje
6. ✅ **Cost map** - Korištenje cost map-a umjesto samo OccupancyGrid-a

## Troubleshooting

### Build Problemi

**Greška: "libexec directory does not exist"**
```bash
colcon build --packages-select student_assignment_02 --symlink-install
```

**Greška: "Package not found"**
```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash
```

### Runtime Problemi

**"Mapa nije primljena"**
```bash
ros2 run student_assignment_02 map_republisher
```

**"Putanja nije pronađena"**
- Provjerite da je cilj u slobodnom prostoru
- Pokusajte s bližom lokacijom

**"RViz ne prikazuje ništa"**
- Postavite Fixed Frame na 'map'
- Provjerite da su topic-i dostupni: `ros2 topic list`

Detaljnije: [QUICK_START.md](./QUICK_START.md#problemi-i-rješenja)

## Dokumentacija

| Datoteka | Sadržaj |
|----------|--------|
| **[QUICK_START.md](./QUICK_START.md)** | Brza uputa s primjerima i build instrukcijama |
| **[A_STAR_PATH_PLANNER.md](./A_STAR_PATH_PLANNER.md)** | Tehnieke detalje i tunig |
| **[build_and_run.sh](./build_and_run.sh)** | Skripta za automatski build |

## 🌟 Novi Features - v2.0

✨ **Dinamički Goal Pose**
- Sluša na `/goal_pose` topic
- RViz 2D Goal Pose tool integracija
- Automatska replaniranja bez restarta

✨ **Build Script**
- `./build_and_run.sh build` - Automatski build
- `./build_and_run.sh clean` - Očist starih build-a
- `./build_and_run.sh run_all` - Prikaži sve naredbe

✨ **Poboljšana Dokumentacija**
- Build troubleshooting
- Dinamički goal primjeri
- Detaljnije greške

## Instalacija Zavisnosti

```bash
# ROS 2 paketi
rosdep install --from-paths . --ignore-src -r -y

# Python zavisnosti
pip install numpy
```

## Autor

**Krešimir Hartl** (KxHartl)  
Email: kh239762@fsb.hr  
Projekt: Autonomous Systems Planning - Assignment 02  
Datum: 7. siječnja 2026.

## Licenca

Apache-2.0

## Citiranje

```bibtex
@software{hartl2026astar,
  author = {Hartl, Krešimir},
  title = {A* Path Planning Node for ROS 2 with Dynamic Goal Pose},
  year = {2026},
  url = {https://github.com/KxHartl/projektiranje-autonomnih-sustava}
}
```

---

**Ažurljenja:** 7. siječnja 2026. - Dodana dinamička goal pose integracija
