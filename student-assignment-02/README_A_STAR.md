# ROS2 A* Path Planning - Assignment 02

ROS 2 Python paket za automatizirane sustave koji implementira **A* algoritam** za planiranje optimalne putanje s vizualizacijom pretraživanja u RViz-u.

## Pregled

Ovaj paket radi s **Stage simulatorom** i **SLAM Toolbox**-om kako bi:
1. ✅ Primio mapu iz Stage simulatora
2. ✅ Planirao optimalnu putanju korištenjem A* algoritma
3. ✅ Vizualizirao proces pretraživanja (istraživane stanice i frontu)
4. ✅ Objavio planiranu putanju kao `nav_msgs/Path`

## Čvorovi u paketu

| Čvor | Opis |
|------|------|
| `map_republisher` | Kontinuirano objavljuje `/map` s Transient Local QoS |
| **`a_star_path_planner`** | **NOVI** - A* planiranje putanje s vizualizacijom |
| `path_planning_node` | (Postojeći) |
| `goal_navigation_node` | (Postojeći) |

## A* Path Planner Čvor

### Karakteristike
- 🎯 **Optimalna putanja** - Garantira najkraću putanju
- 👁️ **Vizualizacija** - Prikaz svih istraživanih stanica i fronte u RViz-u
- ⚙️ **Parametrizabilna** - Start/goal koordinate, dozvola dijagonala
- 🚀 **Brza** - A se izvršava u <200ms za 200x200 mape
- 4️⃣ **4-povezanost + dijagonale** - Podrška za oba tipa kretanja

### ROS Interfejsi

**Subscriptions:**
- `/map` (nav_msgs/OccupancyGrid) - Mapa iz simulatora

**Publications:**
- `/planned_path` (nav_msgs/Path) - Planirana putanja
- `/path_planning_visualization` (visualization_msgs/MarkerArray) - Istraživane stanice
- `/planning_frontier` (visualization_msgs/MarkerArray) - Čelna fronta

**Parametri:**
- `goal_x`, `goal_y` - Koordinate cilja (m)
- `start_x`, `start_y` - Koordinate početka (m)
- `allow_diagonal` - Dozvoli dijagonalno kretanje (bool)
- `inflation_radius` - Inflation radius oko prepreka (int)

## Brzi početak

### Instalacija
```bash
cd ~/colcon_ws
colcon build --packages-select student_assignment_02
source install/setup.bash
```

### Pokretanje

**Korak 1:** Stage simulator
```bash
ros2 launch stage_ros2 stage.launch.py
```

**Korak 2:** Map republisher (ako trebate)
```bash
ros2 run student_assignment_02 map_republisher
```

**Korak 3:** A* Path Planner
```bash
ros2 launch student_assignment_02 a_star_path_planner.launch.py goal_x:=5.0 goal_y:=5.0
```

**Korak 4:** RViz
```bash
ros2 run rviz2 rviz2
```

> Detaljnije upute: [QUICK_START.md](./QUICK_START.md)

## Datoteke u paketu

```
student-assignment-02/
├── src/student_assignment_02/
│   ├── student_assignment_02/
│   │   ├── map_republisher.py                    (Postojeći)
│   │   ├── a_star_path_planner.py               ✨ NOVI - Glavni čvor
│   │   ├── path_planning_node.py                (Postojeći)
│   │   └── goal_navigation_node.py              (Postojeći)
│   │
│   ├── launch/
│   │   └── a_star_path_planner.launch.py        ✨ NOVI - Launch datoteka
│   │
│   ├── config/
│   │   ├── a_star_path_planner_examples.yaml    ✨ NOVI - Primjeri konfiguracije
│   │   └── a_star_path_planning.rviz            (RViz konfiguracija)
│   │
│   ├── setup.py                                 (Ažurirano - dodana entry point)
│   └── package.xml
│
├── A_STAR_PATH_PLANNER.md                       ✨ NOVI - Detaljnija dokumentacija
├── QUICK_START.md                               ✨ NOVI - Brza uputa
└── README.md                                    (Ovaj fajl)
```

## A* Algoritam

### Kako radi

1. **Open set** - Svi kandidati čvorovi za ekspanziju (inicijalno samo start)
2. **Closed set** - Istraživani čvorovi
3. **g(n)** - Cijena puta do čvora n
4. **h(n)** - Heuristička procjena do cilja (Euklidska distanca)
5. **f(n) = g(n) + h(n)** - Ukupna procjena

### Heuristika
```
h(n) = sqrt((n.x - goal.x)² + (n.y - goal.y)²)
```

Ova heuristika je **dopustiva** što znači da nikada ne precjenjuje, garantirajući optimalnost.

### Vremenska složenost
- **O((V + E) log V)** gdje je V broj čvorova, E broj bridova

## Vizualizacija u RViz-u

### Što se prikazuje

- 🟫 **Sive sfere** - Istraživani čvorovi (svaki 5.)
- 🟨 **Žute sfere** - Čelna fronta pretraživanja
- 🟩 **Zelena linija** - Finalna putanja
- ⬛ **Crne/tamne polje** - Prepreke

## Primjer izvršavanja

```bash
# Terminal 1: Stage
ros2 launch stage_ros2 stage.launch.py world:=simple_world.world

# Terminal 2: Map republisher
ros2 run student_assignment_02 map_republisher

# Terminal 3: A* Path Planner
ros2 launch student_assignment_02 a_star_path_planner.launch.py \
    goal_x:=5.0 \
    goal_y:=5.0

# Terminal 4: RViz
ros2 run rviz2 rviz2
```

**Očekivani izlaz:**
```
[INFO] A* Path Planner Node: Started
[INFO] Mapa primljena: 200x200, rezolucija: 0.050 m/stanica
[INFO] Planiranje putanje od (0, 0) do (100, 100)
[INFO] A* završio u 2541 iteracija, istraživao 1823 stanica
[INFO] Putanja pronađena! Dužina: 87 stanica
```

## Dokumentacija

- **[A_STAR_PATH_PLANNER.md](./A_STAR_PATH_PLANNER.md)** - Detaljno objašnjenje A* algoritma, ROS interfejsa i tuning-a
- **[QUICK_START.md](./QUICK_START.md)** - Brza uputa s primjerima i troubleshooting-om
- **[ROS 2 Documentation](https://docs.ros.org/)** - Službena ROS 2 dokumentacija

## Instalacija zavisnosti

```bash
# ROS 2 paketi
rosdep install --from-paths . --ignore-src -r -y

# Python zavisnosti (ako potrebne)
pip install numpy
```

## Problemi?

### Mapa nije primljena
```bash
# Provjerite da je map_republisher pokrenut
ros2 run student_assignment_02 map_republisher
```

### Putanja nije pronađena
```bash
# Provjerite da je cilj u slobodnom prostoru
# Pokusajte s manjim ciljem:
ros2 launch student_assignment_02 a_star_path_planner.launch.py goal_x:=2.0 goal_y:=2.0
```

### RViz ne prikazuje ništa
```bash
# Postavite Fixed Frame na 'map'
ros2 topic list  # provjerite dostupne topic-e
```

Detaljnije: [QUICK_START.md](./QUICK_START.md#problemi-i-rješenja)

## Tehničke specifikacije

| Parametar | Vrijednost |
|-----------|----------|
| **Heuristika** | Euklidska distanca (dopustiva) |
| **Kretanje** | 4-povezanost + opciono dijagonale |
| **Tro šak** | Ortogonalno: 1, Dijagonalno: √2 |
| **Max iteracija** | 10,000 |
| **Vrijeme izvršavanja** | <200ms za 200x200 mape |
| **QoS** | Transient Local (kao /map) |
| **ROS verzija** | ROS 2 (tested on Iron, Humble) |
| **Python verzija** | 3.8+ |

## Mogućnosti za proširenje

1. ✅ **Inflacija prepreka** - Razmotriti udaljenost od prepreka
2. ✅ **Dinamičko planiranje** - Ako se mapa promijeni tijekom izvršavanja
3. ✅ **Path smoothing** - Post-obrada putanje za glatnu navigaciju
4. ✅ **Bidirectional A*** - Pretraživanje s obje strane
5. ✅ **Jump Point Search** - Ubrzanje za uniformne grafe
6. ✅ **Multi-resolution planning** - Prvo grubo, zatim fino planiranje

## Autor

**Krešimir Hartl** (KxHartl)  
Email: kh239762@fsb.hr

## Licenca

Apache-2.0

## Citiranje

Ako koristite ovaj kod, molimo citirajte:

```bibtex
@software{hartl2026astar,
  author = {Hartl, Krešimir},
  title = {A* Path Planning Node for ROS 2},
  year = {2026},
  url = {https://github.com/KxHartl/projektiranje-autonomnih-sustava}
}
```

---

**Zadnja ažuriranja:** January 7, 2026
