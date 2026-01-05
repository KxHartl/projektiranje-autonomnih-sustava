# Kompletan Vodič - A* Planiranje Putanje i Navigacija Robota

## 🎯 Pregled

Ovaj vodič pokriva:
1. **A* Algoritam** - Pronalaženje optimalnih putanja
2. **Vizualizacija u RViz-u** - Prikor prikaz putanje
3. **Navigacija Robota** - Automatsko slijedjenje putanje

---

## 💫 A* Algoritam - Kako Radi

### Glavne Karakteristike

```
f(n) = g(n) + h(n)

g(n) = Cijena od početka do čvora n
h(n) = Heuristička procjena od čvora n do cilja
f(n) = Ukupna procijenjena cijena putanje kroz n
```

### Učlanke

```cpp
struct PathNode {
    int x, y;           // Koordinate u grid-u
    float g_cost;       // Cijena od početka
    float h_cost;       // Heuristička cijena do cilja
    float f_cost;       // g_cost + h_cost
    PathNode* parent;   // Prethodni čvor
};
```

### Proces

1. **Inicijalizacija**
   - Dodaj početni čvor u open_list
   - Kreiraj closed_list za obidjene čvorove

2. **Glavna Petlja**
   ```
   dok je open_list nije prazna:
       - Pronadi čvor s najmanjom f_cost iz open_list
       - Ako je to cilj: rekonstruiraj putanju i vrati
       - Dodaj u closed_list
       - Za sve susjede:
           - Ako je zauzet ili u closed_list: preskoči
           - Ako je u open_list: ažuriraj ako je novi put jeftiniji
           - Inače: dodaj u open_list
   ```

3. **Heurističke Funkcije**
   
   **Manhattan distanca:**
   ```cpp
   h = |x_cilj - x_trenutni| + |y_cilj - y_trenutni|
   ```
   - Brža
   - Za kretanje u 4 smjera
   
   **Euklidska distanca:**
   ```cpp
   h = sqrt((x_cilj - x_trenutni)^2 + (y_cilj - y_trenutni)^2)
   ```
   - Preciznija
   - Za kretanje u 8 smjerova

### 8-Smjerno Kretanje

```
  NW  N  NE
  (-1,-1) (0,-1) (1,-1)
    \    |    /
     \   |   /
W----(-1,0) +node (1,0)----E
     /   |   \
    /    |    \
  (-1,1) (0,1) (1,1)
  SW  S  SE

Cijena:
- Direktni (N,S,E,W) = 1.0
- Dijagonalni (NE,NW,SE,SW) = 1.414 (sqrt(2))
```

---

## 🚀 Pokretanje - Brza Uputa

### Preduvjeti

1. **Mapirana mapa** (spremaćem ste kao `~/my_map`)
2. **ROS 2 Humble** instaliran
3. **Stage simulator** instaliran
4. **Package izgrađen**

### Korak 1: Priprema

```bash
# Otidi u direktorij
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02

# Očisti stare build datoteke
rm -rf build install log

# Build paket
colcon build
source install/setup.bash

# Očisti RViz cache
rm -rf ~/.config/rviz2/ ~/.rviz2/ ~/.ros/
```

### Korak 2: Pokrenite 3 Terminala

#### **Terminal 1: Stage Simulator + SLAM**
```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

ros2 launch student_assignment_02 complete_mapping_launch.py
```

Oveaj će pokrenuti:
- Stage simulator
- Robot State Publisher
- SLAM Toolbox

#### **Terminal 2: RViz**
```bash
rviz2
```

U RViz-u:
- Kliknite `Panels` → `Add New Panel` → `Displays`
- Dodajte:
  1. **Grid** (rviz_default_plugins/Grid)
  2. **TF** (rviz_default_plugins/TF)
  3. **LaserScan** (rviz_default_plugins/LaserScan)
     - Topic: `/base_scan`
  4. **Map** (rviz_default_plugins/Map)
     - Topic: `/map`
  5. **Path** (rviz_default_plugins/Path)
     - Topic: `/planned_path`
  6. **MarkerArray** (rviz_default_plugins/MarkerArray)
     - Topic: `/visualization_marker_array`

- Postavite `Fixed Frame` na `map`

#### **Terminal 3: Path Planning + Navigation**
```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
source install/setup.bash

# Pokrenite path_planning_node
ros2 run student_assignment_02 path_planning_node
```

U četvrtom terminalu (opciono):
```bash
# Pokrenite goal_navigation_node
ros2 run student_assignment_02 goal_navigation_node
```

---

## 📄 Što bi trebalo vidjeti

### U Stage Simulatoru
- Robot se pojavljuje u svijetu
- LaserScan skenira prepreke

### U RViz-u
- ✅ Grid pozadina (bijete linije)
- ✅ TF transformacije (crvene/zelene/plave osi)
- ✅ LaserScan točke (bijele točke oko robota)
- ✅ Mapa iz SLAM-a (crna = zauzeto, bijelo = slobodno)
- ✅ **Zelena linija** - A* planirana putanja
- ✅ **Zelena sfera** - Početna točka
- ✅ **Crvena sfera** - Ciljna točka

### U Terminalima
```
[path_planning_node]: Nova putanja primljena s 150 točaka
[path_planning_node]: A* završen u 2345 iteracija
[goal_navigation_node]: Waypoint 45 dostignut. Ideći...
```

---

## 🔓 Importantni Parametri

### Path Planning Node

**Funkcionalnost:**
- Svakih 5 sekundi planira novu putanju
- Koristi Manhattan heurističku funkciju
- 8-smjerno kretanje

**Rezultat:**
- Objavljuje `/planned_path` (nav_msgs/Path)
- Objavljuje `/visualization_marker_array` (vizualizacija)

### Goal Navigation Node

**Parametri:**
```yaml
linear_speed: 0.2        # m/s (brzina naprijed)
angular_speed: 0.5       # rad/s (brzina rotacije)
goal_tolerance: 0.1      # m (distanca do waypointa)
angle_tolerance: 0.1     # rad (tolerancija kuta)
```

**Funkcionalnost:**
- Slijedi planiranu putanju waypoint po waypoint
- Rotira se prema sljedećem waypoint-u
- Krene naprijed kada je zarotiran

---

## 🛠 Prilagodba

### Promjena Početne i Ciljne Točke

U `path_planning_node.cpp`, funkt ija `plan_path_callback()`:

```cpp
// Početna pozicija (lijeva strana mape)
int start_x = 5;
int start_y = current_map_.info.height / 2;

// Ciljna pozicija (desna strana mape)
int goal_x = current_map_.info.width - 5;
int goal_y = current_map_.info.height / 2;
```

Promjena:
```cpp
// Primjer: Od (10,10) do (50,50)
int start_x = 10;
int start_y = 10;
int goal_x = 50;
int goal_y = 50;
```

### Promjena Heurističke Funkcije

U `a_star()` funkciji:

```cpp
// Manhattan (trenutna)
float new_h = heuristic(new_x, new_y, goal_x, goal_y);

// Euklidska (alternativa)
float new_h = heuristic_euclidean(new_x, new_y, goal_x, goal_y);
```

### Promjena Brzine Robota

U `goal_navigation_node.cpp`:

```cpp
this->declare_parameter("linear_speed", 0.2);   // Promijeni 0.2 na dr. vrijednost
this->declare_parameter("angular_speed", 0.5);  // Promijeni 0.5 na dr. vrijednost
```

Ili preko command linea:
```bash
ros2 run student_assignment_02 goal_navigation_node \
  --ros-args -p linear_speed:=0.5 -p angular_speed:=1.0
```

---

## 🔍 Dijagnostika

### Putanja se ne planira

1. **Provjerite je li mapa primljena:**
   ```bash
   ros2 topic echo /map --once
   ```
   Trebali biste vidjeti OccupancyGrid podatke

2. **Provjerite logove:**
   ```bash
   ros2 node list
   ```
   Trebali biste vidjeti `/path_planning_node`

3. **Provjerite je li mapa validna:**
   - Ne smije biti sve 0 (sve slobodno)
   - Ne smije biti sve 100 (sve zauzeto)

### Robot se ne giba

1. **Provjerite postoji li putanja:**
   ```bash
   ros2 topic echo /planned_path
   ```

2. **Provjerite da li se "/cmd_vel" publikuje:**
   ```bash
   ros2 topic echo /cmd_vel
   ```
   Trebali biste vidjeti Twist poruke

3. **Provjerite Stage sluša /cmd_vel:**
   ```bash
   ros2 topic list
   ```
   Trebali biste vidjeti `/cmd_vel`

### A* se zaglavilo

Ako A* traje >5 sekundi:
- Mapa je prevelika
- Nema putanje do cilja
- MAX_ITERATIONS je premali (trenutno: 100000)

---

## 📖 Implementacijske Detalje

### Grid Koordinate vs Metrske Koordinate

```cpp
// Grid koordinate (x, y) → Metrske koordinate
metars_x = (grid_x + 0.5) * resolution + origin.x
metars_y = (grid_y + 0.5) * resolution + origin.y
```

### Zašto 8-Smjerna Kretanja?

Umjesto 4-smjernog:
- Omogućava dijagonalne poteze (skraćuje putanju)
- Realištičnije za robote koji se mogu okrenuti
- Bolje iskorištava prostor

### Closed List Optimizacija

Koristim `std::set` za O(log n) pretraživanje:
```cpp
std::set<std::pair<int, int>> closed_set;
```

Umjesto linearne pretrage kroz vektor.

---

## ✨ Rezultat

Kada je sve ispravno postavljeno:

1. **Stage simulator** prikazuje robot u svijetu
2. **RViz** prikazuje mapu, senzore i planiranu putanju
3. **Path Planning Node** pronalazi optimalnu putanju svakih 5 sekundi
4. **Goal Navigation Node** prati planiranu putanju
5. **Robot se automatski giba do cilja**

---

## 📁 Datoteke

| Datoteka | Opis |
|----------|------|
| `path_planning_node.cpp` | A* algoritam i vizualizacija |
| `goal_navigation_node.cpp` | Navigacija robota |
| `CMakeLists.txt` | Build konfiguracija |
| `complete_mapping_launch.py` | Launch file za sve čvorove |

---

**Status:** ✅ A* algoritam i navigacija su kompletan i spreman za korišćenje!

**Zadnja ažuriranja:** 5. siječnja 2026.
