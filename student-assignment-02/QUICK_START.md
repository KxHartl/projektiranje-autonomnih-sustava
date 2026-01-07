# A* Path Planner - Brza uputa za pokretanje

## Korak 1: Build paketa

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build/ install/ log/  # Očisti stare build-e
colcon build --packages-select student_assignment_02
source install/setup.bash
```

⚠️ **VAŽNO:** Ako dobiješ error "libexec directory does not exist", izvrši:
```bash
colcon build --packages-select student_assignment_02 --symlink-install
```

## Korak 2: Pokrenite Stage simulator

U jednom terminalu:
```bash
# Stage simulator s čulom s preprekama
ros2 launch stage_ros2 stage.launch.py world:=<path_to_world_file>.world
```

## Korak 3: Pokrenite Map Republisher (ako nije uključen u launch file)

U drugom terminalu:
```bash
ros2 run student_assignment_02 map_republisher
```

Evo provjere da li je mapa dostupna:
```bash
ros2 topic echo /map --once
```

## Korak 4: Pokrenite A* Path Planner

U trećem terminalu:

### Verzija 1: Koristeći launch datoteke (preporučeno)

**S parametrima:**
```bash
ros2 launch student_assignment_02 a_star_path_planner.launch.py \
    goal_x:=5.0 \
    goal_y:=5.0 \
    start_x:=0.0 \
    start_y:=0.0
```

**Bez parametara (čeka goal iz RViza):**
```bash
ros2 launch student_assignment_02 a_star_path_planner.launch.py
```

### Verzija 2: Direktnim pokretanjem
```bash
ros2 run student_assignment_02 a_star_path_planner --ros-args \
    -p goal_x:=5.0 \
    -p goal_y:=5.0 \
    -p start_x:=0.0 \
    -p start_y:=0.0 \
    -p allow_diagonal:=true
```

## Korak 5: Otvorite RViz

U četvrtom terminalu:
```bash
ros2 run rviz2 rviz2
```

U RViz-u:
1. Postavite **Fixed Frame** na `map`
2. Kliknite "Add" i dodajte:
   - **Map** - `/map` (prikazuje kartu)
   - **Path** - `/planned_path` (prikazuje putanju)
   - **MarkerArray** - `/path_planning_visualization` (istraživane stanice)
   - **MarkerArray** - `/planning_frontier` (čelna fronta)

Ali lakše je koristiti spremljenu konfiguraciju:
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix student_assignment_02)/share/student_assignment_02/config/a_star_path_planning.rviz
```

## 🎯 Dinamički Goal Pose iz RViza

### Korištenje 2D Goal Pose Tool-a

✨ **NOVO:** Čvor sada sluša na `/goal_pose` topic!

1. **U RViz-u**, kliknite na "2D Goal Pose" tool (zeleni strelica)
2. **Kliknite na mapu** gdje želite da bude cilj
3. **Čvor će automatski replanirati putanju**

Ne trebate ponovno pokretati čvor - automatski će se preplanirati kada postavite novi goal!

### Primjer

```bash
# Terminal 1 - Stage
ros2 launch stage_ros2 stage.launch.py

# Terminal 2 - Map Republisher  
ros2 run student_assignment_02 map_republisher

# Terminal 3 - A* Path Planner (sluša na /goal_pose)
ros2 run student_assignment_02 a_star_path_planner

# Terminal 4 - RViz
ros2 run rviz2 rviz2

# Sada možete klikati na mapu u RViz-u da postavite goal!
```

### Programski Goal

Ako trebate zadati goal kao parametar:
```bash
ros2 launch student_assignment_02 a_star_path_planner.launch.py goal_x:=8.5 goal_y:=7.2
```

## Očekivani Izlaz

U terminalu gdje ste pokrenuli `a_star_path_planner` trebali biste vidjeti:

```
[INFO] [a_star_path_planner]: A* Path Planner Node: Started
[INFO] [a_star_path_planner]: Slusa na /goal_pose za dinamicki goal (RViz 2D Goal Pose)
[INFO] [a_star_path_planner]: Mapa primljena: 200x200, rezolucija: 0.050 m/stanica
[INFO] [a_star_path_planner]: Planiranje putanje od (0, 0) do (100, 100)
[INFO] [a_star_path_planner]: A* završio u 2541 iteracija, istraživao 1823 stanica
[INFO] [a_star_path_planner]: Putanja pronađena! Dužina: 87 stanica

# Kada kliknete na novu goal poziciju u RViz-u:
[INFO] [a_star_path_planner]: Nova goal pose primljena iz RViza: (5.25, 6.75)
[INFO] [a_star_path_planner]: Planiranje putanje od (0, 0) do (105, 135)
[INFO] [a_star_path_planner]: A* završio u 1892 iteracija, istraživao 1243 stanica
[INFO] [a_star_path_planner]: Putanja pronađena! Dužina: 72 stanica
```

U RViz-u trebali biste vidjeti:
- 🔴 **Sive sfere**: Istraživane stanice
- 🟡 **Žute sfere**: Čelna fronta pretraživanja
- 🟢 **Zelena linija**: Planirana putanja
- ⬛ **Crna/tamna polja**: Prepreke

## Provjera Topic-a

```bash
# Provjeri da li se putanja objavljuje
ros2 topic echo /planned_path

# Provjeri broj marker-a
ros2 topic echo /path_planning_visualization

# Provjeri frontu
ros2 topic echo /planning_frontier

# Provjeri goal pose
ros2 topic echo /goal_pose
```

## Problemi i Rješenja

### Problem 1: "Package 'student_assignment_02' not found"

```bash
# Provjerite da ste Source-ali setup.bash
source ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/install/setup.bash

# Ili iz student-assignment-02 direktorija:
source install/setup.bash
```

### Problem 2: "libexec directory does not exist"

```bash
# Očisti build i prebildaj:
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build/ install/ log/
colcon build --packages-select student_assignment_02 --symlink-install
source install/setup.bash
```

### Problem 3: "Mapa nije primljena"

```bash
# Provjerite da je map_republisher pokrenut
ros2 run student_assignment_02 map_republisher

# Provjerite topic
ros2 topic list | grep map
# trebali biste vidjeti: /map
```

### Problem 4: "Putanja nije pronađena"

```bash
# Provjerite da je cilj u slobodnom prostoru
# Pokusajte s manjim ciljem npr. goal_x:=2.0 goal_y:=2.0

# Ili kliknite na praznu lokaciju u RViz-u
```

### Problem 5: RViz ne prikazuje ništa

```bash
# 1. Postavite Fixed Frame na 'map'
# 2. Provjerite da su topic-i dostupni:
ros2 topic list

# 3. Provjerite da su markeri dostupni:
ros2 topic echo /path_planning_visualization --once
```

### Problem 6: "2D Goal Pose tool" ne radi

```bash
# Provjerite da je /goal_pose dostupan:
ros2 topic list | grep goal_pose

# Provjerite da je RViz konfiguriran
# Trebali biste vidjeti tool u RViz-u
```

## Brzina Izvršavanja

Za 200x200 mapu s rezolucijom 0.05m:
- **Vrijeme planiranja**: ~50-200ms
- **Broj istraživanih stanica**: 500-3000 (ovisno o broju prepreka)
- **Dužina putanje**: 20-200 stanica

## Dalje Opcije

### Pokrenite sve u jednoj launch datoteci

Stvorite novu `all_in_one.launch.py` datoteku:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(package='stage_ros2', executable='stage_ros2_node', name='stage_ros2'),
        Node(package='student_assignment_02', executable='map_republisher', name='map_republisher'),
        Node(package='student_assignment_02', executable='a_star_path_planner', name='a_star_path_planner'),
    ])
```

Potom pokrenite:
```bash
ros2 launch student_assignment_02 all_in_one.launch.py
```

## Tips i Trikovi

### Promijenite Rezoluciju Vizualizacije

Ako je previše markera ili previše malo, izmijenjat ćete kod u `a_star_path_planner.py`:
```python
# Linija ~300:
if idx % 5 != 0:  # Prikazuje svaku 5. stanicu
```

Promijenite `5` na različitim vrijednostima:
- `2` - više markera (bolja vizualizacija, sporije)
- `10` - manje markera (brže, manja vizualizacija)

### Brže Planiranje

Ako planing traje previše dugo, možete:
1. Povećati `inflation_radius` kako biste izbjegli prepreke
2. Onemogućiti dijagonale: `allow_diagonal:=false`
3. Postavljati ciljeve na bliže lokacije

### Detaljnije Logging-a

```bash
# Vidjeti sve ROS debug poruke:
ros2 run student_assignment_02 a_star_path_planner --ros-args --log-level DEBUG
```

## Dodatni Resursi

- [A_STAR_PATH_PLANNER.md](./A_STAR_PATH_PLANNER.md) - Detaljnija dokumentacija
- [README_A_STAR.md](./README_A_STAR.md) - Kompletan README
- ROS 2 dokumentacija: https://docs.ros.org/
- Visualization Markers: https://wiki.ros.org/rviz/DisplayTypes/Marker
- RViz 2D Goal Pose: https://github.com/ros2/rviz/tree/humble/rviz_default_plugins
