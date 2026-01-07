# A* Path Planner - Brza uputa za pokretanje

## Korak 1: Build paketa

```bash
cd ~/colcon_ws
colcon build --packages-select student_assignment_02
source install/setup.bash
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

### Verzija 1: Korištenjem launch datoteke (preporučeno)
```bash
ros2 launch student_assignment_02 a_star_path_planner.launch.py \
    goal_x:=5.0 \
    goal_y:=5.0 \
    start_x:=0.0 \
    start_y:=0.0
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

## Očekivani izlaz

U terminalu gdje ste pokrenuli `a_star_path_planner` trebali biste vidjeti:

```
[INFO] [a_star_path_planner]: A* Path Planner Node: Started
[INFO] [a_star_path_planner]: Mapa primljena: 200x200, rezolucija: 0.050 m/stanica
[INFO] [a_star_path_planner]: Planiranje putanje od (0, 0) do (100, 100)
[INFO] [a_star_path_planner]: A* završio u 2541 iteracija, istražio 1823 stanica
[INFO] [a_star_path_planner]: Putanja pronađena! Dužina: 87 stanica
```

U RViz-u trebali biste vidjeti:
- 🟤 **Sive sfere**: Istraživane stanice
- 🟡 **Žute sfere**: Čelna fronta pretraživanja
- 🟢 **Zelena linija**: Planirana putanja
- 🟫 **Crna/tamna polja**: Prepreke

## Provjera topic-a

```bash
# Provjeri da li se putanja objavljuje
ros2 topic echo /planned_path

# Provjeri broj marker-a
ros2 topic echo /path_planning_visualization

# Provjeri mrežu čelne fronte
ros2 topic echo /planning_frontier
```

## Problemi i rješenja

### "Package 'student_assignment_02' not found"
```bash
# Provjerite da ste Source-ali setup.bash
source ~/colcon_ws/install/setup.bash
```

### "Mapa nije primljena"
```bash
# Provjerite da je map_republisher pokrenut
ros2 topic list | grep map
# trebali biste vidjeti: /map
```

### "Putanja nije pronađena"
```bash
# Provjerite da je cilj u slobodnom prostoru
# Pokusajte s manjim ciljem npr. goal_x:=2.0 goal_y:=2.0
```

### RViz ne prikazuje ništa
```bash
# Postavite Fixed Frame na 'map'
# Provjerite da su topic-i dostupni:
ros2 topic list
```

## Brzina izvršavanja

Za 200x200 mapu s rezolucijom 0.05m:
- **Vrijeme planiranja**: ~50-200ms
- **Broj istraživanih stanica**: 500-3000 (ovisno o broju prepreka)
- **Dužina putanje**: 20-200 stanica

## Dalje opcije

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
        Node(package='student_assignment_02', executable='a_star_path_planner', name='a_star_path_planner',
            parameters=[{'goal_x': 5.0, 'goal_y': 5.0}]),
    ])
```

Potom pokrenite:
```bash
ros2 launch student_assignment_02 all_in_one.launch.py
```

## Dodatni resursi

- [A_STAR_PATH_PLANNER.md](./A_STAR_PATH_PLANNER.md) - Detaljnija dokumentacija
- ROS 2 dokumentacija: https://docs.ros.org/
- Visualization Markers: https://wiki.ros.org/rviz/DisplayTypes/Marker
