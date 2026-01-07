# A* Path Planner Node

## Opis

`a_star_path_planner` je ROS 2 čvor koji koristi **A* algoritam** za planiranje optimalne putanje na 2D mapi dobivenoj iz Stage simulatora. Čvor vizualizira proces pretraživanja prostora i objavljivava pronađenu putanju.

## Funkcionalnosti

✅ **A* Algoritam** - Efikasno pretraživanje s heurističkom funkcijom (Euklidska distanca)  
✅ **Vizualizacija pretraživanja** - Prikaz svih istraživanih stanica u RViz-u  
✅ **Vizualizacija fronte** - Prikaz čelne fronte pretraživanja (stanice u open setu)  
✅ **Objava putanje** - Objavljivanje planiranu putanju kao `nav_msgs/Path`  
✅ **Parametrizacija** - Podesivi start/goal, dozvola dijagonalnog kretanja  
✅ **4-povezanost i dijagonale** - Podrška za oba tipa kretanja  

## ROS Interfejsi

### Subscriptions

| Topic | Tip | Opis |
|-------|-----|------|
| `/map` | `nav_msgs/OccupancyGrid` | Primanje mape iz Stage simulatora (Transient Local QoS) |

### Publications

| Topic | Tip | Opis |
|-------|-----|------|
| `/planned_path` | `nav_msgs/Path` | Planirana putanja od starta do cilja |
| `/path_planning_visualization` | `visualization_msgs/MarkerArray` | Vizualizacija istraživanih stanica (sive sfere) |
| `/planning_frontier` | `visualization_msgs/MarkerArray` | Vizualizacija čelne fronte (žute sfere) |

### Parametri

| Parametar | Tip | Zadana vrijednost | Opis |
|-----------|-----|------------------|------|
| `goal_x` | float | 5.0 | X koordinata cilja (m) |
| `goal_y` | float | 5.0 | Y koordinata cilja (m) |
| `start_x` | float | 0.0 | X koordinata početka (m) |
| `start_y` | float | 0.0 | Y koordinata početka (m) |
| `allow_diagonal` | bool | true | Dozvoli dijagonalno kretanje |
| `inflation_radius` | int | 1 | Inflation radius oko prepreka (stanice) |

## Pokretanje

### Korištenjem launch datoteke

```bash
ros2 launch student_assignment_02 a_star_path_planner.launch.py goal_x:=10 goal_y:=10
```

### S custom parametrima

```bash
ros2 run student_assignment_02 a_star_path_planner --ros-args \
  -p goal_x:=8.0 \
  -p goal_y:=8.0 \
  -p start_x:=1.0 \
  -p start_y:=1.0
```

## A* Algoritam

### Pseudokod

```
open_set = {start}
came_from = {}
g_score = {start: 0}
f_score = {start: h(start, goal)}

while open_set nije prazna:
    current = čvor s najmanjim f_score iz open_set
    
    if current == goal:
        rekonstruiraj putanju
        return putanja
    
    remove current iz open_set
    
    for neighbor u susjedi(current):
        tentative_g = g_score[current] + cost(current, neighbor)
        
        if tentative_g < g_score[neighbor]:
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score[neighbor] = g_score[neighbor] + h(neighbor, goal)
            add neighbor u open_set

return "Nema putanje"
```

### Heurističke funkcije

- **Euklidska distanca**: `h(n) = sqrt((n.x - goal.x)² + (n.y - goal.y)²)`
- **Heuristika je dopustiva** (nikada ne precjenjuje), što garantira optimalnost A*

### Tro šak kretanja

- **Ortogonalno** (gore, dolje, lijevo, desno): cost = 1
- **Dijagonalno** (ako je omogućeno): cost = √2

## Vizualizacija u RViz-u

### Slojevi

1. **Map** - Karta s preprekama (s `map_republisher`)
2. **Grid** - Referentna mreža
3. **Explored Cells** - Sve istraživane stanice (sive sfere, prikaz svakog 5. elementa)
4. **Frontier** - Čelna fronta pretraživanja (žute sfere)
5. **Planned Path** - Finalna putanja (zelena linija)
6. **Odometry** - Pozicija robota

### Interpretacija boja

- 🟤 **Sive sfere** - Stanice koje je algoritam već istraživao
- 🟡 **Žute sfere** - Stanice u čelnoj fronti (čekaju istraživanje)
- 🟢 **Zelena linija** - Finalna optimalna putanja
- 🟫 **Crne/tamne sjene** - Prepreke na mapi

## Primjer korištenja

### 1. Pokrenite Stage simulator

```bash
ros2 launch stage_ros2 stage.launch.py world:=path_planning_world.world
```

### 2. Pokrenite map republisher (ako nije pokrenut)

```bash
ros2 run student_assignment_02 map_republisher
```

### 3. Pokrenite A* path planner

```bash
ros2 launch student_assignment_02 a_star_path_planner.launch.py goal_x:=5 goal_y:=5
```

### 4. Otvorite RViz

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix student_assignment_02)/share/student_assignment_02/config/a_star_path_planning.rviz
```

## Informacioni izlazi

Čvor ispisuje sljedeće informacije:

```
[INFO] A* Path Planner Node: Started
[INFO] Mapa primljena: 100x100, rezolucija: 0.050 m/stanica
[INFO] Planiranje putanje od (0, 0) do (100, 100)
[INFO] A* završio u 1534 iteracija, istraživao 832 stanica
[INFO] Putanja pronađena! Dužina: 45 stanica
```

## Performanse

- **Vremenska složenost**: O((V + E) * log V) gdje je V broj stanica, E broj bridova
- **Prostorna složenost**: O(V) za čuvanje istraživanih i otvorenih čvorova
- **Optimnost**: A* s dopustivom heurističkom funkcijom garantira optimalnu putanju
- **Tipično vrijeme izvršavanja**: < 100ms za 100x100 mapu

## Mogućnosti za poboljšanja

1. **Inflacija prepreka** - Razmotriti rastojanje od prepreka, ne samo zauzete stanice
2. **Jump Point Search** - Ubrzava A* za uniformne grafe
3. **Bidirectional A*** - Pretraživanje s obje strane
4. **Dinamička preplaniranje** - Ako se mapa promijeni tijekom izvršavanja
5. **Path smoothing** - Post-obrada putanje za glatku navigaciju
6. **Cost map** - Korištenje cost map-a umjesto samo OccupancyGrid-a

## Instalacija i build

```bash
cd ~/colcon_ws/src/projektiranje-autonomnih-sustava/student-assignment-02
colcon build --packages-select student_assignment_02
```

## Problemi i rješenja

### Problem: "Putanja nije pronađena"

**Uzrok**: Goal je u prepreci ili nije dostigljiv  
**Rješenje**: Provjerite parametre `goal_x`, `goal_y` i da li su u slobodnom prostoru

### Problem: Čvor se ne pokreće

**Uzrok**: Mapa nije primljena  
**Rješenje**: Provjerite da `map_republisher` radi i da je `/map` dostupan

### Problem: Spora vizualizacija

**Uzrok**: Previše markera se prikazuje  
**Rješenje**: Kod prikazuje samo svaki 5. element istraživanih stanica, to je normalno

## Autori i izvori

- Implementacija: KxHartl
- ROS 2 dokumentacija: https://docs.ros.org/
- A* algoritam: https://en.wikipedia.org/wiki/A*_search_algorithm

## Licenca

Apache-2.0
