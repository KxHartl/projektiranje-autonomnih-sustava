# Student Assignment 02: SLAM Mapping i Path Planning

Ovaj paket implementira SLAM mapiranje koristeći Stage simulator, online asinkroni SLAM Toolbox, i A* path planning algoritam sa vizualizacijom u RViz-u.

## Instalacija

### 1. Instalacija potrebnih paketa

```bash
sudo apt-get update
sudo apt-get install ros-humble-stage-ros2
sudo apt-get install ros-humble-slam-toolbox
sudo apt-get install ros-humble-rviz2
sudo apt-get install ros-humble-robot-state-publisher
```

### 2. Kompajliranje paketa

```bash
cd ~/ros2_ws  # ili vaš ROS2 workspace
colcon build --packages-select student_assignment_02
source install/setup.bash
```

Ako ima grešaka pri kompajliranju, provjerite da su sve zavisnosti instalirane:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Struktura Paketa

```
student_assignment_02/
├── launch/
│   ├── stage_launch.py              # Stage simulator + TF + RViz
│   ├── online_async_launch.py        # SLAM Toolbox async mapper
│   ├── complete_mapping_launch.py    # Kompletan sistem
│   ├── navigation_launch.py          # Nav2 navigacijski stack (optional)
│   └── localization_launch.py        # Lokalizacija (optional)
├── config/
│   ├── mapper_params_online_async.yaml  # SLAM parametri
│   ├── nav2_params.yaml              # Nav2 parametri
│   └── rviz_config.rviz              # RViz konfiguracija
├── src/
│   └── path_planning_node.cpp        # A* algoritam i vizualizacija
├── world/
│   └── map_01.world                  # Stage simulacijska okruženja
└── CMakeLists.txt / package.xml      # Konfiguracija kompajliranja
```

## Pokretanje

### Opcija 1: Kompletan sistem (PREPORUČENO)

Ovo pokreće Stage simulator, SLAM mapiranje, path planning i RViz na jednom mjestu:

```bash
ros2 launch student_assignment_02 complete_mapping_launch.py
```

### Opcija 2: Samo Stage + RViz (bez SLAM-a)

```bash
ros2 launch student_assignment_02 stage_launch.py stage:=true rviz:=true
```

### Opcija 3: Samo SLAM (pretpostavka: Stage je već pokrenut)

```bash
ros2 launch student_assignment_02 online_async_launch.py use_sim_time:=true
```

## Rješavanje Problema

### Problem: "tf not published" u RViz-u

**Rješenje:** Stage simulator sada koristi `enforce_prefixes: False`, što omogućava ispravnu publikaciju transformacija.

### Problem: LaserScan se ne prikazuje u RViz-u

**Rješenje:** Provjerite da su sljedeće transformacije dostupne:

```bash
ros2 run tf2_tools view_frames.py
```

Očekivana TF hijerarhija:
```
map
 └── odom
     └── base_link
         └── base_scan
```

Provjerite topike:

```bash
ros2 topic list
```

Očekivani topici:
- `/base_scan` - LaserScan podaci
- `/map` - Mapirana mapa (od SLAM-a)
- `/tf` - Transformacije
- `/odom` - Odometrija (ako je dostupna)

### Problem: SLAM toolbox se ne pokreće

**Rješenje:** Provjerite da je topic `/base_scan` dostupan. Ako Stage koristi drugačije ime topika, trebate ga remap-ati u launch datoteci.

Provjerite dostupne topike:

```bash
ros2 topic info /base_scan
```

## RViz Vizualizacija

Kada se RViz pokrene, trebate dodati sljedeće displaye:

1. **Grid** - Prikazaž grid za referencu
2. **TF** - Prikazuje sve transformacijske frame-ove
3. **LaserScan** - Prikazuje trenutne laser snimke
4. **Map** - Prikazuje mapiranu mapu
5. **Markers** - Prikazuje A* pretragu i putanju

### Automatska Konfiguracija

RViz se automatski učitava s konfiguracijskom datotekom `config/rviz_config.rviz` koja sadrži sve potrebne displaye.

Ako trebate prilagoditi konfiguraciju:

```bash
rviz2 -d ~/ros2_ws/install/student_assignment_02/share/student_assignment_02/config/rviz_config.rviz
```

## Path Planning - A* Algoritam

### Kako radi

1. **Čitanje mape** - `path_planning_node` čeka `/map` topik od SLAM-a
2. **A* pretraga** - Primjer planira putanju od (1,1) do (10,10) u mapiranom prostoru
3. **Vizualizacija** - Putanja se prikazuje kao:
   - 🟢 **Zelena linija** - Planirana putanja
   - 🟢 **Zelena sfera** - Početna točka
   - 🔴 **Crvena sfera** - Ciljna točka

### Prilagođavanje Start/Goal Točaka

U datoteci `src/path_planning_node.cpp`, u `map_callback` funkciji:

```cpp
if (map_received_) {
    plan_path(1, 1, 10, 10);  // Promijenite (1,1) i (10,10) po potrebi
}
```

Brojevi predstavljaju indekse grid stanica (x, y).

## Simulacija Robot Kretanja

Za upravljanje robotom u Stage simulatoru:

1. Otvorite novi terminal
2. Koristite `Teleop` za upravljanje:

```bash
ros2 run turtlebot3_teleop teleop_keyboard  # Ako je dostupan
```

Ili koristite RViz `2D Nav Goal` tool za postavljanje cilja.

## Mapiranje Naprednosti

Provjerite napredak mapiranja kroz RViz monitor:

1. Mapu možete vidjeti u RViz prozoru (Map display)
2. LaserScan točke pokazuju dostignute senzorske podatke
3. TF frame-ovi pokazuju robot poziciju (base_link)

### Spremanje Mape

Nakon što ste zadovoljni sa mapom, mogu je spremiti:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

Ovo će kreirati:
- `my_map.pgm` - Mapira kao slika
- `my_map.yaml` - Mapira metapodaci

## Parametri SLAM-a

SVAKI parametar u `config/mapper_params_online_async.yaml` može se prilagoditi:

- `resolution: 0.05` - Veličina grid stanice (5cm)
- `scan_topic: /base_scan` - Topic od kojeg se čitaju laser skeni
- `do_loop_closing: true` - Detektira i zattvara petlje
- `minimum_travel_distance: 0.5` - Minimalna distanca prije nove skenirane slike

## Napredne Opcije

### Korištenje različitih Okruženja

Ako imate druge `.world` datoteke, koristite ih s:

```bash
ros2 launch student_assignment_02 complete_mapping_launch.py world:=moja_mapa
```

### Lokalizacija umjesto Mapiranja

Za lokalizaciju na postojećoj mapi:

```bash
ros2 launch student_assignment_02 localization_launch.py
```

### Navigacija (Nav2)

Za autonomnu navigaciju robota:

```bash
ros2 launch student_assignment_02 navigation_launch.py use_sim_time:=true
```

## Debugging

### Prikazivanje svih dostupnih topika:

```bash
ros2 topic list
```

### Praćenje specifičnog topika:

```bash
ros2 topic echo /base_scan
```

### Provjera transformacija:

```bash
ros2 run tf2_tools view_frames.py
xdg-open frames.pdf  # Za vizualizaciju
```

### ROS2 logging:

```bash
ros2 launch student_assignment_02 complete_mapping_launch.py --log-level debug
```

## Očekivani Rezultati

Nakon pokretanja sustava, trebali biste vidjeti:

1. ✅ Stage simulator s robotom
2. ✅ RViz s prikazanom mapom
3. ✅ Laser scan vizualizacije
4. ✅ TF frame-ove
5. ✅ A* putanju planiranu kroz mapu

## Napomene

- Sustav koristi simulacijsko vrijeme (`use_sim_time: true`), što znači da vrijeme dolazi od Stage simulatora
- SLAM Toolbox koristi async mapper za bolju performansu
- A* algoritam podržava 8-smjerno kretanje (uključujući dijagonale)

## Dodatne Resurse

- [SLAM Toolbox Dokumentacija](https://github.com/SteveMacenski/slam_toolbox)
- [ROS2 Stage ROS2 Dokumentacija](https://github.com/ros-simulation/stage_ros2)
- [Nav2 Dokumentacija](https://navigation.ros.org/)

## Pitanja i Probleme

Ako naiđete na probleme, provjerite:

1. Jesu li sve zavisnosti instalirane (`rosdep install`)
2. Je li workspace pravilno kompajliran (`colcon build`)
3. Jesu li svi topici dostupni (`ros2 topic list`)
4. Jesu li TF transformacije pravilno postavljene (`ros2 run tf2_tools view_frames.py`)

---

**Zadnja ažuriranja:** January 4, 2026
