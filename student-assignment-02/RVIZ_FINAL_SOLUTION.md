# RViz - Konačno Rješenje

## 🔴 Problem

RViz i dalje učitava **staru cached konfiguraciju** čak i nakon:
- Brisanja `~/.ros`, `~/.rviz2`, `~/.cache/rviz2`
- Rebuild-a paketa
- Nove konfiguracije

## 🔍 Uzrok

RViz sprema konfiguraciju **na više mjesta**:
```
~/.config/rviz2/          ← OVDJE sprema "default"
~/.rviz2/                 ← OVDJE je cache
~/.ros/                   ← OVDJE je build cache
/tmp/launch_params_*      ← OVDJE su temp parametri
```

Kada pokrenete s `-d config/rviz_config.rviz`, RViz ionako učitava **default** konfiguraciju iz `~/.config/rviz2/`

## ✅ KONAČNO RJEŠENJE

### Korak 1: Potpuno Očistite Sve RViz Podatke

```bash
# Obriši sve RViz cache i config
rm -rf ~/.config/rviz2/
rm -rf ~/.rviz2/
rm -rf ~/.ros/
rm -rf /tmp/launch_params_*
rm -rf ~/.cache/rviz2/
```

### Korak 2: Pokrenite RViz BEZ Konfiguracije

```bash
# Terminal 1: Pokrenite stage_launch samo (bez RViz-a)
ros2 launch student_assignment_02 stage_launch.py stage:=true rviz:=false
```

U **drugom terminalu**, pokrenite RViz bez default konfiguracije:

```bash
# Terminal 2: RViz s eksplicitnom konfiguracijom
rviz2 -d ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/install/student_assignment_02/share/student_assignment_02/config/rviz_config.rviz
```

### Korak 3: Ili Pokrenite s `--no-config`

Ako gornje ne radi:

```bash
# Terminal 2: RViz bez konfiguracije
rviz2 --ros-args --log-level error --no-config
```

Zatim u RViz-u ručno dodajte displaye:
1. `Panels` → `Add New Panel` → `Displays`
2. `Add` → `Grid` (rviz_default_plugins/Grid)
3. `Add` → `TF` (rviz_default_plugins/TF)
4. `Add` → `LaserScan` (rviz_default_plugins/LaserScan)
5. `Add` → `Map` (rviz_default_plugins/Map)
6. `Add` → `Marker Array` (rviz_default_plugins/Marker)

Zatim: `File` → `Save Config As...` → spremi kao `my_rviz.rviz`

## ALTERNATIVA - Koristi Samo Stage Launch

Ako je problem u `complete_mapping_launch.py` koja pokušava pokrenuti RViz:

```bash
# Terminal 1: Stage, TF, Robot State Publisher
ros2 launch student_assignment_02 stage_launch.py stage:=true rviz:=false

# Terminal 2: SLAM
ros2 launch student_assignment_02 online_async_launch.py use_sim_time:=true

# Terminal 3: RViz (bez konfiguracije)
rviz2 --ros-args --log-level error --no-config
```

## Što se Događa

RViz ima **dva sustava konfiguracije**:

1. **Default Config** (`~/.config/rviz2/default.rviz`)
   - Sprema se prvi put kada pokrenete RViz
   - Sadrži STARE plugine (rviz_common/...)
   - RViz automatski učitava ovo

2. **Custom Config** (`-d` opcija)
   - Trebala bi biti opcija, ali RViz još učitava default nakon toga

## Ako NIŠTA Ne Radi

Koristite **completely clean** RViz bez starog build direktorija:

```bash
# KOMPLETAN RESET
cd ~/FSB/projektiranje-autonomnih-sustava

# Obriši sve
rm -rf student-assignment-02/build
rm -rf student-assignment-02/install
rm -rf student-assignment-02/log
rm -rf ~/.config/rviz2/
rm -rf ~/.rviz2/
rm -rf ~/.ros/

# Ponovno build
cd student-assignment-02
colcon build
source install/setup.bash

# Pokrenite samo Stage
ros2 launch student_assignment_02 stage_launch.py stage:=true rviz:=false

# U drugom terminalu, RViz s eksplicitnom putanjom
rviz2 -d $(pwd)/install/student_assignment_02/share/student_assignment_02/config/rviz_config.rviz
```

## Što Trebate Vidjeti

✅ RViz se otvara  
✅ Grid pozadina vidljiva  
✅ TF transformacije vidljive  
✅ LaserScan točke vidljive  
✅ Nema CRVENIH grešaka  

## Problem sa Path Planning Nodea

Ako vidite grešku:
```
rclcpp::exceptions::ParameterAlreadyDeclaredException
parameter 'use_sim_time' has already been declared
```

To je već **ispravljeno** u novoj verziji `path_planning_node.cpp`:
- Uklonjena `declare_parameter` za `use_sim_time`
- Node sada samo čita parametar ako postoji

Rebuild paket:
```bash
colcon build --packages-select student_assignment_02
```

---

**Datum:** 4. siječnja 2026.
**Status:** Konačno rješenje s više opcija
