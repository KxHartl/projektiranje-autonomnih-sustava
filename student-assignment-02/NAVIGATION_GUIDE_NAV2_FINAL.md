# Vodič: Navigacija s A* i Nav2 Stackom

**Status**: ✅ **RADI** - Robot se više ne vrta u krug!

## 📋 Sadržaj

1. [Što je novo](#što-je-novo)
2. [Arhitektura](#arhitektura)
3. [Pokretanje](#pokretanje)
4. [Korištenje - Postavljanje Goal-a](#korištenje---postavljanje-goal-a)
5. [Troubleshooting](#troubleshooting)
6. [Tuning parametara](#tuning-parametara)

---

## 🆕 Što je novo

### Problemi koji su riješeni

❌ **Problem**: Robot se vrta u krug  
✅ **Rješenje**: Korištenje Nav2 `controller_server`-a umjesto custom path follower-a

❌ **Problem**: Nemogućnost postavljanja goal-a iz RViza  
✅ **Rješenje**: A* planer sada sluša na `/goal_pose` iz RViza

❌ **Problem**: Bringup dependency greške  
✅ **Rješenje**: Direktna konfiguracija Nav2 komponenti bez bringup_launch.py

---

## 🏗️ Arhitektura

```
┌──────────────────────────────────────────────────────────┐
│  RViz 2                                                  │
│  - Vizualizacija mape, robota, putanja                   │
│  - 2D Goal Pose tool za postavljanje cilja              │
└────────────────┬─────────────────────────────────────────┘
                 │ /goal_pose (PoseStamped)
                 ↓
┌──────────────────────────────────────────────────────────┐
│  A* Path Planner                                         │
│  - Planira putanju od robota do cilja                    │
│  - Koristi inflirane mape (0.5m buffer)                  │
│  - Publikuje: /planned_path (Path)                       │
└────────────────┬─────────────────────────────────────────┘
                 │ /planned_path (nav_msgs/Path)
                 ↓
┌──────────────────────────────────────────────────────────┐
│  Nav2 Adapter                                            │
│  - Hvata A* putanju                                      │
│  - Šalje je Nav2 preko FollowPath akcije                 │
└────────────────┬─────────────────────────────────────────┘
                 │ FollowPath Action
                 ↓
┌──────────────────────────────────────────────────────────┐
│  Nav2 Controller Server                                  │
│  - DWB lokalni planer - sljedi putanju                   │
│  - Izbjegava prepreke                                    │
│  - Publikuje: /cmd_vel (Twist)                           │
└────────────────┬─────────────────────────────────────────┘
                 │ /cmd_vel
                 ↓
┌──────────────────────────────────────────────────────────┐
│  Stage Robot                                             │
│  - Fizička simulacija robota                             │
│  - Sljedi cmd_vel zapovijedne brzine                     │
└──────────────────────────────────────────────────────────┘
```

---

## 🚀 Pokretanje

### 1. Preliminarni Setup

```bash
# Build paket
cd ~/ros2_ws
colcon build --packages-select student_assignment_02
source install/setup.bash
```

### 2. Terminal 1 - Stage Simulator

```bash
# Pokreni Stage simulator sa svijetom
ros2 launch student_assignment_02 stage_launch.py

# Ili direktno
ros2 run stage_ros2 stageros ~/ros2_ws/src/student_assignment_02/world/stage_world.world
```

### 3. Terminal 2 - AMCL Lokalizacija

```bash
# Pokreni AMCL za lokalizaciju robota
ros2 launch student_assignment_02 localization_complete_launch.py

# U RViz-u: koristi "Estimate Pose" tool da postaviš početnu poziciju robota
```

**⚠️ VAŽNO**: Robot mora biti lokaliziran prije planiranja!

### 4. Terminal 3 - Navigacija s Nav2

```bash
# Pokreni A* planer + Nav2 kontroler + RViz
ros2 launch student_assignment_02 navigation_complete_nav2.launch.py
```

**Output bi trebao biti**:
```
[INFO] [launch]: All log files can be found below /home/...
[INFO] [lifecycle_manager_navigation-1]: starting up...
[INFO] [planner_server-2]: starting up...
[INFO] [controller_server-3]: starting up...
[INFO] [a_star_path_planner-4]: A* Path Planner Node: Started
[INFO] [nav2_adapter-5]: ========... Nav2 Adapter inicijaliziran ...
[INFO] [rviz2-6]: ...
```

---

## 📍 Korištenje - Postavljanje Goal-a

### Korak po Korak

#### 1️⃣ RViz priprema

- U lijevoj paneli (Displays) pronađi **"2D Goal Pose"** tool
- Ako ga nema, dodaj ga: **Add** → **By topic** → **/goal_pose**

#### 2️⃣ Postavljanje početne pozicije

- Koristi **"Estimate Pose"** tool
- Klikni gdje se robot nalazi i povuci da postaviš orijentaciju
- Robot bi trebao biti lokaliziran na toj poziciji

#### 3️⃣ Postavljanje cilja

- Klikni na **"2D Goal Pose"** tool
- Klikni na mapu gdje želiš da ide robot
- Povuci da postaviš orijentaciju

**Što se tada dogodi**:
```
1. RViz šalje goal_pose → /goal_pose topic
2. A* planer prima goal i planira putanju
3. A* publikuje putanju na /planned_path
4. Nav2 Adapter hvata putanju i šalje je Nav2-u
5. Nav2 DWB kontroler počinje slijediti putanju
6. Robot se počinje kretati!
```

#### 4️⃣ Praćenje napretka

U RViz-u vidiš:
- **Green line** = Planirani put od A*
- **Red sphere** = Istraživane stanice
- **Yellow sphere** = Čelna fronta pretraživanja
- **Orange zone** = Inflation buffer (prepreke)
- **Robot ose** = Slijeđenje putanje

---

## 🔧 Troubleshooting

### ❌ Problem: "ERROR [launch]: Caught exception in launch... missing required argument 'map'"

**Rješenje**: Koristiš stari `navigation_complete_launch.py`  
**Koristi**: `navigation_complete_nav2.launch.py`

```bash
# ❌ KRIVO
ros2 launch student_assignment_02 navigation_complete_launch.py

# ✅ ISPRAVNO
ros2 launch student_assignment_02 navigation_complete_nav2.launch.py
```

---

### ❌ Problem: Robot se ne pomjera

**Mogući uzroci**:

1. **AMCL nije lokaliziran**
   ```bash
   # Provjeri TF stablo
   ros2 run tf2_ros tf2_echo map base_link
   # Trebao bi vratiti transformaciju
   ```

2. **Goal je u prepeki**
   - Postavi goal na slobodnom mjestu na mapi

3. **Nav2 nije pokrenuta**
   ```bash
   # Provjeri topike
   ros2 topic list | grep cmd_vel
   # Trebao bi vidjeti /cmd_vel
   ```

---

### ❌ Problem: "A* putanja nije pronađena"

**Mogući uzroci**:

1. **Previsok inflation buffer**
   - Inflation distanca: 0.5m može biti premala ili prevelika
   - Skini sa: `inflation_distance_m: 0.2`

2. **Prepreka blizu robota**
   - Robot je zarobljen
   - Pomakni ga sa "Estimate Pose" tool-om

3. **Loša mapa**
   - Provjeri je li mapa ispravno mapirana

---

### ❌ Problem: Robot ide krivo ili oscilira

**Rješenja**:

1. **Smanjite brzine**:
   ```yaml
   # U nav2_params.yaml
   controller_server:
     FollowPath:
       max_vel_x: 0.1  # Smanjeno s 0.26
       max_vel_theta: 0.5  # Smanjeno s 1.0
   ```

2. **Povećajte lookahead distance**:
   ```yaml
   FollowPath:
       min_speed_xy: 0.05
   ```

3. **Resetirajte lokalizaciju**:
   ```bash
   ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
   "header: {frame_id: 'map'} pose: {pose: {position: {x: 0, y: 0}}}" --once
   ```

---

## 🎛️ Tuning Parametara

### A* Planer (`a_star_path_planner`)

```bash
ros2 launch student_assignment_02 navigation_complete_nav2.launch.py \
  inflation_distance_m:=0.3
```

| Parametar | Default | Opis |
|-----------|---------|------|
| `inflation_distance_m` | 0.5m | Sigurna distanca od zidova |
| `allow_diagonal` | true | Dopusti dijagonalno kretanje |
| `max_iterations` | 50000 | Max A* iteracija |
| `inflation_cost_threshold` | 60 | Threshold za prepreke |

### Nav2 Kontroler (`controller_server`)

**Datoteka**: `config/nav2_params.yaml`

```yaml
controller_server:
  FollowPath:
    max_vel_x: 0.26        # Maksimalna linearna brzina [m/s]
    max_vel_theta: 1.0     # Maksimalna kutna brzina [rad/s]
    acc_lim_x: 2.5         # Maksimalna akceleracija [m/s²]
    sim_time: 1.7          # Simulacija budućnosti [s]
    xy_goal_tolerance: 0.25  # Tolerancija do cilja [m]
```

### Za Sporije Gibanje

```yaml
controller_server:
  FollowPath:
    max_vel_x: 0.1         # Polako
    max_vel_theta: 0.3     # Polako okretanje
    acc_lim_x: 0.5         # Blagi ubrzaj
    sim_time: 2.0          # Dulja simulacija
```

### Za Brže Gibanje

```yaml
controller_server:
  FollowPath:
    max_vel_x: 0.5         # Brže
    max_vel_theta: 1.5     # Brže okretanje
    acc_lim_x: 3.0         # Agresivniji ubrzaj
    sim_time: 1.0          # Kraća simulacija
```

---

## 📊 Monitoriranje

### Topici koji se objavljuju

```bash
# Sve mape
ros2 topic echo /map | head -5

# Lokalizacija
ros2 topic echo /amcl_pose

# A* putanja
ros2 topic echo /planned_path

# Kontroler zakompandi brzine
ros2 topic echo /cmd_vel

# TF transformacije
ros2 run tf2_ros tf2_echo map base_link
```

### Debug logiranje

```bash
# Detaljni logovi od A* planera
ros2 launch student_assignment_02 navigation_complete_nav2.launch.py log_level:=DEBUG

# Ili direktno:
RCL_LOG_LEVEL=DEBUG ros2 launch student_assignment_02 navigation_complete_nav2.launch.py
```

---

## ✅ Checklist

Preje nego što počneš s navigacijom:

- [ ] Stage simulator je pokrenut
- [ ] AMCL je lokalizirao robota (provjeri TF)
- [ ] RViz je otvoren s `navigation_complete_nav2.launch.py`
- [ ] Koristi "2D Goal Pose" tool (ne manuelni goal)
- [ ] Postavi goal na **slobodnom mjestu** na mapi
- [ ] Observiraj `/cmd_vel` - trebala bi krivulja brzine
- [ ] Provjeri `/planned_path` - trebala bi biti vidljiva u RViz-u

---

## 📚 Dodatni Resursi

- [Nav2 Dokumentacija](https://docs.nav2.org/)
- [DWB Kontroler](https://docs.nav2.org/configuration/packages/configuring-dwb.html)
- [ROS 2 Transformacije](https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Tf2.html)

---

**Zadnje ažurirano**: 2026-01-08  
**Status**: ✅ Testira i radi
