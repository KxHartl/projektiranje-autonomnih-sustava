# 🎯 Interactive 2D Goal Pose Navigation - Vodič

## ✨ Što je Novo?

Path Planning Node sada:
- ✅ Čeka 2D Goal Pose iz RViz-a
- ✅ Koristi trenutnu lokaciju robota (iz TF)
- ✅ Računa A* putanju od robota DO cilja
- ✅ Prikazuje putanju u RViz-u u realnom vremenu

---

## 🚀 BRZO POKRETANJE

### Terminal 1: Setup i Build

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build install log
colcon build
source install/setup.bash
```

### Terminal 2: Stage Simulator + SLAM

```bash
ros2 launch student_assignment_02 complete_mapping_launch.py
```

Čekajte dok se mapa ne počne graditi (30+ sekundi)...

### Terminal 3: RViz

```bash
rviz2
```

**Što trebate dodati u RViz:**

1. **Grid** (rviz_default_plugins/Grid)
2. **TF** (rviz_default_plugins/TF) - za transformacije
3. **LaserScan** (rviz_default_plugins/LaserScan)
   - Topic: `/base_scan`
4. **Map** (rviz_default_plugins/Map)
   - Topic: `/map`
5. **Path** (rviz_default_plugins/Path)
   - Topic: `/planned_path` ✅ **ZELENA LINIJA**
6. **MarkerArray** (rviz_default_plugins/MarkerArray)
   - Topic: `/visualization_marker_array`

**Važno:** Postavite `Fixed Frame` na `map`

### Terminal 4: Path Planning Node

```bash
ros2 run student_assignment_02 path_planning_node
```

Trebate vidjeti:
```
Path Planning Node inicijaliziran
Mapa primljena: 200 x 200, rezolucija: 0.05 m/cell
✓ Čekam 2D Goal Pose iz RViz-a...
```

### Terminal 5: Goal Navigation Node (Opciono)

```bash
ros2 run student_assignment_02 goal_navigation_node
```

---

## 🎮 Kako Koristiti 2D Goal Pose

### Korak 1: Postavite Početnu Poziciju (opciono)

U RViz-u:
1. Kliknite na **"2D Pose Estimate"** tool (gornja toolbar)
2. Kliknite na mapu gdje je robot
3. Povucite mišem da postavite pravac robota

**Rezultat:**
```
[path_planning_node]: Initial pose postavljen: (2.50, 5.00, 0.00 rad)
```

### Korak 2: Postavite Cilj

1. Kliknite na **"2D Goal Pose"** tool (gornja toolbar)
2. Kliknite na mapu gdje želite cilj
3. Povucite mišem da postavite pravac cilja

### Korak 3: Gledajte Rezultat

U RViz-u trebate vidjeti:
- 🟢 **Zelena linija** - A* planirana putanja
- 🟢 **Zelena sfera** - Početna pozicija robota
- 🔴 **Crvena sfera** - Ciljna pozicija

U terminalima:
```
=== [Plan 1] ===
Robot pozicija: (2.50 m, 5.00 m)
Goal pozicija: (8.75 m, 12.50 m)
Grid: start (50, 100) -> goal (175, 250)
✓ Putanja pronađena! Duljina: 150 čvorova
A* završen u 2345 iteracija
```

---

## 🔄 Kako Radi

### 1️⃣ Čekanje na Goal Pose

Node čeka `/goal_pose` topic iz RViz-a:
- Koristi 2D Goal Pose tool
- Svaki klik = nova meta

### 2️⃣ Pronalaženje Robot Pozicije

Node automatski čita robot poziciju iz:
- **TF transformacije** `/map` → `/base_link`
- Ako TF nije dostupan: koristi zadnju postavljenu poziciju
- Ako ništa nije dostupno: koristi (0, 0)

### 3️⃣ A* Planiranje

```
Robot pozicija (TF ili zadnja)  ──┐
                                  ├──> A* Algoritam ──> Putanja
Goal pozicija (iz RViz)          ──┘
```

### 4️⃣ Publikovanje Rezultata

- **`/planned_path`** - Putanja u metarskim koordinatama (nav_msgs/Path)
- **`/visualization_marker_array`** - Markeri za RViz

---

## 📊 Što se Zbiva u Pozadini

```cpp
// 1. Goal pose stigla
void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 2. Pročitaj robot poziciju
    update_robot_pose_from_tf();  // Čita /map → /base_link
    
    // 3. Konvertiraj u grid koordinate
    int start_x = world_to_grid_x(robot_x);
    int goal_x = world_to_grid_x(msg->pose.position.x);
    
    // 4. Pokreni A*
    auto path = a_star(start_x, goal_x);
    
    // 5. Publiciraj rezultat
    path_publisher_->publish(path);
}
```

---

## 🔍 Transformacije Koordinata

### World → Grid
```cpp
grid_x = (world_x - origin_x) / resolution
grid_y = (world_y - origin_y) / resolution
```

### Grid → World
```cpp
world_x = (grid_x + 0.5) * resolution + origin_x
world_y = (grid_y + 0.5) * resolution + origin_y
```

**Primjer:**
- Mapa: origem (0, 0), rezolucija 0.05 m/cell
- Grid (100, 200) → World (5.0 m, 10.0 m)

---

## 🛠️ Prilagodbe

### Promjena Default Početne Pozicije

Ako TF nije dostupan, node koristi:
```cpp
robot_x_ = 0.0;  // Promijenite
robot_y_ = 0.0;  // Promijenite
robot_theta_ = 0.0;
```

### Promjena TF Imena

Ako vaš robot koristi drugačije frame-ove:
```cpp
auto transform = tf_buffer_.lookupTransform(
    "map",           // Target frame
    "base_link",     // Source frame (PROMIJENITE AKO TREBATE)
    tf2::TimePointZero
);
```

### Dodavanje Padding-a oko Prepreka

Ako robot "preskače" prepreke, povećajte padding:
```cpp
return current_map_.data[index] < 50;  // Promijenite 50 na veći broj (npr. 60)
```

---

## ❌ Problemi i Rješenja

### Problem: "Mapa još nije primljena"

**Uzrok:** SLAM još nije izgradio mapu  
**Rješenje:** Čekajte 30+ sekundi i pokušajte ponovo postaviti goal

### Problem: Robot pozicija je (0, 0) ili kriva

**Uzrok:** TF transformacija nije dostupna  
**Rješenje:**
1. Postavite početnu poziciju korištenjem 2D Pose Estimate
2. Provjerite da je Stage simulator pokrenut

### Problem: Putanja nije pronađena

**Uzrok:** Nema slobodnog puta između robota i cilja  
**Rješenje:** Odaberite drugi cilj koji je dostupan

### Problem: Putanja prolazi kroz zidove

**Uzrok:** Padding je premali (vrijednost < 50)  
**Rješenje:** Povećajte vrijednost sa 50 na 60-70

---

## 📝 ROS 2 Topics

| Topic | Tip | Opis |
|-------|-----|------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | **INPUT:** 2D Goal Pose iz RViz-a |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | **INPUT:** Početna pozicija |
| `/map` | `nav_msgs/OccupancyGrid` | **INPUT:** Mapa iz SLAM-a |
| `/planned_path` | `nav_msgs/Path` | **OUTPUT:** Planirana A* putanja |
| `/visualization_marker_array` | `visualization_msgs/MarkerArray` | **OUTPUT:** Markeri za RViz |
| `/tf` | `tf2_msgs/TFMessage` | **INPUT:** Transform /map → /base_link |

---

## 📊 A* Performanse

| Parametar | Vrijednost |
|-----------|----------|
| Heuristika | Manhattan distanca |
| Kretanje | 8 smjerova (N, NE, E, ...) |
| Max iteracija | 100,000 |
| Vrijeme izvršavanja | <3 sekunde |
| Memorija | Minimalna (<10MB) |

---

## ✅ Checklist

- [ ] Build je uspješan (`colcon build` bez grešaka)
- [ ] Stage simulator je pokrenut
- [ ] SLAM je izgradio mapu
- [ ] RViz prikazuje `/map` topic
- [ ] RViz prikazuje TF transformacije
- [ ] Path Planning Node je pokrenut
- [ ] Node kaže: "Čekam 2D Goal Pose iz RViz-a..."
- [ ] Možete kliknuti "2D Goal Pose" tool
- [ ] Zelena linija se pojavljuje u RViz-u
- [ ] Logovi pokazuju A* planiranje

---

## 🎬 Što je Sljedeće?

Ako je sve ispravno:
1. Putanja je planirana
2. Pokrenite `goal_navigation_node` u Terminal 5
3. Robot će automatski slijediti putanju
4. Gledajte kako robot ide do cilja! 🤖

---

**Status:** ✅ Interactive 2D goal pose navigation je spreman!

**Zadnja ažuriranja:** 5. siječnja 2026.
