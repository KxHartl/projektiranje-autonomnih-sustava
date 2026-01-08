# 🚀 Quick Start - Navigacija s Nav2 (4 Terminala)

## 1️⃣ Build

```bash
cd ~/ros2_ws
colcon build --packages-select student_assignment_02
source install/setup.bash
```

## 2️⃣ Pokreni u Odvojenim Terminalima

### Terminal 1: Stage Simulator
```bash
ros2 launch student_assignment_02 stage_launch.py
```
Àekaj dok se simulator pokrene...

### Terminal 2: AMCL Lokalizacija
```bash
ros2 launch student_assignment_02 localization_complete_launch.py
```

**VAŽNO**: U RViz-u koristi "**Estimate Pose**" tool da lokaliziraš robota!

### Terminal 3: RViz Vizualizacija
```bash
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/student_assignment_02/config/rviz_navigation.rviz
```

Ili otvori manu RViz konfiguraciju sa localhost.

### Terminal 4: Navigacija (A* + Nav2) ⭐
```bash
ros2 launch student_assignment_02 navigation_complete_nav2.launch.py
```

**Trebao bi vidjeti**:
```
[INFO] [a_star_path_planner-1]: A* Path Planner Node: Started
[INFO] [nav2_adapter-5]: ============... Nav2 Adapter inicijaliziran ...
[INFO] [lifecycle_manager-1]: Activating Navigation manager
```

## 3️⃣ Koristi - Postavi Goal

### Korak po Korak:

1. **U RViz-u**: Odaberi **"2D Goal Pose"** tool (lijeva panel)
2. **Klikni** na mapu gdje želiš da ide robot
3. **Povuci** miš da postaviš orijentaciju robota
4. **Pusti** tipku - GOTOVO!

### 💡 Što se Dogada:

```
RViz (2D Goal Pose tool)
    ↓ /goal_pose
A* Planer (računa od /base_link do cilja SVAKI PUT)
    ↓ /planned_path
Nav2 Adapter (hvata putanju)
    ↓ FollowPath akcija
Nav2 DWB Controller (sljedi putanju)
    ↓ /cmd_vel
Robot se KREĆE! 🚀
```

## 🔈 Za Novi Cilj

1. **Ponovi Korak 3** - Koristi "2D Goal Pose" tool opet
2. A* **automatski** re-planira od nove pozicije robota
3. Nav2 **automatski** sljedi novu putanju

**NE trebaju nove komande!** 🎆

---

## ❌ Problemi & Rješenja

| Problem | Rješenje |
|---------|----------|
| "A* putanja nije pronađena" | Postavi cilj na **slobodno** mjestu, daleko od prepreka |
| Robot se ne kreće | Lokaliziraj robota sa "Estimate Pose" u RViz-u |
| "Nav2 FollowPath server nije dostupan" | Čekaj 2-3 sekunde da se Nav2 inicijalizira |
| Adapter ne hvata putanju | Provjeri: `ros2 topic echo /planned_path` |
| A* planer se ne vidi | Provjerite je li pokrenuta u Terminalu 4 |

---

## 📋 Vodiči

- **Kratko**: Ovaj file (5 minuta)
- **Detaljno**: [FIX_NAVIGATION.md](./FIX_NAVIGATION.md) (ispravke)
- **Kompletno**: [NAVIGATION_GUIDE_NAV2_FINAL.md](./NAVIGATION_GUIDE_NAV2_FINAL.md) (sve opcije)

---

**Status**: ✅ Radi!  
**Terminali**: 4 (Stage, AMCL, RViz, Navigacija)
