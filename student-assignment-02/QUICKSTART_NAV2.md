# 🚀 Quick Start - Navigacija s Nav2 (5 minuta)

## 1️⃣ Build

```bash
cd ~/ros2_ws
colcon build --packages-select student_assignment_02
source install/setup.bash
```

## 2️⃣ Pokreni u četirima Terminalima

### Terminal 1: Stage Simulator
```bash
ros2 launch student_assignment_02 stage_launch.py
```

### Terminal 2: AMCL Lokalizacija
```bash
ros2 launch student_assignment_02 localization_complete_launch.py
```

**☝️ U RViz-u**: Koristi "Estimate Pose" da postaviš poziciju robota

### Terminal 3: Navigacija s Nav2
```bash
ros2 launch student_assignment_02 navigation_complete_nav2.launch.py
```

RViz se otvara automatski!

### Terminal 4: Monitor (opciono)
```bash
ros2 topic echo /cmd_vel
```

## 3️⃣ Koristi

1. **U RViz-u**: Odaberi **"2D Goal Pose"** tool
2. **Klikni** na mapu gdje želiš da ide robot
3. **Povuci** da postaviš orijentaciju
4. **🚀 Robot se počinje kretati!**

## 🔈 Što se dogada?

```
RViz (2D Goal Pose)
    ↓ /goal_pose
A* Planer
    ↓ /planned_path
Nav2 Adapter
    ↓ FollowPath Action
Nav2 DWB Kontroler
    ↓ /cmd_vel
Robot se kreće!
```

## ❌ Problemi?

| Problem | Rješenje |
|---------|----------|
| "missing required argument 'map'" | Koristi `navigation_complete_nav2.launch.py` ne `navigation_complete_launch.py` |
| Robot se ne pomjera | Lokaliziraj robota sa "Estimate Pose" u RViz-u |
| "A* putanja nije pronađena" | Postavi goal na **slobodnom mjestu** na mapi |
| Robot ide krivo | Smanji brzine u `config/nav2_params.yaml` |

## 💺 Vodič s detaljima

Za više informacija, pogledaj: **[NAVIGATION_GUIDE_NAV2_FINAL.md](./NAVIGATION_GUIDE_NAV2_FINAL.md)**

---

**Status**: ✅ Radi!
