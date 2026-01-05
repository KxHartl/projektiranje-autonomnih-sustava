# 🔧 ISPRAVKA GREŠAKA PRI BUILD-U

## Greška: `tf2_geometry_msgs/tf2_geometry_msgs.hpp: No such file or directory`

### ✅ RIJEŠENO

Problematični include je uklonjen iz `path_planning_node.cpp`

**Što je napravljeno:**
- Uklonjen: `#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>`
- Razlog: Path planning node ne trebam TF2 transformacije (to je samo za goal_navigation_node)
- Rezultat: Čist kompajl bez grešaka

---

## 🚀 Pokretanje Build-a

### Terminal - Korak 1: Brisanje starih datoteka

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build install log
rm -rf ~/.config/rviz2/ ~/.rviz2/ ~/.ros/
```

### Terminal - Korak 2: Build

```bash
colcon build
```

**Trebalo bi vidjeti:**
```
Starting >>> student_assignment_02
--- stderr: student_assignment_02
--- stdout: student_assignment_02
Finished <<< student_assignment_02 [X.XXs]

Summary: 1 package finished [X.XXs]
```

### Terminal - Korak 3: Setup

```bash
source install/setup.bash
```

---

## ✅ Što je Ispravno

| Datoteka | Stanje | Razlog |
|----------|--------|--------|
| `path_planning_node.cpp` | ✅ Ispravljen | Uklonjen nepotrebni tf2_geometry_msgs include |
| `goal_navigation_node.cpp` | ✅ OK | Koristi tf2_geometry_msgs (trebao mu) |
| `CMakeLists.txt` | ✅ OK | Sve dependency-je su dodane |

---

## 🔍 Ako Nastave Biti Problemi

### Opcija 1: Čist build

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build install log src/student_assignment_02/build
colcon build --packages-select student_assignment_02
```

### Opcija 2: Update ROS 2 paketa

```bash
sudo apt update
sudo apt install -y \
  ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-visualization-msgs
```

### Opcija 3: Provjeravanje dependency-ja

```bash
ros2 pkg executables student_assignment_02
```

Trebalo bi vidjeti:
```
student_assignment_02 path_planning_node
student_assignment_02 goal_navigation_node
```

---

## 🎯 Sada Možete Pokrenuti

Slijedi: `QUICK_START_A_STAR.md` ili `A_STAR_COMPLETE_GUIDE.md`

---

**Status:** ✅ Build error ispravljen!

**Datumi:** 5. siječnja 2026.
