# Ispravka - Entry Points Problem

## Problem

Nakon build-a, `ros2 run` nije mogao pronaći niti jedan čvor.

## Uzrok

1. **Nedostaje `__init__.py`** - Python package direktorij mora sadržavati `__init__.py` datoteku
2. **Nedostaju čvorovi** - `path_planning_node.py` i `goal_navigation_node.py` nisu postojali

## Rješenje (GOTOVO)

✅ Kreiran `__init__.py` u `student_assignment_02/student_assignment_02/`  
✅ Kreirani dummy čvorovi za sve entry points  
✅ Svi čvorovi sada trebaju biti dostupni

## Što Trebate Učiniti

### Korak 1: Clean Build (VAŽNO!)

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build/ install/ log/
colcon build --packages-select student_assignment_02 --symlink-install
source install/setup.bash
```

### Korak 2: Provjera Entry Points

Provjerite da svi čvorovi sada postoje:

```bash
ros2 pkg list | grep student
```

Trebali biste vidjeti:
```
student_assignment_02
```

### Korak 3: Provjerite sve čvorove

```bash
# Trebali biste vidjeti sve 4 čvora:
ros2 run student_assignment_02 a_star_path_planner --help
ros2 run student_assignment_02 map_republisher --help
ros2 run student_assignment_02 path_planning_node --help
ros2 run student_assignment_02 goal_navigation_node --help
```

## Datoteke Koje Su Kreirane

| Datoteka | Opis |
|----------|------|
| `__init__.py` | ✅ NOVO - Obavezna za Python package |
| `path_planning_node.py` | ✅ NOVO - Dummy čvor (možete ga proširiti) |
| `goal_navigation_node.py` | ✅ NOVO - Dummy čvor (možete ga proširiti) |

## Što Trebate Znati

### Python Package Struktura

FOR Python package trebate:
1. **`__init__.py`** - Označava direktorij kao Python package
2. **Python moduli** - `.py` datoteke s kodom
3. **`setup.py`** - S entry_points koji pokazuju na main() funkcije

### ROS 2 Entry Points Format

```python
entry_points={
    'console_scripts': [
        'naziv_noda = package_name.modul_name:main',  # Format
        'a_star_path_planner = student_assignment_02.a_star_path_planner:main',
    ],
},
```

## Testiranje

### Testira jednog čvora:

```bash
ros2 run student_assignment_02 a_star_path_planner
```

Trebali biste vidjeti:
```
[INFO] [a_star_path_planner]: A* Path Planner Node: Started
[INFO] [a_star_path_planner]: Slusa na /goal_pose za dinamicki goal (RViz 2D Goal Pose)
```

Pritisnite `Ctrl+C` za zaustavljanje.

## Ako Još Uvijek Ne Radi

### 1. Provjerite lokaciju datoteka

```bash
ls -la ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/student_assignment_02/
```

Trebali biste vidjeti:
```
__init__.py
a_star_path_planner.py
map_republisher.py
path_planning_node.py
goal_navigation_node.py
```

### 2. Provjerite setup.py

```bash
cat ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/src/student_assignment_02/setup.py | grep -A 10 "entry_points"
```

Trebali biste vidjeti sve 4 entry points.

### 3. Provjerite install direktorij

```bash
ls -la ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/install/student_assignment_02/lib/student_assignment_02/
```

Trebali biste vidjeti sve `.py` datoteke.

## Struktura Direktorija (ISPRAVLJENA)

```
student-assignment-02/
├── src/
│   └── student_assignment_02/
│       ├── student_assignment_02/
│       │   ├── __init__.py                    ✅ NOVO
│       │   ├── a_star_path_planner.py         ✅ EXISTING
│       │   ├── map_republisher.py             ✅ EXISTING
│       │   ├── path_planning_node.py          ✅ NOVO
│       │   └── goal_navigation_node.py        ✅ NOVO
│       ├── launch/
│       │   └── a_star_path_planner.launch.py
│       ├── config/
│       │   └── ...
│       ├── setup.py                           ✅ OK
│       └── package.xml
├── FIX_INSTRUCTIONS.md                        (Ovaj fajl)
└── ...
```

## Zašto Je To Bilo Potrebno?

ROS 2 koristi Python setuptools za instalaciju paketa. Setuptools traži:
1. Python package direktorij (s `__init__.py`)
2. Entry points u `setup.py`
3. Validne Python moduli s `main()` funkcijom

Bez `__init__.py`, setuptools ne prepoznaje direktorij kao package, pa entry points ne mogu biti instalirani.

## Dalje Korake

1. **Testirajte sve čvorove**
2. **Pokrenite A* path planner** s mapom iz Stage simulatora
3. **Postavite goal iz RViza** s 2D Goal Pose tool-om

---

**Status**: ✅ FIXANO

Svi čvorovi trebaju biti dostupni nakon rebuild-a s `--symlink-install` opcijom!
