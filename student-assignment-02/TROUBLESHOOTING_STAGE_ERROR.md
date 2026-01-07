# 🔧 TROUBLESHOOTING: Stage Simulator Error

## Problem

```
[stage_ros2-1] err: unable to open include file /home/khartl/FSB/projektiranje-autonomnih-sustava/student-assignment-02/install/student_assignment_02/share/student_assignment_02/world/include/robots.inc : No such file or directory
[stage_ros2-1] err:  Failed to open file /home/khartl/FSB/projektiranje-autonomnih-sustava/student-assignment-02/install/student_assignment_02/share/student_assignment_02/world/map_01.world
```

## Uzrok

Stage simulator traži datoteke u `install/` direktoriju, ali one se ne kopiraju tijekom build-a jer **`setup.py` nije pravilno konfiguriran**.

**Ažurirano:** `setup.py` sada uključuje sve potrebne datoteke.

---

## ✅ RJEŠENJE

### Korak 1: Obriši stare build datoteke

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build install log
```

### Korak 2: Rebuild s novim setup.py

```bash
colcon build --symlink-install
```

**VAŽNO:** Flag `--symlink-install` omogućava da file changes ne trebaju rebuild!

### Korak 3: Verificiraj da su datoteke kopirane

```bash
# Provjera da su .inc datoteke kopirane
ls -la install/student_assignment_02/share/student_assignment_02/world/include/

# Trebalo bi vidjeti:
# robots.inc
# pioneer2dx.inc
```

**Expected output:**
```
total 12
drwxr-xr-x 2 khartl khartl 4096 Jan  7 12:30 .
drwxr-xr-x 4 khartl khartl 4096 Jan  7 12:30 ..
-rw-r--r-- 1 khartl khartl 1523 Jan  7 12:30 robots.inc
-rw-r--r-- 1 khartl khartl  892 Jan  7 12:30 pioneer2dx.inc
```

### Korak 4: Provjera world datoteka

```bash
ls -la install/student_assignment_02/share/student_assignment_02/world/

# Trebalo bi vidjeti:
# map_01.world
# map_01.png
# include/ (direktorij)
```

### Korak 5: Pokreni ponovno

```bash
source install/setup.bash
ros2 launch student_assignment_02 mapping_launch.py
```

---

## 🔍 Što se promijenilo u setup.py

**Prije (POGREŠNO):**
```python
data_files=[
    ('share/' + package_name + '/world',
        glob('world/*.world')),  # Samo .world datoteke!
]
```

**Sada (ISPRAVNO):**
```python
data_files=[
    ('share/' + package_name + '/world',
        glob('world/*.world')),  # .world datoteke
    ('share/' + package_name + '/world',
        glob('world/*.png')),    # PNG slike
    ('share/' + package_name + '/world/include',
        glob('world/include/*.inc')),  # Include datoteke s robot definicijama
]
```

---

## 📁 Stage World Struktura

```
student_assignment_02/world/
├── map_01.world                    # Stage world file
├── map_01.png                      # World bitmap
└── include/
    ├── robots.inc                  # Pioneer2DX s laserskim skenerom
    └── pioneer2dx.inc              # Generička robot definicija
```

**Kako to radi:**
1. `map_01.world` koristi `include "include/robots.inc"`
2. Stage simulator traži `robots.inc` u istom direktoriju
3. Trebam ga pronaći u `include/` subdirektoriju
4. `setup.py` mora kopirati sve datoteke!

---

## 🧪 Verifikacijski Test

Ako je sve dobro, trebali bi vidjeti:

```bash
# Terminal 1
rviz2 -d config/rviz_config.rviz

# Terminal 2
ros2 launch student_assignment_02 mapping_launch.py
```

**Output bi trebao biti:**
```
[INFO] [launch]: All log files can be found below /home/khartl/.ros/log/...
[INFO] [stage_ros2-1]: process started with pid [XXXX]
[INFO] [robot_state_publisher-2]: process started with pid [XXXX]
[INFO] [rviz2-4]: process started with pid [XXXX]
[static_transform_publisher-3] [INFO] [...]: Spinning until stopped - publishing transform
```

**NE bi trebao biti error:**
```
err: unable to open include file
err: Failed to open file
```

---

## 🛑 Ako problem još uvijek postoji

### Debug korak-po-korak

```bash
# 1. Gdje je Stage traži datoteke?
grep -r "world_file" install/

# 2. Koji file path Stage koristi?
ros2 launch student_assignment_02 mapping_launch.py 2>&1 | grep -i "world\|error\|err"

# 3. Provjera TF-a
ros2 run tf2_tools view_frames.py

# 4. Provjera topic-a
ros2 topic list
```

### Alternativno: Koristi izvorni world path

Ako `install/` dalje ima problema, možete koristiti izvorni path:

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
ros2 launch student_assignment_02 mapping_launch.py world_file:=$(pwd)/src/student_assignment_02/world/map_01.world
```

Ali **ovo nije best practice** - trebate koristiti installirane datoteke.

---

## ✅ Checklist za Rješavanje

- [ ] Obrisao `build/`, `install/`, `log/` direktorije
- [ ] `setup.py` je ažuriran s `.inc` i `.png` datotekama
- [ ] Rebuild s `colcon build --symlink-install`
- [ ] `install/student_assignment_02/share/student_assignment_02/world/include/robots.inc` postoji
- [ ] `install/student_assignment_02/share/student_assignment_02/world/map_01.world` postoji
- [ ] `install/student_assignment_02/share/student_assignment_02/world/map_01.png` postoji
- [ ] `source install/setup.bash` izvršen
- [ ] `ros2 launch student_assignment_02 mapping_launch.py` radi bez error-a

---

**Ako ne radi**, pokrenite sa debug flagom:

```bash
ros2 launch student_assignment_02 mapping_launch.py --debug
```

Ili pregledajte log datoteke:

```bash
cat ~/.ros/log/latest/stage_ros2-*.log
```

---

## 📞 Dodatne Naredbe

```bash
# Popis svih datoteka u instaliranom paketu
ls -R install/student_assignment_02/share/

# Provjera setup.py konfiguracije
python3 -c "from setuptools import setup; import setup as s; print(s.data_files)"

# Detaljni build output
colcon build --verbose

# Clean build
colcon build --force-cmake-configure
```

---

**Status:** ✅ FIXED - setup.py je ažuriran

**Commit:** a57fe9c2ef2483b827f439fd95675221efd41c49

**Ažurirано:** 7. Siječnja 2026., 12:24 CET
