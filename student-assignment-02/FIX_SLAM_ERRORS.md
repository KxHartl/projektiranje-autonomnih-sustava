# 🔧 ISPRAVI SLAM GREŠKE - Akcijski Plan

## Greške koje ste Vidjeli

```
[ERROR] Failed to compute odom pose
[ERROR] no map received
```

---

## ✅ BRZO RJEŠENJE (3 koraka)

### Korak 1: Obriši Build Datoteke

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build install log
```

### Korak 2: Rebuild s Novim Konfiguracijom

```bash
colcon build --symlink-install
source install/setup.bash
```

**VAŽNO:** `--symlink-install` omogućava brze izmjene bez rebuild-a!

### Korak 3: Pokrenite Mapping

```bash
ros2 launch student_assignment_02 mapping_launch.py
```

---

## 🔍 Što se Promijenilo

Ažurirao sam 3 datoteke:

### 1. **setup.py** - Sad kopira sve world datoteke
```python
# Sada uključuje:
('share/' + package_name + '/world/include', glob('world/include/*.inc'))
```

### 2. **stage_launch.py** - Poboljšani TF
```python
# Ispravke:
- 'use_static_transformations': False  # Dinamičke TF
- 'publish_ground_truth': True         # Za debugging
- arguments=['0', '0', '0', '0', '0', '0', '1', ...]  # Proper quaternion
```

### 3. **online_async_launch.py** - Bolji SLAM setup
```python
# Dodao:
- emulate_tty=True  # Bolji output
- Pravilne remappings za /scan → /base_scan
```

---

## 🔍 Verifikacijski Test

Kako provjeriti da li sada radi?

```bash
# Terminal 1: RViz
rviz2 -d config/rviz_config.rviz

# Terminal 2: Mapping Launch
ros2 launch student_assignment_02 mapping_launch.py

# Terminal 3: Provjera Topic-a
ros2 topic list
```

**Trebalo bi vidjeti:**

✅ `/base_scan` - Laser podaci
✅ `/map` - SLAM mapa
✅ `/odom` - Odometrija
✅ `/tf` - Transformacije

**Trebali bi vidjeti u RViz-u:**

✅ Robot u mapi
✅ Laser zrake (crvene linije)
✅ Gradeća se mapa (siva polja)

---

## 🧪 Ako Problem Opet Postoji

### Problem: "/base_scan" ne postoji

```bash
# Provjera Stage laser konfiguracije
grep -i "laser" src/student_assignment_02/world/include/robots.inc

# Trebalo bi vidjeti laser blok
```

Ako nema laser-a, javi!

### Problem: TF transformacije su prazne

```bash
# Provjera TF-a
ros2 topic echo /tf | head -20

# Trebalo bi vidjeti:
# - transforms: [...]
```

Ako je prazno, javi!

---

## 💡 Što se trebalo Desiti

```
1. Stage pokreće robot u simulaciji
   ↓
2. Robot emitira /base_scan laser podatke
   ↓
3. SLAM toolbox prima /base_scan
   ↓
4. SLAM počinje mapirati
   ↓
5. /map topic se pojavljuje
   ↓
6. RViz prikazuje mapu
   ↓
7. Mapa se živo gradi kako robot ide
```

---

## 🌟 Kako Dalje?

Kada je mapiranje gotovo (vidite dobru mapu u RViz-u):

```bash
# Terminal 4: Spremi mapu
mkdir -p ~/my_map
cd ~/my_map
ros2 run nav2_map_server map_saver_cli --fmt pgm -f map
```

Tada možete testirati **Path Planning & Navigaciju**!

---

## 📁 Commit History

- **a57fe9c**: setup.py - Kopira world/include/*.inc datoteke
- **341231d**: TROUBLESHOOTING_STAGE_ERROR.md
- **93d83ec**: SLAM_MAPPING_DEBUG.md
- **da41a5b**: stage_launch.py - Poboljšani TF
- **55fa5b1**: online_async_launch.py - Bolji SLAM

---

## ✅ Checklist

- [ ] `rm -rf build install log`
- [ ] `colcon build --symlink-install`
- [ ] `source install/setup.bash`
- [ ] `ros2 launch student_assignment_02 mapping_launch.py`
- [ ] Vidim robot u RViz-u
- [ ] Vidim laser zrake
- [ ] Vidim /map topic
- [ ] Mapa se gradi

---

**Status:** ✅ FIXED

**Što Trebate Učiniti:** Izvršite 3 koraka gore

**Ako Problem Persists:** Javite output od `ros2 topic list`
