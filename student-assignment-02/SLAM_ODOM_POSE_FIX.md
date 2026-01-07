# 🔧 "Failed to compute odom pose" - FINAL FIX

## Dijagnoza

Topic list izgleda OK:
```
/base_scan ✅ (laser podaci)
/map ✅ (SLAM mapa)
/odom ✅ (odometrija)
/tf ✅ (transformacije)
```

ALI vidiš greške:
```
[WARN] Failed to compute odom pose
[WARN] no map received
```

---

## Problem: TF Frame Mismatch

SLAM toolbox očekuje:
```
odom → base_link → base_scan
```

Ali ti vjerojatno imaš:
```
odom → base_link  (bez base_scan!
```

Ili Stage koristi drugačije frame names.

---

## ✅ RJEŠENJE - 2 Koraka

### Korak 1: Provjeri što Stage koristi

```bash
# Vidi TF tree
ros2 run tf2_tools view_frames.py
cat frames.pdf

# Trebalo bi vidjeti:
# map
#  └── odom
#      └── base_link
#          └── base_scan
```

**Ako TF tree nije ispravan** (npr. base_scan nedostaje), to je problem!

### Korak 2: Ažuriraj mapper_params_online_async.yaml

**Otvorite:** `src/student_assignment_02/config/mapper_params_online_async.yaml`

**Trebalo bi imati:**

```yaml
slam_toolbox:
  ros__parameters:
    # KLJUČNE LINIJE:
    odom_frame: odom              # ← Ova što Stage koristi
    map_frame: map                # ← Ova što SLAM koristi
    base_frame: base_link         # ← Ova što Stage koristi
    scan_topic: /base_scan        # ← Ova što Stage koristi!
    
    # Ostalo ostaje isto...
```

---

## 🚀 Ako TF Tree NEMA base_scan

Tada trebate ažurirati `stage_launch.py`:

**Lokacija:** `src/student_assignment_02/launch/stage_launch.py`

**Promjena:**

```python
# Trebalo bi u robot_state_publisher:
robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time,
        'robot_description': """
<robot name="robot">
  <link name="base_link"/>
  <link name="base_scan"/>
  <link name="odom"/>                              {# ← DODAJTE OVO!
  <joint name="odom_base_link" type="fixed">     {# ← DODAJTE OVO!
    <parent link="odom"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="base_scan_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_scan"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
        """
    }],
    remappings=[...]
)
```

---

## 🔍 DEBUG: Što su Frame Names?

```bash
# Vidi koje frame imena koristi Stage
ros2 topic echo /tf | head -30

# Trebalo bi vidjeti nešto poput:
# - header:
#     frame_id: 'odom'        ← Stage koristi ovo
#   child_frame_id: 'base_link' ← kao parent

# Ili Stage koristi nešto drugo!
```

Ako vidite drugačija imena, trebam znati što!

---

## 📊 Očekivani TF Tree (Ispravan)

```
odom (static publisher)
  └── base_link (Stage publishes)
      └── base_scan (robot_state_publisher)

map (SLAM publishes)
  └── odom
```

**Ako nema `map → odom`**, to je greška! SLAM trebao bi to objaviti.

---

## 🎯 Što se Trebalo Desiti

```
1. Stage startuje
   └── Publishes /tf: odom → base_link
   └── Publishes /base_scan laser

2. robot_state_publisher
   └── Publishes /tf: base_link → base_scan

3. SLAM toolbox
   └── Čita /base_scan
   └── Čita /tf
   └── Publishes /map
   └── Publishes /tf: map → odom

4. RViz
   └── Prikazuje /map (fixed frame = map)
```

---

## ⚠️ Zašto "Failed to compute odom pose"?

**Mogućnosti:**

1. **SLAM ne može pronaći transformacije**
   - Nema `base_link → base_scan` TF-a
   - Nema `/base_scan` topic-a
   - Frame names se ne poklapaju

2. **SLAM čeka /scan umjesto /base_scan**
   - Remapinga nije ispravan

3. **Laser podaci su loši**
   - `/base_scan` je prazan
   - `/base_scan` ima granice (0 do 8.0m je sve)

---

## 📋 Akcijski Plan

- [ ] Provjeri TF tree: `ros2 run tf2_tools view_frames.py`
- [ ] Vidi je li `base_scan` u TF tree-u
- [ ] Ako nema, ažuriraj `robot_state_publisher` URDF
- [ ] Rebuild: `colcon build --symlink-install`
- [ ] Ponovno pokreni: `ros2 launch student_assignment_02 mapping_launch.py`
- [ ] Provjeri greške
- [ ] Javiš što vidim u TF tree-u

---

## 🔗 TF Tree Diagn

Kada pokrenete:

```bash
ros2 run tf2_tools view_frames.py
```

I vidite `frames.pdf` - trebalo bi objaviti što vidite (je li `base_scan` tamo?).

---

**Status:** 🔍 Čekam TF tree informacije

**Sljedeće:** Javite output od `ros2 run tf2_tools view_frames.py`
