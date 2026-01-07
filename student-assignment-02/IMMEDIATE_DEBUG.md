# 🚠 IMMEDIATE DEBUG - Izvršite OVO Sada

## Problem

Topic list je OK, ali SLAM javlja:
```
[WARN] Failed to compute odom pose
[WARN] no map received
```

---

## 🔍 Što Trebam Znati

Pokrenite ove naredbe **tijekom pokretanja** `mapping_launch.py`:

### 1. TF Tree

```bash
ros2 run tf2_tools view_frames.py
cat frames.pdf | head -30  # Ili otvorite sa oknom
```

**Trebalo bi vidjeti nešto poput:**
```
map
 └─ odom
     └─ base_link
         └─ base_scan

ILI

odom
 └─ base_link
     └─ base_scan
```

**Javite što vidite!**

---

### 2. Laser Podaci

```bash
ros2 topic echo /base_scan | head -10
```

**Trebalo bi vidjeti:**
```
header:
  seq: 123
  stamp:
    sec: 1234567890
  frame_id: 'base_scan'           <--- KEY!
angle_min: -3.14159...
angle_max: 3.14159...
ranges: [8.0, 8.0, 7.95, 7.90, ...]  <--- Laser readings
```

**Ako je frame_id nešto drugačije (npr. 'laser' ili 'robot_scan'), to je problem!**

Javite: `frame_id: ???`

---

### 3. TF Transformacije

```bash
ros2 topic echo /tf | head -50
```

**Trebalo bi vidjeti listu transformacija poput:**
```
- header:
    frame_id: 'odom'
  child_frame_id: 'base_link'
  transform: ...

- header:
    frame_id: 'base_link'
  child_frame_id: 'base_scan'
  transform: ...
```

**Javite sve `frame_id` i `child_frame_id` kombinacije koje vidite!**

---

### 4. SLAM Log

```bash
# Terminal gdje je pokrenuta mapping_launch.py - pogledajte RED linije:
ros2 launch student_assignment_02 mapping_launch.py 2>&1 | grep -i "error\|warn\|frame"
```

**Trebalo bi vidjeti nešto poput:**
```
[INFO] [...]: Odometry pose transformation
[WARN] Failed to compute odom pose
[WARN] Message Filter dropping message: frame 'base_scan' at time ...
```

Javite točne greške!

---

## 📌 Što Trebam Od Vas

Kopirati-paste u reply:

```
--- TF TREE ---
(output od frames.pdf ili ros2 run tf2_tools view_frames)

--- LASER FRAME ---
frame_id: base_scan    <-- Što je vaš frame_id?

--- TF TRANSFORMS ---
(output od ros2 topic echo /tf)

--- SLAM ERRORS ---
(relevantne error/warn linije)
```

---

## 🌟 Brz Fix - Ako "frame 'base_link' does not exist"

To znači da SLAM ne vidi `base_link` frame!

**Rješenje:**
Ažurirajte `mapper_params_online_async.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    base_frame: base_link         # <- Pazite na spelling!
    odom_frame: odom              # <- Pazite na spelling!
    map_frame: map                # <- Pazite na spelling!
```

SVA IMENA TREBAJU BITI TOČNA!

---

## 🔜 Ako "frame 'base_scan' does not exist"

**To znači da robot_state_publisher ne publishes base_scan -> base_link TF!**

**Rješenje:**
Provjerite da `stage_launch.py` ima:

```python
<robot name="robot">
  <link name="base_link"/>
  <link name="base_scan"/>                    # <-- Base_scan link
  <joint name="base_scan_joint" type="fixed">  # <-- Joint koji ih povezuje
    <parent link="base_link"/>
    <child link="base_scan"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
```

---

## 🚀 Što Trebate Učiniti

1. Pokrenite sve 3 debug naredbe gore
2. Kopira-paste rezultate
3. Javite koja greška se pojavljuje

Tada ću znati što točno popraviti!

---

**Status:** 🔍 ČEKAM DEBUG OUTPUT

**Sljedeće:** Javite rezultate gornje 4 naredbe!
