# RViz Cache Cleanup - Kako Riješiti Plugin Greške

## 🔴 Problem

RViz i dalje izbacuje greške:
```
The plugin for class 'rviz_common/Grid' failed to load
The plugin for class 'rviz_common/TF' failed to load
The plugin for class 'rviz_common/Orbit' failed to load
```

Čak i nakon ažuriranja konfiguracije.

## 🔍 Uzrok

RViz sprema **cache** stare konfiguracije na vašem računalu:

```
~/.ros/rviz2_ogre_media_cache/
~/.rviz2/
```

Cache содržava stare plugin reference na `rviz_common/...` umjesto `rviz_default_plugins/...`

## ✅ Rješenje - OČISTI CACHE

### Korak 1: Zaustavite RViz

Ako je RViz pokrenut, zaustavite ga (Ctrl+C u terminalu).

### Korak 2: Očistite Cache Datoteke

```bash
# Očistite RViz cache
rm -rf ~/.ros/rviz2_ogre_media_cache/
rm -rf ~/.rviz2/
rm -rf ~/.cache/rviz2/
```

### Korak 3: Očistite Build Direktorij (Preporučeno)

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build install log
```

### Korak 4: Ponovno Build-ajte

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
colcon build
source install/setup.bash
```

### Korak 5: Pokrenite RViz

```bash
ros2 launch student_assignment_02 stage_launch.py stage:=true rviz:=true
```

Ili kompletan sustav:

```bash
ros2 launch student_assignment_02 complete_mapping_launch.py
```

## Očekivani Rezultat

Sada bi trebalo:

✅ RViz se pokreće bez greški  
✅ Grid se vidi  
✅ TF transformacije se vide  
✅ LaserScan točke se vide  
✅ Mapa se kreira  
✅ Markeri (A* putanja) se vide  

## Ako Problem Persisti

Ako i dalje vidite greške:

### Opcija A: Krenite s novom RViz konfiguracijom

```bash
# Pokrenite RViz bez konfiguracije
rviz2

# Zatim:
# 1. File → Open Config
# 2. Navigirajte na: ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/install/student_assignment_02/share/student_assignment_02/config/rviz_config.rviz
# 3. Kliknite "Open"
```

### Opcija B: Kreirajte novu konfiguraciju iz nule

```bash
# Pokrenite RViz bez konfiguracije
rviz2 --no-config

# Zatim u RViz-u:
# 1. Add → Display → Grid (rviz_default_plugins/Grid)
# 2. Add → Display → TF (rviz_default_plugins/TF)
# 3. Add → Display → LaserScan (rviz_default_plugins/LaserScan)
# 4. Add → Display → Map (rviz_default_plugins/Map)
# 5. Add → Display → Marker Array (rviz_default_plugins/Marker)

# Zatim:
# File → Save Config As...
# Spremi kao: ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/config/rviz_config_clean.rviz
```

## Kompletan Reset (Ako Sve Ostalo Ne Radi)

```bash
# PAŽNJA: Ovo će obrisati sve RViz konfiguracije
rm -rf ~/.ros
rm -rf ~/.rviz2
rm -rf ~/.cache/rviz2

# Zatim build i pokrenite ponovo
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
rm -rf build install log
colcon build
source install/setup.bash
ros2 launch student_assignment_02 complete_mapping_launch.py
```

## Što Se Dogod

### Prije Čišćenja:
```
~/.rviz2/
  └── recent_display_configs
      └── Contains old rviz_common/Grid references
  └── recent_plugins
      └── Contains cached plugin list with rviz_common
```

### Nakon Čišćenja:
```
RViz će:
1. Učitati novu konfiguraciju (rviz_config.rviz)
2. Koja koristi SAMO rviz_default_plugins
3. Kreirati novi cache s ispravnim pluginima
4. Sve trebalo bi raditi
```

## Provjera je li Cache Očistio

Provjerite da su direktoriji obrisani:

```bash
ls -la ~/.ros/rviz2_ogre_media_cache/
ls -la ~/.rviz2/
```

Ako niste vidjeli ni jedan direktorij, znači da je cache pravilno obrisan.

## Dodatni Savjeti

### 1. Provjerite Environment Variables

```bash
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
```

Za lokalnu komunikaciju, trebali bi biti:
```bash
unset ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY=1
```

### 2. Provjerite RViz verziju

```bash
rviz2 --version
ros2 pkg list | grep rviz
```

### 3. Provjera dostupnih plugina

```bash
ros2 plugin list rviz_default_plugins
```

Trebali biste vidjeti:
```
rviz_default_plugins/Grid
rviz_default_plugins/TF
rviz_default_plugins/LaserScan
rviz_default_plugins/Map
rviz_default_plugins/Marker
rviz_default_plugins/Orbit
... itd
```

## Status

✅ RViz konfiguracija je ažurirana  
✅ Koristi SAMO rviz_default_plugins  
✅ Cache trebate očistiti kako je gore navedeno  

---

**Datum:** 4. siječnja 2026.
**ROS2 Verzija:** Humble
