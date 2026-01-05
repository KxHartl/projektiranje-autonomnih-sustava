# 🗺️ Fleksibilan Odabir Mapa - Vodič

## 🚨 Direktorij Struktura

Sve mape trebaju biti u direktoriju `data/maps/`:

```
student-assignment-02/
├── data/
│   └── maps/
│       ├── my_map/              ← Vaša trenutna mapa
│       │   ├── map.yaml
│       │   ├── map.pgm
│       │   └── map.txt
│       ├── office/              ← Primjer druge mape
│       │   ├── map.yaml
│       │   ├── map.pgm
│       │   └── map.txt
│       ├── lab/                 ← Primjer treće mape
│       │   ├── map.yaml
│       │   ├── map.pgm
│       │   └── map.txt
│       └── warehouse/           ← Primjer četvrte mape
│           ├── map.yaml
│           ├── map.pgm
│           └── map.txt
├── src/
├── build/
├── install/
└── ...
```

---

## 🚀 BRZI START

### Setup

```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
colcon build
source install/setup.bash
```

### Pokretanje s Odabirom Mape

**Koristi default mapu (my_map):**
```bash
ros2 launch student_assignment_02 flexible_path_planning_launch.py
```

**Odaberi drugu mapu:**
```bash
ros2 launch student_assignment_02 flexible_path_planning_launch.py map_name:=office
```

**Primjeri drugih mapa:**
```bash
# Lab mapa
ros2 launch student_assignment_02 flexible_path_planning_launch.py map_name:=lab

# Warehouse mapa
ros2 launch student_assignment_02 flexible_path_planning_launch.py map_name:=warehouse

# Bilo koja druga mapa u data/maps/
ros2 launch student_assignment_02 flexible_path_planning_launch.py map_name:=custom_environment
```

---

## 🎯 Kako Prebaciti Vašu Mapu

### Korak 1: Kreirajte Direktorij

```bash
mkdir -p ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/my_map
```

### Korak 2: Prebacite Mape

Ako je mapa u `~/my_map/`:

```bash
cp ~/my_map/map.yaml ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/my_map/
cp ~/my_map/map.pgm ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/my_map/
cp ~/my_map/map.txt ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/my_map/ 2>/dev/null || true
```

### Korak 3: Provjerite

```bash
ls -la ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/my_map/
```

Trebalo bi vidjeti:
```
-rw-r--r-- 1 khartl khartl  50K Jan  5 10:00 map.pgm
-rw-r--r-- 1 khartl khartl 150B Jan  5 10:00 map.yaml
```

---

## 📂 Dodavanje Nove Mape

### Korak 1: Kreirajte Direktorij za Novu Mapu

```bash
mkdir -p ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/nova_mapa
```

### Korak 2: Prebacitre YAML i PGM Datoteke

```bash
cp /path/to/your/map.yaml ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/nova_mapa/
cp /path/to/your/map.pgm ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/nova_mapa/
```

### Korak 3: Testirajte

```bash
ros2 launch student_assignment_02 flexible_path_planning_launch.py map_name:=nova_mapa
```

---

## 📋 Kako Naci Sve Dostupne Mape

```bash
ls -d ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/*/
```

Output:
```
/home/khartl/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/my_map/
/home/khartl/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/office/
/home/khartl/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/lab/
```

---

## 🔧 Kako Funkcionira

### Launch File Parameter

Launch file `flexible_path_planning_launch.py` ima parametar `map_name`:

```python
map_name_arg = DeclareLaunchArgument(
    'map_name',
    default_value='my_map',  # Default ako nije naveden
    description='Name of the map in data/maps/'
)
```

### Putanja do Mape

Izgra putanju dinamički:

```python
map_yaml = os.path.join(
    package_dir,
    '..',  # Izlazi iz src/student_assignment_02
    '..',  # Izlazi iz src
    'data',
    'maps',
    LaunchConfiguration('map_name'),  # ← Koristi map_name parametar
    'map.yaml'
)
```

**Rezultat za `map_name:=lab`:**
```
~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/lab/map.yaml
```

---

## 📄 Primjena

### Scenarij 1: Testiranje s Različitim Mapama

```bash
# Terminal 1: Pokreni s lab mapom
ros2 launch student_assignment_02 flexible_path_planning_launch.py map_name:=lab

# Testiraj A* i navigaciju
# ...

# Zausti (Ctrl+C)

# Terminal 1: Pokreni s office mapom
ros2 launch student_assignment_02 flexible_path_planning_launch.py map_name:=office

# Testiraj s drugom mapom
# ...
```

### Scenarij 2: Batch Testiranje

```bash
#!/bin/bash
# test_all_maps.sh

for map in my_map office lab warehouse; do
    echo "Testing map: $map"
    ros2 launch student_assignment_02 flexible_path_planning_launch.py map_name:=$map
    # RViz testiranje...
    sleep 10
done
```

---

## 🚿 Ako Ne Radi

### Greška: "Could not find map.yaml"

**Provjerite:**
```bash
ls ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/my_map/map.yaml
```

Trebalo bi vidjeti datoteka.

Ako ne postoji:
```bash
mkdir -p ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/my_map
cp ~/my_map/map.* ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/my_map/
```

### Greška: "Map name not found"

Provjerite naziv mape:

```bash
# Listu dostupnih mapa
ls -1 ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/

# Trebalo bi vidjeti:
# my_map
# office
# lab
# ...
```

Koristite tocno taj naziv:
```bash
ros2 launch student_assignment_02 flexible_path_planning_launch.py map_name:=office
```

---

## 📁 Ažuriranje map.yaml

Ako trebate prilagoditi mapu, uredite `map.yaml`:

```yaml
image: map.pgm
resolution: 0.05      # 5 cm po grid cell
origin: [0.0, 0.0, 0.0]  # Offset mape
negate: 0             # 0 = crno je zauzeto, 1 = bijelo je zauzeto
occupied_thresh: 0.65
free_thresh: 0.196
```

Promjene se mogu učiniti bez ponovnog mapiranja:

```bash
nano ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02/data/maps/my_map/map.yaml
```

Aonda pokrenite ponovo launch file.

---

## ✅ Checklist

- [ ] `data/maps/` direktorij postoji
- [ ] `data/maps/my_map/` postoji s `map.yaml` i `map.pgm`
- [ ] `colcon build` je bez grešaka
- [ ] Možete pokrenuti: `ros2 launch student_assignment_02 flexible_path_planning_launch.py`
- [ ] Možete pokrenuti s parametrom: `ros2 launch ... map_name:=office`
- [ ] RViz prikazuje mapu
- [ ] Možete koristiti 2D Goal Pose
- [ ] A* planira putanju
- [ ] Robot se giba

---

## 📄 Datoteke

| Datoteka | Opis |
|----------|------|
| `flexible_path_planning_launch.py` | Launch file s map_name parametrom |
| `data/maps/` | Direktorij s mapama |
| `data/maps/my_map/` | Vaša mapa |

---

**Status:** ✅ Fleksibilan odabir mapa je spreman!

**Zadnja ažuriranja:** 5. siječnja 2026.
