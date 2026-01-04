# RViz - Čist Start (Bez Config Datoteka)

## 🎯 Rješenje

Odustat ćemo od config datoteka. Pokrenite RViz bez bilo čega:

```bash
# Očistite cache
rm -rf ~/.config/rviz2/
rm -rf ~/.rviz2/
rm -rf ~/.ros/

# Pokrenite čisti RViz
rviz2
```

## 📋 Što Trebate Dodati u RViz

Kada se RViz otvori (prazan ekran), dodajte:

### 1. **Displays Panel** (ako nema)
- **Panels** → **Add New Panel** → **Displays**

### 2. **Dodajte Displaye**
U Displays panelu, kliknite **Add**:

| Redni broj | Display | Tip |
|-----------|---------|-----|
| 1 | Grid | rviz_default_plugins/Grid |
| 2 | TF | rviz_default_plugins/TF |
| 3 | LaserScan | rviz_default_plugins/LaserScan |
| 4 | Map | rviz_default_plugins/Map |
| 5 | Marker Array | rviz_default_plugins/MarkerArray |

### 3. **Postavke za Svaki Display**

#### Grid
- ✅ **Enabled: true**
- Reference Frame: `<Fixed Frame>`
- Cell Size: 1

#### TF
- ✅ **Enabled: true**
- Frame Timeout: 15
- Marker Scale: 1
- Show Names: true
- Show Axes: true

#### LaserScan
- ✅ **Enabled: true**
- Topic: `/base_scan`
- Size (m): 0.05
- Style: Flat Squares

#### Map
- ✅ **Enabled: true**
- Topic: `/map`
- Color Scheme: map
- Transparency: 0.7

#### Marker Array
- ✅ **Enabled: true**
- Marker Topic: `/visualization_marker_array`

### 4. **Glavne Postavke**
- **Global Options** → **Fixed Frame**: `map`
- **Global Options** → **Frame Rate**: 30

## 🚀 Kompletan Proces

### Terminal 1: Stage
```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
ros2 launch student_assignment_02 stage_launch.py stage:=true rviz:=false
```

### Terminal 2: SLAM (opciono)
```bash
cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-02
ros2 launch student_assignment_02 online_async_launch.py use_sim_time:=true
```

### Terminal 3: RViz
```bash
# Očisti cache
rm -rf ~/.config/rviz2/ ~/.rviz2/ ~/.ros/

# Pokreni čist RViz
rviz2
```

## ✨ Trebalo bi vidjeti

Kada sve dodate:

✅ Grid pozadina (bijele kvadratne linije)  
✅ TF frame-ovi (crvene/zelene/plave osi)  
✅ LaserScan točke (bijele točke oko robota)  
✅ Mapa (ako je SLAM pokrenut)  
✅ Markeri - A* putanja (ako je path planning pokrenut)  

## Ako Trebate Spremi Config Nakon

Kada ste zadovoljni sa postavkama:

```
File → Save Config As...
Spremi kao: my_clean_rviz.rviz
```

Zatim možete koristiti:
```bash
rviz2 -d my_clean_rviz.rviz
```

## Napomene

- **Fixed Frame mora biti `map`** - inače se transformacije neće prikazati
- Ako vidite crvene warning-e o frame-ovima, to je OK dok se sustav inicijalizira
- Čekajte 2-3 sekunde da se sve učita
- Ako nema nikakvih vidljivih podataka, provjerite su li čvorovi pokrenut (`ros2 node list`)

---

**Status:** ✅ Čist start, bez config datoteka
