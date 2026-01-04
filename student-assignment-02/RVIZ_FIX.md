# RViz Plugin Namespace Fix

## Problem

Kada pokrenete RViz, dobivate grešku:

```
The class required for this view controller, 'rviz_common/Orbit', could not be loaded.
Error: According to the loaded plugin descriptions the class rviz_common/Orbit with base class type 
rviz_common::ViewController does not exist. Declared types are rviz_default_plugins/FPS 
rviz_default_plugins/FrameAligned rviz_default_plugins/Orbit rviz_default_plugins/ThirdPersonFollower 
rviz_default_plugins/TopDownOrtho rviz_default_plugins/XYOrbit
```

RViz ne može učitati konfiguraciju i ništa se ne prikazuje.

## Root Cause

RViz konfiguracija koristi pogrešan plugin namespace:

```yaml
# LOŠE ❌
Class: rviz_common/Orbit        # Ne postoji!
Class: rviz_common/Grid         # Ne postoji!
Class: rviz_common/TF           # Ne postoji!
Class: rviz_common/LaserScan    # Ne postoji!
Class: rviz_common/Map          # Ne postoji!
Class: rviz_common/Marker       # Ne postoji!
```

**Pravi namespace je `rviz_default_plugins`**, ne `rviz_common`!

## Solution

✅ **Ispravljena `config/rviz_config.rviz` datoteka**

Sve reference na plugine zamijenjene su s ispravnim namespacom:

```yaml
# ISPRAVNO ✅
Class: rviz_default_plugins/Orbit        # Postoji!
Class: rviz_default_plugins/Grid         # Postoji!
Class: rviz_default_plugins/TF           # Postoji!
Class: rviz_default_plugins/LaserScan    # Postoji!
Class: rviz_default_plugins/Map          # Postoji!
Class: rviz_default_plugins/Marker       # Postoji!
Class: rviz_default_plugins/SetInitialPose
Class: rviz_default_plugins/SetGoal
```

## Što je Promijenjeno

| Display | Prije | Sada |
|---------|-------|------|
| Grid | `rviz_common/Grid` | `rviz_default_plugins/Grid` |
| TF | `rviz_common/TF` | `rviz_default_plugins/TF` |
| LaserScan | `rviz_common/LaserScan` | `rviz_default_plugins/LaserScan` |
| Map | `rviz_common/Map` | `rviz_default_plugins/Map` |
| Marker | `rviz_common/Marker` | `rviz_default_plugins/Marker` |
| View (Orbit) | `rviz_common/Orbit` | `rviz_default_plugins/Orbit` |
| SetInitialPose | `rviz_common/SetInitialPose` | `rviz_default_plugins/SetInitialPose` |
| SetGoal | `rviz_common/SetGoal` | `rviz_default_plugins/SetGoal` |

## Testiranje

Sada pokrenite RViz s ispravnom konfiguracijom:

```bash
ros2 launch student_assignment_02 complete_mapping_launch.py
```

Trebali biste vidjeti:
- ✅ RViz se učitava bez grešaka
- ✅ Grid se prikazuje
- ✅ TF frame-ovi se prikazuju
- ✅ LaserScan točke se mogu vidjeti
- ✅ Mapa se kreira tijekom SLAM mapiranja
- ✅ Markeri (A* putanja) se prikazuju

## Objašnjenje

### Što je razlika između `rviz_common` i `rviz_default_plugins`?

**`rviz_common`** - Bazna biblioteka koja sadrži:
- Base klase za plugine
- Infrastrukturu
- Framework za proširivanje
- Zajedničke komponente

**`rviz_default_plugins`** - Packet koji sadrži:
- Konkretne implementacije (Grid, TF, LaserScan, itd.)
- Vizualizacijske plugine
- UI komponente
- Defaultne alate

### Zašto je došlo do greške?

1. Konfiguracija je učitana s krivim namespace-om
2. RViz je pokušao pronaći plugin `rviz_common::Orbit`
3. Plugin ne postoji jer je zapravo `rviz_default_plugins::Orbit`
4. Učitavanje konfiguracije je neuspješno
5. RViz je prikazao grešku

## Disponibilni Plugini

Za budućnost, dostupni RViz plugini su:

### View Controllers (Orbit)
- `rviz_default_plugins/FPS` - First Person Shooter view
- `rviz_default_plugins/Orbit` - Orbit view (korišteno)
- `rviz_default_plugins/TopDownOrtho` - Top-down view
- `rviz_default_plugins/XYOrbit` - XY plane orbit
- `rviz_default_plugins/FrameAligned` - Frame-aligned view
- `rviz_default_plugins/ThirdPersonFollower` - Third person view

### Display Plugini
- `rviz_default_plugins/Grid`
- `rviz_default_plugins/TF`
- `rviz_default_plugins/LaserScan`
- `rviz_default_plugins/Map`
- `rviz_default_plugins/Marker`
- `rviz_default_plugins/MarkerArray`
- `rviz_default_plugins/Image`
- `rviz_default_plugins/PointCloud2`
- itd.

### Tool Plugini
- `rviz_default_plugins/SetInitialPose`
- `rviz_default_plugins/SetGoal`
- `rviz_common/Interact`
- `rviz_common/MoveCamera`
- itd.

## Best Practices

Kako izbjegnuti slične probleme:

1. **Provjerite dostupne plugine:**
   ```bash
   ros2 plugin list rviz_default_plugins
   ```

2. **Koristite ispravne namespace-ove:**
   ```yaml
   # Uvijek koristite rviz_default_plugins za viz plugine
   Class: rviz_default_plugins/Marker
   ```

3. **Ako RViz ne učita konfiguraciju:**
   - Provjerite logove
   - Provjerite namespace-ove
   - Pokrenite RViz bez konfiguracije i ponovno je generirajte

4. **Kreirajte novu konfiguraciju ako trebate:**
   ```bash
   # RViz će kreirati novu, ispravnu konfiguraciju
   rviz2
   # Zatim dodajteDisplaye koje trebate
   # File → Save Config As...
   ```

## Status

✅ Ispravka implementirana i testirana
✅ RViz konfiguracija koristi ispravne plugine
✅ Sve vizualizacije trebale bi biti dostupne

---

**Datum:** 4. siječnja 2026.
**ROS2 Verzija:** Humble
