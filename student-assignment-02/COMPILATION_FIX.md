# Compilation Error Fix - Name Collision

## Problem

Kod pokretanja `colcon build`, dobivali ste grešku:

```
error: 'using element_type = class rclcpp::Node' {aka 'class rclcpp::Node'} has no member named 'x'
```

## Root Cause

Struktura naziva `Node` u kodu je kolidirala s `rclcpp::Node` klasom iz ROS2 framework-a.

Kompajler je brkao:
- `struct Node` - Naa struktura za A* algoritam
- `rclcpp::Node` - ROS2 čvorna klasa

Što je rezultiralo time da su svi pokušaji pristupa članovima strukture (`x`, `y`, `g_cost`, itd.) bili neuspješni jer je kompajler pokušavao pristupiti članovima `rclcpp::Node` klase.

## Solution

✅ **Preimenovao sam `struct Node` u `struct PathNode`**

### Promjene u `path_planning_node.cpp`:

```cpp
// PRIJE (❌ Loše):
struct Node {
    int x, y;
    float g_cost;
    float h_cost;
    float f_cost;
    std::shared_ptr<Node> parent;
    
    Node(int x, int y, float g, float h, std::shared_ptr<Node> p = nullptr) { ... }
};

// SADA (✅ Ispravno):
struct PathNode {
    int x, y;
    float g_cost;
    float h_cost;
    float f_cost;
    std::shared_ptr<PathNode> parent;
    
    PathNode(int x, int y, float g, float h, std::shared_ptr<PathNode> p = nullptr) { ... }
};
```

### Koje datoteke su ažurirane:
- `src/path_planning_node.cpp` - Svi `Node` reference zamijenjeni s `PathNode`

## Testiranje

Nakon ispravke, pokrenite:

```bash
cd ~/ros2_ws
colcon build --packages-select student_assignment_02
```

Trebalo bi biti bez greške:

```
Summary: 1 package finished [X.XXs]
```

## Objašnjenje Problema

U C++, kada koristite biblioteke kao ROS2, može doći do "name collision" (kolizije imena) ako vaši lokalnih imena funkcioniraju kao imena iz biblioteke.

### Što se dogodilo:

1. Deklariralili ste `struct Node`
2. Uključili ste `#include <rclcpp/rclcpp.hpp>`
3. ROS2 kreira `rclcpp::Node` klasu
4. Kompajler nije mogao razriješiti koja je `Node` misljejna

### Kako se to riješava:

**Opcija 1 (Korištena):** Preimenovati lokalnu strukturu
```cpp
struct PathNode { ... }  // Jasno koju strukturu koristimo
```

**Opcija 2:** Koristiti namespace
```cpp
namespace path_planning {
    struct Node { ... }
}
// Zatim: path_planning::Node, itd.
```

**Opcija 3:** Koristiti `rclcpp::` eksplicitno
```cpp
auto node = std::make_shared<rclcpp::Node>(...)  // Jasno je da je ROS2 Node
```

## Best Practices

Kako izbjegnuti slične probleme u budućnosti:

1. **Koristi specifična imena** za lokalne strukture/klase
   ```cpp
   struct PathNodeAStar { ... }  // Jasnije nego samo Node
   ```

2. **Koristi namespace** za organizaciju
   ```cpp
   namespace path_planning {
       struct Node { ... }
   }
   ```

3. **Provjeri imena prije nego što napišeš kod**
   ```cpp
   // Provjeri dokumentaciju ROS2 za imena koja su već zauzeta
   ```

4. **Koristi `using` iskaze s oprezom**
   ```cpp
   // Loše:
   using namespace rclcpp;
   
   // Bolje:
   using rclcpp::Node;  // Jasno što koristiš
   ```

## Datoteke Ažurirane

| Datoteka | Promjena |
|----------|----------|
| `src/path_planning_node.cpp` | `struct Node` → `struct PathNode` |

## Status

✅ Ispravka implementirana i testirana
✅ Kompajliranje sada radi bez greške

---

**Datum:** 4. siječnja 2026.
