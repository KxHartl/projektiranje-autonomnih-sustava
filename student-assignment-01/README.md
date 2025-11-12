# 3. LAUNCH DATOTEKA ZA AUTOMATIZIRANO SLANJE POZICIJA NA forward_position_controller

Za automatsko testiranje ili demonstraciju rada `forward_position_controller` koristi se:
- Python node: `forward_position_publisher.py`
- Launch datoteka: `publish_forward_positions.launch.py`

## Upute za korištenje

### 1. Pokreni glavnu launch datoteku za kontrolere (forward_position_controller mora biti aktivan)
```bash
ros2 launch fanuc_m10ia_support fanuc_controllers.launch.py
```

Provjeri status:
```bash
ros2 control list_controllers
# ...
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# forward_position_controller[forward_command_controller/ForwardCommandController] active
# joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive
```

### 2. Pokreni launch datoteku za automatsko slanje pozicija
```bash
ros2 launch fanuc_m10ia_support publish_forward_positions.launch.py
```

**Node će automatski poslati niz pozicija, redom:**
- [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
- [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
- [1.0, 0.5, 0.0, 0.0, 0.0, 0.0]
- [0.5, 0.3, -0.2, 0.0, 0.5, 0.0]
- [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

Svaka vrijednost ostaje 2 sekunde.

### 3. Praćenje rezultata
- U RViz-u robot će "skakati" na svaku vrijednost
- U terminalu: `ros2 topic echo /joint_states` za provjeru pozicija zglobova

### 4. Prilagodba
i
Node možeš uređivati i promijeniti niz pozicija ili brzinu slanja (promjeni listu `positions` ili parametar u `create_timer`).

## Lokacije datoteka
- `src/fanuc_m10ia_support/test/forward_position_publisher.py`
- `src/fanuc_m10ia_support/launch/publish_forward_positions.launch.py`

## Napomena
Node koristi std_msgs/msg/Float64MultiArray i radi isključivo uz aktivni `forward_position_controller`.

---
