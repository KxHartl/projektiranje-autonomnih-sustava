# Testiranje joint_trajectory_controller

Ovaj test potvrđuje ispravnost funkcioniranja `joint_trajectory_controller` kontrolera i glatko pomicanje 6DoF Fanuc M10iA robota.

## Pokretanje testa

1. **Pokreni launch datoteku:**
   ```bash
   ros2 launch fanuc_m10ia_support fanuc_controllers.launch.py
   ```

2. **Aktiviraj joint_trajectory_controller:**
   
   ```bash
   ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller
   ros2 control list_controllers
   # Očekuj:
   # joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
   # forward_position_controller[forward_command_controller/ForwardCommandController] inactive
   # joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
   ```

3. **Pokreni testnu skriptu:**
   ```bash
   cd ~/FSB/projektiranje-autonomnih-sustava/student-assignment-01/src/fanuc_m10ia_support/test
   chmod +x test_trajectory.py
   python3 test_trajectory.py
   ```
   
4. **Promatraj gibanje robota u RViz-u**
   Robot mora glatko slijediti zadane točke trajektorije.

## O testnoj skripti

Skripta koristi ROS2 action server `joint_trajectory_controller/follow_joint_trajectory` i šalje sljedeće trajektorije:
- Početna pozicija svih zglobova
- Novo stanje (pomakni 2. zglob, zakreni više zglobova...)
- Povratak na početak

Sve promjene se odvijaju glatko. Skripta je dostupna na:
`src/fanuc_m10ia_support/test/test_trajectory.py`

## Napomena
- Skripta radi samo uz aktiviranu ROS2 workspaceu.
- Prethodno instaliraj potrebne Python pakete:
   ```bash
   pip3 install rclpy control-msgs trajectory-msgs
   ```
- Kod je jasno komentiran za daljnje proširenje/testiranje.

## Reference
- https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html
- https://github.com/ros-controls/ros2_controllers
