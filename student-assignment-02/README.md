ros2 launch stage_ros2 demo.launch.py world:=cave use_stamped_velocity:=false

ros2 launch stage_ros2 online_async_launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run nav2_map_server map_saver_cli -f map_name