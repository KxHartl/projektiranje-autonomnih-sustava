# turtlesim_sensors_template

## Additionals

Commands that may need to be run for _plotjuggler_ (vizualization of data) and _robot_localization_ (for sensor fusion)
```bash
sudo apt update
```

```bash
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
```
```bash
sudo apt-get install ros-$ROS_DISTRO-robot-localization
```
## Sensor fusion
1. Example, creating a workspace named _das_sensor_fusion_:

```bash
mkdir -p ~/das_sensor_fusion/src
cd ~/das_sensor_fusion/src
```

Make sure you are inside **das_sensor_fusion/src** folder.
```bash
cd ~/das_sensor_fusion/src
git clone https://github.com/CRTA-Lab/turtlesim_sensors_template.git
```
## _robot_localization_ links:
[robot_localization - Docs](https://docs.ros.org/en/noetic/api/robot_localization/html/index.html)

[robot_localization - Paper](https://docs.ros.org/en/lunar/api/robot_localization/html/_downloads/robot_localization_ias13_revised.pdf)




