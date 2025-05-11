# Robotermodellierung

## 1. Git Repository klonen
```bash
git clone https://github.com/nils93/Robotermodellierung.git sero_ws
cd sero_ws
```

## 2. Setup-Skript ausführen
```bash
./setup.sh
source devel/setup.bash
```

## 3. Gazebo laden
```bash
roslaunch multi_robot_station spawn_multi_robot.launch
```

## Quellen
Nachstehende Repo's werden verwendet:
```bash
https://github.com/pal-robotics/gazebo_ros_link_attacher.git src/gazebo_ros_link_attacher
```