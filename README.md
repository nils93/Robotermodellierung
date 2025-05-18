# Robotermodellierung/sero_multi_station_moveit_path_planning

## 1. Clone git repository 
```bash
git clone https://github.com/nils93/Robotermodellierung.git sero_ws
cd sero_ws
```

## 2. Change branch 
```bash
git checkout sero_multi_station_moveit_path_planning
```

## 3. Build workspace
```bash
catkin build
```

## 4. Source workspace 
```bash
source devel/setup.bash
```

## 5. Open movit with gazebo simulation
```bash
roslaunch sero_multi_station bringup_moveit.launch
```

## 6. Open movit with gazebo simulation and station peripherals
```bash
roslaunch sero_multi_station factory_station.launch
```


## Quellen
Nachstehende Repo's werden verwendet:
```bash
https://github.com/pal-robotics/gazebo_ros_link_attacher.git src/gazebo_ros_link_attacher
```