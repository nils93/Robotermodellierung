# 🤖 Robotermodellierung – MoveIt! Path Planning mit Gazebo

## 📦 1. Clone git repository 
```bash
git clone https://github.com/nils93/Robotermodellierung.git sero_ws && cd sero_ws
```

## 🛠️ 2. Start the setup.sh
```bash
./setup.sh
```

## 🔁 3. Source your workspace 
```bash
source devel/setup.bash
```

## 🚀 4. Open movit with gazebo simulation
```bash
roslaunch sero_multi_station bringup_moveit.launch
```

## 🏭 5. Open movit with gazebo simulation and station peripherals
```bash
roslaunch sero_multi_station factory_station.launch
```