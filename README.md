# 🤖 Robotermodellierung – MoveIt! Path Planning mit Gazebo

## 📦 1. Clone git repository 
```bash
git clone https://github.com/nils93/Robotermodellierung.git sero_ws && cd sero_ws
```

## 🛠️ 2. Start the setup.sh
```bash
./setup.sh
```

## 🧠 3. Run station logic
```bash
rosrun sero_multi_station pathplanning.py
```

## 🍿 4. Enjoy the ultimate sero experience!

## In case something bad happens, here are the key commands from the setup.sh
```bash
source devel/setup.bash && roslaunch sero_multi_station factory_station.launch
source devel/setup.bash && rosrun sero_hmi hmi_gui.py
```
