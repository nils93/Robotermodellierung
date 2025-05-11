# Packages anlegen
```
nifa@DESKTOP-0ASOLVD:~/catkin_ws/src$ catkin_create_pkg sero_1 urdf xacro gazebo_ros
Created file sero_1/package.xml
Created file sero_1/CMakeLists.txt
Successfully created files in /home/nifa/catkin_ws/src/sero_1. Please adjust the values in package.xml.
nifa@DESKTOP-0ASOLVD:~/catkin_ws/src$ catkin_create_pkg sero_2 urdf xacro gazebo_ros
Created file sero_2/package.xml
Created file sero_2/CMakeLists.txt
Successfully created files in /home/nifa/catkin_ws/src/sero_2. Please adjust the values in package.xml.
nifa@DESKTOP-0ASOLVD:~/catkin_ws/src$ catkin_create_pkg sero_3 urdf xacro gazebo_ros
Created file sero_3/package.xml
Created file sero_3/CMakeLists.txt
Successfully created files in /home/nifa/catkin_ws/src/sero_3. Please adjust the values in package.xml.
```

In den entsprechenden Ordner wechseln, in diesem Fall: sero_2 und die Unterordner für launch, meshes und urdf sowie die Dateien spawn_sero_2.launch und sero_2.urdf anlegen.
```
cd sero_2/
mkdir launch meshes urdf
```

Die Struktur sollte dann so aussehen:

```
├── CMakeLists.txt
├── launch
│   └── spawn_sero_2.launch
├── meshes
│   ├── base.stl
│   ├── ee.stl
│   ├── link_01.stl
│   ├── link_02.stl
│   ├── link_03.stl
│   ├── link_04.stl
│   └── link_05.stl
├── package.xml
└── urdf
    └── sero_2.urdf
```

Dann wenn alles erstellt wurde, einmal den Catkin-Workspace bauen und sourcen.
```
cd ~/catkin_ws/src
catkin_make
source ~/catkin_ws/devel/setup.bash
```

Im Anschluss das Launchfile starten
```
roslaunch multi_robot_station spawn_multi_robot.launch
```