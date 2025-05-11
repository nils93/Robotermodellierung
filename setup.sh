#!/usr/bin/env bash
#
# Erstinstallation + Build für den Catkin-Workspace
# Nutzbar unter Ubuntu 20.04, ROS Noetic

set -e  # bei Fehler sofort abbrechen

# 1) APT-Pakete für Gazebo, MoveIt, URDF-Tools …
echo "🔧  Abhängigkeiten (apt) installieren …"
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-moveit \
  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-ros-control ros-noetic-ros-controllers \
  ros-noetic-xacro

# 2) rosdep: System-Abhängigkeiten aus den package.xml
echo "🔧  rosdep lösen …"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3) Workspace bauen
echo "🔧  Catkin-Workspace bauen …"
catkin_make

# 4) Hinweis
echo -e "\n✅  Fertig. Jetzt noch:"
echo "   source devel/setup.bash"
echo "   roslaunch multi_robot_station spawn_multi_robot.launch"
