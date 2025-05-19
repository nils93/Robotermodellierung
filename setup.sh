#!/usr/bin/env bash
#
# Erstinstallation + Build für den Catkin-Workspace
# Nutzbar unter Ubuntu 20.04, ROS Noetic

set -e  # bei Fehler sofort abbrechen

# 1) Workspace bauen
echo "🔧  Catkin-Workspace bauen …"
catkin build

# 2) Hinweis
echo -e "\n✅  Fertig. Jetzt noch:"
echo "   source devel/setup.bash"
echo "   roslaunch sero_multi_station factory_station.launch"
