#!/usr/bin/env bash
#
# @file setup.sh
# @brief Builds the workspace and launches the SERO simulation and HMI.
# @package sero_ws

set -e

# 1) Bestätigung vor dem Build
echo -e "\n🔧 This will build the Catkin workspace (catkin build)."
read -p "Do you want to continue? [y/n] " build_choice

case "$build_choice" in
  y|Y )
    echo "🔨 Building workspace..."
    catkin build
    ;;
  n|N )
    echo "❌ Aborted by user before build."
    exit 0
    ;;
  * )
    echo "⚠️ Invalid input. Please enter y or n."
    exit 1
    ;;
esac

# 2) Bestätigung vor dem Start
echo -e "\n⚙️  This will launch the SERO simulation and HMI in two separate terminals."
read -p "Do you want to continue? [y/n] " run_choice

case "$run_choice" in
  y|Y )
    echo "🚀 Launching components..."

    # Terminal 1: Simulation
    gnome-terminal -- bash -c "source devel/setup.bash && roslaunch sero_multi_station factory_station.launch; exec bash"

    # Terminal 2: HMI GUI
    gnome-terminal -- bash -c "source devel/setup.bash && rosrun sero_hmi hmi_gui.py; exec bash"

    echo "✅ All components launched."
    ;;
  n|N )
    echo "❌ Aborted by user before launching."
    exit 0
    ;;
  * )
    echo "⚠️ Invalid input. Please enter y or n."
    exit 1
    ;;
esac
