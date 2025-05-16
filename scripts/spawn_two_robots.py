#!/usr/bin/env python3

import subprocess
import time
import os

def cleanup_ros():
    print("🧹 Cleaning up old ROS/Gazebo processes ...")
    cleanup_cmd = [
        "killall", "-q", "-9",
        "roslaunch", "rosmaster", "gzserver", "gzclient",
        "spawn_model", "spawner", "move_group",
        "controller_spawner", "robot_state_publisher"
    ]
    subprocess.call(cleanup_cmd)
    time.sleep(2)

def wait_for_roscore(timeout=15):
    print("⏳ Waiting for roscore ...")
    start_time = time.time()
    while time.time() - start_time < timeout:
        result = os.system("rosnode list > /dev/null 2>&1")
        if result == 0:
            print("✅ ROS master is running.")
            return True
        time.sleep(1)
    print("❌ Timeout waiting for ROS master.")
    return False

def wait_for_service(service_name, timeout=30):
    print(f"⏳ Waiting for ROS service '{service_name}' ...")
    start_time = time.time()
    while time.time() - start_time < timeout:
        result = os.system(f"rosservice list | grep -q {service_name}")
        if result == 0:
            print(f"✅ Service '{service_name}' is available.")
            return True
        time.sleep(1)
    print(f"❌ Timeout waiting for service '{service_name}'")
    return False

def launch_in_terminal(command):
    subprocess.Popen([
        "gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"
    ])

if __name__ == "__main__":
    cleanup_ros()

    # Starte Gazebo-Welt
    launch_in_terminal("roslaunch multi_robot_station gazebo_world.launch")

    if not wait_for_roscore():
        exit(1)

    if not wait_for_service("/gazebo/spawn_urdf_model", timeout=30):
        exit(1)

    # Roboter 1 starten
    launch_in_terminal(
        "roslaunch sero_1_moveit spawn_robot.launch "
        "robot_name:=sero_1 robot_namespace:=robot1 "
        "urdf_path:=$(rospack find sero_1)/urdf/sero_1.urdf "
        "world_pose:='-x -1.0 -y 0.0 -z 0.5 -Y 0.0'"
    )
    time.sleep(2)

    # Roboter 2 starten
    launch_in_terminal(
        "roslaunch sero_2_moveit spawn_robot.launch "
        "robot_name:=sero_2 robot_namespace:=robot2 "
        "urdf_path:=$(rospack find sero_2)/urdf/sero_2.urdf "
        "moveit_package:=sero_2_moveit "
        "world_pose:='-x 1.0 -y 0.0 -z 0.5 -Y 1.57'"
    )
