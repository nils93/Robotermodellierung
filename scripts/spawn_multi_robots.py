#!/usr/bin/env python3

import subprocess
import time
import os

launched_terminals = []


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

def wait_for_topic(topic_name, timeout=20):
    print(f"⏳ Waiting for topic '{topic_name}' ...")
    import rosgraph
    start = time.time()
    while time.time() - start < timeout:
        try:
            master = rosgraph.Master('/rostopic')
            topics = master.getPublishedTopics("/")
            if topic_name in [t[0] for t in topics]:
                print(f"✅ Topic '{topic_name}' is available.")
                return True
        except:
            pass
        time.sleep(1)
    print(f"❌ Timeout: Topic '{topic_name}' not available.")
    return False

def launch_in_terminal(command):
    print(f"🚀 Launching in terminal: {command}")
    proc = subprocess.Popen([
        "gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"
    ])
    launched_terminals.append(proc.pid)


if __name__ == "__main__":
    cleanup_ros()

    print("🌍 Launching Gazebo...")
    launch_in_terminal("roslaunch multi_robot_station gazebo_world.launch")

    if not wait_for_roscore():
        exit(1)

    if not wait_for_topic("/clock", timeout=20):
        exit(1)

    print("🤖 Spawning robot and starting MoveIt...")
    launch_in_terminal("roslaunch sero_1_moveit spawn_robot.launch")

    #Fahre eine Bewegung
    print("🎯 Publishing motion command...")
    time.sleep(3)
    launch_in_terminal(
        "rostopic pub --once /sero_1/arm_controller/command trajectory_msgs/JointTrajectory "
        "'{header: {stamp: {secs: 0, nsecs: 0}}, joint_names: [\"j1\", \"j2\", \"j3\", \"j4\", \"j5\"], "
        "points: [{positions: [0.2, -0.5, 1.0, 0.5, -0.2], time_from_start: {secs: 2, nsecs: 0}}]}'"
    )

    print("🟢 Setup complete. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        cleanup_ros()
        print("🛑 Closing all launched terminal windows ...")
        for pid in launched_terminals:
            try:
                os.kill(pid, 9)
            except ProcessLookupError:
                pass
        print("👋 Exiting script.")
