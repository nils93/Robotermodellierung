
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

def wait_for_controllers_running(namespace, controllers, timeout=10):
    import subprocess
    import time
    print("⏳ Waiting for controllers to become active...")
    start = time.time()
    while time.time() - start < timeout:
        output = subprocess.check_output(["rosservice", "call", f"/{namespace}/controller_manager/list_controllers"])
        if all(f'name: "{c}"\n    state: "running"' in output.decode() for c in controllers):
            print("✅ All controllers are running.")
            return True
        time.sleep(1)
    print("❌ Timeout: Some controllers did not start properly.")
    return False

def launch_in_terminal(command):
    print(f"🚀 Launching in terminal: {command}")
    subprocess.Popen([
        "gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"
    ])

def wait_for_topic(topic_name, timeout=20):
    print(f"⏳ Waiting for topic '{topic_name}' ...")
    import rosgraph
    import rospy
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


def wait_for_controller_service(namespace, timeout=10):
    service_name = f"/{namespace}/controller_manager/load_controller"
    print(f"⏳ Waiting for controller service '{service_name}' ...")
    start = time.time()
    while time.time() - start < timeout:
        try:
            result = subprocess.run(["rosservice", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if service_name in result.stdout.decode():
                print(f"✅ Controller service '{service_name}' is available.")
                return True
        except Exception as e:
            print(f"⚠️ Error while checking services: {e}")
        time.sleep(1)
    print(f"❌ Timeout: Controller service '{service_name}' not available.")
    return False





def spawn_and_setup_robot(namespace, model_name, urdf_path, pose_args, moveit_package):
    print(f"\n🛠️ Spawning and setting up robot in namespace: {namespace}")

    # Setze URDF-Parameter
    print("📦 Setting robot_description param und Spawning model in Gazebo...")
    spawn_cmd = (
        f'rosparam set /{namespace}/robot_description "$(cat {urdf_path})" && '
        f'rosrun gazebo_ros spawn_model -urdf -param /{namespace}/robot_description '
        f'-model {model_name} {pose_args}'
    )
    os.system(spawn_cmd)

    #os.system(f'rosparam set /{namespace}/robot_description "$(cat $(rospack find {namespace})/urdf/{namespace}.urdf)')
    #time.sleep(1)

    # Spawn Robot
    #print("📦 Spawning model in Gazebo...")
    #os.system(f'rosrun gazebo_ros spawn_model -urdf -param /{namespace}/robot_description -model {model_name} {pose_args}')
    # result = subprocess.call([
    #     "rosrun", "gazebo_ros", "spawn_model",
    #     "-urdf", "-param", f"/{namespace}/robot_description",
    #     "-model", model_name
    # ] + pose_args.split())
    # if result != 0:
    #     print("❌ Failed to spawn model")
    #     return
    # time.sleep(5)

    # Lade Controller YAML
    print("🔧 Loading controller config...")
    controllers_yaml = os.popen(f"rospack find {moveit_package}").read().strip() + "/config/ros_controllers.yaml"
    subprocess.call(["rosparam", "load", controllers_yaml, f"/{namespace}"])
    time.sleep(1)

    # Starte joint_state_controller
    print("▶️ Spawning joint_state_controller...")
    load_cmd = f"rosparam load $(rospack find {moveit_package})/config/ros_controllers.yaml /{namespace}"
    subprocess.call(load_cmd, shell=True)
    print("▶️ Spawning joint_state_controller in separate terminal...")
    launch_in_terminal(f"rosrun controller_manager spawner joint_state_controller __ns:=/{namespace}")
    time.sleep(1)

    # Lade und starte restliche Controller
    print("🔄 Loading arm_controller and ee_controller...")
    if wait_for_controller_service(namespace):
        for ctrl in ["arm_controller", "ee_controller"]:
            print(f"▶️ Loading {ctrl}...")
            subprocess.call(["rosservice", "call", f"/{namespace}/controller_manager/load_controller", f"name: '{ctrl}'"])
    else:
        print("❌ Controller services not ready. Aborting.")
        return
    time.sleep(1)
    print("▶️ Switching controllers...")
    subprocess.call(["rosservice", "call", f"/{namespace}/controller_manager/switch_controller",
                    "{start_controllers: ['arm_controller', 'ee_controller'], stop_controllers: [], strictness: 2}"])

    # Warte auf aktive Controller
    wait_for_controllers_running(namespace, ["arm_controller", "ee_controller", "joint_state_controller"])
if __name__ == "__main__":
    cleanup_ros()

    print("🌍 Launching Gazebo...")
    launch_in_terminal("roslaunch multi_robot_station gazebo_world.launch")

    if not wait_for_roscore():
        exit(1)

    # Warten bis /clock da ist (Gazebo ready)
    if not wait_for_topic("/clock", timeout=20):
        exit(1)

    print("⏸ Pausing physics in Gazebo...")
    time.sleep(5)
    # if wait_for_service("/gazebo/pause_physics"):
    #     subprocess.call(["rosservice", "call", "/gazebo/pause_physics"])
    # else:
    #     print("❌ Could not pause physics — service not available.")
    #     exit(1)

    print("✅ Gazebo launched and physics paused. Proceed with spawning robot...")


    # Roboter 1 starten
    spawn_and_setup_robot(
        namespace="sero_1",
        model_name="sero_1_test_1",
        urdf_path=os.popen("rospack find sero_1").read().strip() + "/urdf/sero_1.urdf",
        pose_args="-x -1.0 -y 0.0 -z 0.5 -Y 0.0",
        moveit_package="sero_1_moveit"
    )

    print("▶️ Unpausing physics in Gazebo...")
    subprocess.call(["rosservice", "call", "/gazebo/unpause_physics"])

    #Fahre eine Bewegung
    print("🎯 Publishing motion command...")
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
        print("\n👋 Exiting script.")