#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from math import radians

from std_msgs.msg import Bool

abort_flag = False

def abort_callback(msg):
    global abort_flag
    abort_flag = msg.data
    if abort_flag:
        rospy.logwarn("🛑 Abort signal received! Halting sequence...")


def move_to_named_target(group_name, target_name):
    """
    Moves the robot to a predefined named target (e.g., 'home') as set in the MoveIt Setup Assistant.

    Args:
        group_name (str): MoveIt planning group name (e.g., "sero_1_arm")
        target_name (str): Name of the predefined pose (e.g., "home")
    """
    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_planning_time(10)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    rospy.loginfo(f"🏠 Moving {group_name} to named target: '{target_name}'")
    move_group.set_named_target(target_name)

    success = move_group.go(wait=True)
    move_group.stop()

    if success:
        rospy.loginfo(f"✅ {group_name} reached the named target '{target_name}'.")
    else:
        rospy.logwarn(f"❌ {group_name} failed to reach the named target '{target_name}'.")
    return success

def safe_call(label, func):
    global abort_flag
    if abort_flag:
        rospy.logwarn(f"⚠️ Skipping {label} due to abort flag.")
        return False
    rospy.loginfo(f"▶️ Executing: {label}")
    if func:
        rospy.sleep(0.5)
        return True
    else:
        rospy.logwarn(f"❌ Step failed: {label}")
        return False

def move_to_pose(group_name, pose):
    """
    Plans and executes a motion to the specified pose using the given MoveIt group name.
    Includes retry logic, timeout extension, and planner fallback for better reliability.
    """
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # 1. Use current joint state as planning start
    move_group.set_start_state_to_current_state()

    # 2. Conservative motion scaling to avoid large jumps
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    # 3. Tolerances
    move_group.set_goal_position_tolerance(0.05)
    move_group.set_goal_orientation_tolerance(0.1)

    # 4. Extended planning time and attempts
    move_group.set_planning_time(30.0)
    move_group.set_num_planning_attempts(5)

    # 5. Planner setup
    move_group.set_planner_id("RRTConnectkConfigDefault")

    # 6. Set the target pose
    success = move_group.set_joint_value_target(pose, True)
    if not success:
        rospy.logwarn(f"⚠️ IK target for {group_name} was rejected by KDL (approximate solution not found)")
        return False


    rospy.loginfo(f"🚀 Planning to move {group_name} to a new pose with shortest path preference...")
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # 7. Retry with alternate planner if timed out
    if not success:
        rospy.logwarn(f"⚠️ {group_name} failed with RRTConnect – trying fallback planner (PRM)...")
        move_group.set_planner_id("PRMkConfigDefault")
        move_group.set_start_state_to_current_state()
        move_group.set_pose_target(pose)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

    # 8. Slight Z-offset fallback if still failed
    if not success:
        rospy.logwarn(f"🔁 {group_name} failed again – retrying with +1cm Z offset.")
        pose.position.z += 0.01
        move_group.set_start_state_to_current_state()
        move_group.set_pose_target(pose)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

    if success:
        rospy.loginfo(f"✅ {group_name} reached the target pose successfully.")
    else:
        rospy.logwarn(f"❌ {group_name} failed to reach the target pose after all retries.")
    return success



def move_to_position(group_name, x, y, z):
    """
    Moves the robot to a target position (x, y, z) with no strict orientation constraint,
    using set_approximate_joint_value_target() for better IK compatibility with KDL.

    Args:
        group_name (str): MoveIt planning group
        x, y, z (float): Target position in meters

    Returns:
        bool: True if successful, False otherwise
    """
    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_planning_time(20)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    # Create a dummy pose with neutral orientation
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0  # Identity quaternion (neutral)
    pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0

    # Use approximate joint target to relax orientation constraints
    success = move_group.set_joint_value_target(pose, True)
    if not success:
        rospy.logwarn("⚠️ IK target was rejected by KDL – could not find approximate solution")
        return False

    rospy.loginfo(f"🚀 Moving {group_name} to position (approximate IK): x={x}, y={y}, z={z}")
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    if success:
        rospy.loginfo(f"✅ {group_name} reached the position (with relaxed orientation).")
    else:
        rospy.logwarn(f"❌ {group_name} failed to reach the position.")
    return success



def create_pose(name, x, y, z, roll_deg, pitch_deg, yaw_deg):
    """
    Creates and returns a geometry_msgs Pose from position and orientation (RPY in degrees).
    """
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    roll = radians(roll_deg)
    pitch = radians(pitch_deg)
    yaw = radians(yaw_deg)

    q = quaternion_from_euler(roll, pitch, yaw)
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    rospy.loginfo(f"🧱 Created pose '{name}' at x:{x}, y:{y}, z:{z} with RPY({roll_deg}°, {pitch_deg}°, {yaw_deg}°)")
    return pose

def move_to_joint_positions_deg(group_name, joint_values_deg):
    """
    Moves the robot arm to specific joint positions given in degrees.

    Args:
        group_name (str): MoveIt planning group (e.g., "sero_1_arm")
        joint_values_deg (list of float): Joint angles in degrees
    """
    # Convert degrees to radians
    joint_values = [radians(angle) for angle in joint_values_deg]

    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_planning_time(30)
    move_group.set_num_planning_attempts(10)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    move_group.set_joint_value_target(joint_values)

    rospy.loginfo(f"🔧 Moving {group_name} to joint values (deg): {joint_values_deg}")
    success = move_group.go(wait=True)
    move_group.stop()

    if success:
        rospy.loginfo(f"✅ {group_name} reached the joint target.")
    else:
        rospy.logwarn(f"❌ {group_name} failed to move to joint target.")
    return success

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_simple_control', anonymous=True)
    rospy.Subscriber("/abort_path", Bool, abort_callback)

    # Init all MoveGroupCommander instances once
    sero_1 = moveit_commander.MoveGroupCommander("sero_1_arm")
    sero_2 = moveit_commander.MoveGroupCommander("sero_2_arm")
    sero_3 = moveit_commander.MoveGroupCommander("sero_3_arm")

    # Home all robots
    safe_call("SERO 1 to home", lambda: move_to_named_target("sero_1_arm", "sero_1_home"))
    safe_call("SERO 2 to home", lambda: move_to_named_target("sero_2_arm", "sero_2_home"))
    safe_call("SERO 3 to home", lambda: move_to_named_target("sero_3_arm", "sero_3_home"))

    # Move sero_3_arm to several poses
    poses_sero3 = [
        create_pose("pose1", 0.0, 1.1, 0.7, 0, 0, 0),
        create_pose("pose2", 0.8, 0.0, 1.5, 0, 0, 0),
        create_pose("pose3", 0.0, -1.0, 1.5, 0, 0, 0),
        create_pose("pose4", -1.0, 0.0, 0.7, 0, 0, 0),
    ]

    for i, p in enumerate(poses_sero3, 1):
        safe_call(f"SERO 3 pose{i}", lambda p=p: move_to_pose("sero_3_arm", p))  # wichtig: p=p kopieren

    safe_call("SERO 1 position 1", lambda: move_to_position("sero_1_arm", -0.5, 0.2, 1.0))
    safe_call("SERO 1 position 2", lambda: move_to_position("sero_1_arm", -0.5, -0.2, 1.0))
    safe_call("SERO 2 position 1", lambda: move_to_position("sero_2_arm", 0.3, -1.2, 0.6))
    safe_call("SERO 2 position 2", lambda: move_to_position("sero_2_arm", -0.3, -1.2, 0.6))


    move_to_named_target("sero_1_arm", "sero_1_home")
    move_to_named_target("sero_2_arm", "sero_2_home")
    move_to_named_target("sero_3_arm", "sero_3_home")

    moveit_commander.roscpp_shutdown()
