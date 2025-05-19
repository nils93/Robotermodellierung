#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from math import radians

def move_to_pose(group_name, pose):
    """
    Plans and executes a motion to the specified pose using the given MoveIt group name.
    """
    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_planning_time(10)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    move_group.set_pose_target(pose)

    rospy.loginfo(f"🚀 Moving {group_name} to target pose...")
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    if success:
        rospy.loginfo(f"✅ {group_name} reached the target pose.")
    else:
        rospy.logwarn(f"❌ {group_name} failed to reach the target pose.")
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

    move_group.set_planning_time(10)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    # Create a dummy pose with neutral orientation
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0  # Identity quaternion (neutral)

    # Use approximate joint target to relax orientation constraints
    move_group.set_joint_value_target(pose, True)

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
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

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

    move_group.set_planning_time(10)
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


# ---------- Main Setup ----------
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_simple_control', anonymous=True)

"""     # Move sero_3_arm to pose
    sero3_pose1 = create_pose("pose1", 0.0, 1.0, 0.7, 0, 0, 0)
    move_to_pose("sero_3_arm", sero3_pose1)

    # Move sero_1_arm using position-only (no orientation constraint)
    sero1_pose1 = create_pose("pose5", 1.85, 0.35, 1.0, 0, 0, 45)
    move_to_pose("sero_1_arm", sero1_pose1)

    1.497, -0.04697, 1.150, 0.38304033774535784, 0.38304033774535784, 0.5943881867164019

    # Move sero_2_arm using position-only (no orientation constraint)
    move_to_position("sero_2_arm", 0.1, -1.0, 1.0)

    # Continue with full pose motions for sero_3_arm
    sero3_pose2 = create_pose("pose2", 0.8, 0.0, 1.5, 0, 0, 0)
    move_to_pose("sero_3_arm", sero3_pose2)

    sero3_pose3 = create_pose("pose3", 0.0, -0.8, 1.5, 0, 0, 0)
    move_to_pose("sero_3_arm", sero3_pose3)

    sero3_pose4 = create_pose("pose4", -0.8, 0.0, 0.7, 0, 0, 0)
    move_to_pose("sero_3_arm", sero3_pose4) 

    # Move sero_2_arm using position-only (no orientation constraint)
move_to_position("sero_3_arm", 0.0, 1.0, 0.7)

    # Move sero_1_arm to a known joint configuration
    # (e.g., [j1, j2, j3, j4, j5]) — adapt to your robot's DoF
move_to_joint_positions_deg("sero_1_arm", [90, 0, 0, 0, 0])

move_to_joint_positions_deg("sero_1_arm", [0, 90, 0, 0, 0])

move_to_joint_positions_deg("sero_1_arm", [0, 0, 90, 0, 0]) """

move_to_position("sero_3_arm", 0.0, 1.0, 0.7)

move_to_position("sero_1_arm", 1.0, 0.35, 1.0)

move_to_position("sero_2_arm", 0.5, -1.0, 1.0)

moveit_commander.roscpp_shutdown()
