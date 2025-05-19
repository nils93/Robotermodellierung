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

    Args:
        group_name (str): The MoveIt planning group name (e.g., "sero_1_arm")
        pose (geometry_msgs.msg.Pose): Target pose for the end-effector

    Returns:
        bool: True if execution was successful, False otherwise
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

def create_pose(name, x, y, z, roll_deg, pitch_deg, yaw_deg):
    """
    Creates and returns a geometry_msgs Pose from position and orientation (RPY in degrees).

    Args:
        name (str): A label for logging/debugging purposes
        x, y, z (float): Position coordinates
        roll_deg, pitch_deg, yaw_deg (float): Orientation in degrees

    Returns:
        geometry_msgs.msg.Pose: The constructed pose
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

# ---------- Main Setup ----------
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_simple_control', anonymous=True)

    # Example calls
    sero3_pose1 = create_pose("pose1", 0.0, 1.0, 0.7, 0, 0, 0)
    move_to_pose("sero_3_arm", sero3_pose1)

    sero1_pose1 = create_pose("pose1", 1.0, 0.0, 1.0, 0, -45, 0)
    move_to_pose("sero_1_arm", sero1_pose1)

    sero3_pose2 = create_pose("pose2", 0.8, 0.0, 1.5, 0, 0, 0)
    move_to_pose("sero_3_arm", sero3_pose2)

    sero3_pose3 = create_pose("pose3", 0.0, -1.0, 1.5, 0, 0, 0)
    move_to_pose("sero_3_arm", sero3_pose3) 

moveit_commander.roscpp_shutdown()
