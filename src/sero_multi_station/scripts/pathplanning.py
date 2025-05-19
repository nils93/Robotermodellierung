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
    Moves the robot to a target position (x, y, z) with no specific orientation.
    """
    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_planning_time(10)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    move_group.set_position_target([x, y, z])

    rospy.loginfo(f"🚀 Moving {group_name} to position-only target: x={x}, y={y}, z={z}...")
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    if success:
        rospy.loginfo(f"✅ {group_name} reached the position.")
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

# ---------- Main Setup ----------
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_simple_control', anonymous=True)

    # Move sero_3_arm to pose
    sero3_pose1 = create_pose("pose1", 0.0, 1.0, 0.7, 0, 0, 0)
    move_to_pose("sero_3_arm", sero3_pose1)

    # Move sero_1_arm using position-only (no orientation constraint)
    move_to_position("sero_1_arm", 1.0, 0.1, 1.0)

    # Move sero_2_arm using position-only (no orientation constraint)
    move_to_position("sero_2_arm", 0.1, -1.0, 1.0)

    # Continue with full pose motions for sero_3_arm
    sero3_pose2 = create_pose("pose2", 0.8, 0.0, 1.5, 0, 0, 0)
    move_to_pose("sero_3_arm", sero3_pose2)

    sero3_pose3 = create_pose("pose3", 0.0, -1.0, 1.5, 0, 0, 0)
    move_to_pose("sero_3_arm", sero3_pose3)

    moveit_commander.roscpp_shutdown()
