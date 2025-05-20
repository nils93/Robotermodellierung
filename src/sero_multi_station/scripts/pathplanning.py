#!/usr/bin/env python3

## @file pathplanning.py
#  @package sero_multi_station
#  @brief Moves a robot arm between predefined named targets and optionally to custom poses, positions, or joint configurations.
#
#  This script uses MoveIt to execute a typical sequence for a multi-robot cell.
#  It includes named targets and helper functions for absolute poses, Cartesian positions, and joint values.
#
#  @requires rospy
#  @requires moveit_commander
#  @requires geometry_msgs.msg
#  @requires tf.transformations

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from math import radians

def move_to_named_target(group_name, target_name):
    """
    Moves the robot to a predefined named target.
    @param group_name MoveIt group name (e.g. "sero_1_arm")
    @param target_name Name of the predefined target
    """
    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_planning_time(10)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    rospy.loginfo(f"🏠 Moving {group_name} to named target: '{target_name}'")
    move_group.set_named_target(target_name)

    # Use current joint state as planning start
    move_group.set_start_state_to_current_state()

    success = move_group.go(wait=True)
    move_group.stop()

    if success:
        rospy.loginfo(f"✅ {group_name} reached the named target '{target_name}'.")
    else:
        rospy.logwarn(f"❌ {group_name} failed to reach the named target '{target_name}'.")
    return success

def move_to_pose(group_name, pose):
    """
    Plans and executes a motion to the specified absolute pose.
    @param group_name MoveIt group name
    @param pose Target pose as geometry_msgs/Pose
    """
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # 1. Use current joint state as planning start
    move_group.set_start_state_to_current_state()

    # 2. Conservative motion scaling to avoid large jumps
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    # 3. Tight tolerance = avoid overshooting
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.05)

    # 4. Set specific planner (tuned for shortest path in joint space)
    move_group.set_planner_id("RRTConnectkConfigDefault")

    # 5. Set the target pose
    move_group.set_pose_target(pose)

    rospy.loginfo(f"🚀 Planning to move {group_name} to a new pose with shortest path preference...")

    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    if success:
        rospy.loginfo(f"✅ {group_name} reached the target pose successfully.")
    else:
        rospy.logwarn(f"❌ {group_name} failed to reach the target pose.")
    return success

def move_to_position(group_name, x, y, z):
    """
    Moves the robot to a target position (x, y, z) with no strict orientation constraint,
    using set_approximate_joint_value_target() for better IK compatibility with KDL.
    @param group_name MoveIt group
    @param x X position [m]
    @param y Y position [m]
    @param z Z position [m]
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
    @param name Optional label for logging
    @param x,y,z Cartesian coordinates
    @param roll_deg, pitch_deg, yaw_deg Orientation in degrees
    @return geometry_msgs.msg.Pose
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
    @param group_name MoveIt planning group
    @param joint_values_deg List of joint angles in degrees

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

    move_to_named_target("sero_1_arm", "sero_1_home")
    move_to_named_target("sero_2_arm", "sero_2_home")
    move_to_named_target("sero_3_arm", "sero_3_home")
    move_to_named_target("sero_3_arm", "sero_3_pick_wobj")
    move_to_named_target("sero_3_arm", "sero_3_show_wobj_1")
    move_to_named_target("sero_1_arm", "sero_1_start_sanding")
    # rospy.sleep(2)  # Allow time for the arm to reach the pose
    move_to_named_target("sero_1_arm", "sero_1_end_sanding")
    move_to_named_target("sero_1_arm", "sero_1_home")
    move_to_named_target("sero_3_arm", "sero_3_show_wobj_2")
    move_to_named_target("sero_2_arm", "sero_2_start_blowing")
    move_to_named_target("sero_2_arm", "sero_2_end_blowing")
    move_to_named_target("sero_2_arm", "sero_2_home")
    move_to_named_target("sero_3_arm", "sero_3_place_wobj")
    move_to_named_target("sero_3_arm", "sero_3_home")

# Optional manual tests (commented out)
# Move to pose, move to position, and move to joint positions are not called in this example.
# the Reason is that the Moveit inverse kinematics (IK) solver is seemingly not able to find a 
# solution for the many of the given poses. Errors in inverse path planning are hard to reproduce
# at this current stage. You can uncomment the lines below to test them.
#     # Example of moving to a specific pose
#     pose = create_pose("test_pose", 0.6, 0.0, 0.9, 0, 0, 0)
#     move_to_pose("sero_1_arm", pose)
#     # Example of moving to a specific position
#     move_to_position("sero_1_arm", 0.7, 0.0, 1.2)
#     # Example of moving to specific joint positions
#     joint_values_deg = [0, 45, 90, 0, 0, 0]  # Example joint angles in degrees
#     move_to_joint_positions_deg("sero_1_arm", joint_values_deg)

    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Finished all movements.")
    sys.exit(0)