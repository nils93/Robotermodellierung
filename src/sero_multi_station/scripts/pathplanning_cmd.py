#!/usr/bin/env python3

## @file pathplanning_cmd.py
#  @package sero_multi_station
#  @brief Moves a robot arm to a 3D position with neutral orientation using MoveIt.
#
#  This script is intended for basic position-only control (ignoring orientation).
#  It sets a pose goal with w=1.0 and disables orientation constraints.
#
#  @requires rospy
#  @requires moveit_commander
#  @requires geometry_msgs.msg

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def move_to_position(group_name, x, y, z):
    ##
    # @brief Moves the specified MoveIt group to a 3D target position (x, y, z).
    #
    # @param group_name Name of the MoveIt planning group (e.g., `"sero_3_arm"`).
    # @param x X-coordinate in meters
    # @param y Y-coordinate in meters
    # @param z Z-coordinate in meters
    #
    # @details
    # - A dummy quaternion (w = 1) is used for orientation
    # - Orientation tolerance is set to π (any orientation allowed)
    # - Position tolerance is set to 1 cm
    # - Uses `set_pose_target()` and `go(wait=True)` to execute motion
    #
    # @returns Logs success/failure to ROS log output
    ##
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_position_only', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(group_name)

    # Set position-only goal (with neutral orientation)
    pose_target = Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    # Dummy orientation — we don't care, so set w=1 and allow full tolerance
    pose_target.orientation.w = 1.0

    group.set_pose_target(pose_target)

    # Allow orientation tolerance to accept any orientation
    group.set_goal_orientation_tolerance(3.14)  # radians (~180 deg)
    group.set_goal_position_tolerance(0.01)     # 1cm tolerance

    # Plan and execute
    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    if success:
        rospy.loginfo(f"Successfully moved {group_name} to position ({x}, {y}, {z})")
    else:
        rospy.logerr(f"Failed to move {group_name} to position ({x}, {y}, {z})")

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("Usage: move_arm_position_only.py <group_name> <x> <y> <z>")
        print("Example: move_arm_position_only.py ser_1_arm 0.4 0.0 0.5")
        sys.exit(1)

    group_name = sys.argv[1]
    x = float(sys.argv[2])
    y = float(sys.argv[3])
    z = float(sys.argv[4])

    move_to_position(group_name, x, y, z)
