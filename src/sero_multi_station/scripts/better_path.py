#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from math import radians
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState



def move_to_absolute_pose(group, pose):
    group.set_start_state_to_current_state()
    group.set_max_velocity_scaling_factor(0.1)
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_num_planning_attempts(10)            # Erhöht Anzahl der Planungsversuche
    group.set_planning_time(10.0)                  # Erhöht Planungszeit
   #group.set_goal_position_tolerance(0.5)        # Erhöht Positions-Toleranz
    #group.set_goal_orientation_tolerance(0.5)      # Erhöht Orientierungs-Toleranz

    group.set_pose_target(pose)
    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    # Prüfen, was in RPY rauskommt
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rpy = euler_from_quaternion(q)
    rospy.loginfo(f"🎯 Zielorientierung (deg): Roll={rpy[0]*180/3.14:.1f}, Pitch={rpy[1]*180/3.14:.1f}, Yaw={rpy[2]*180/3.14:.1f}")

    if success:
        rospy.loginfo(f"✅ Pose {pose} reached.")
    else:
        rospy.logwarn(f"❌ Pose {pose} not reached.")
    return success

def round_quaternion(q, decimals=3):
    return [round(v, decimals) for v in q]

def create_pose(x, y, z, roll_deg=0, pitch_deg=0, yaw_deg=0):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    q = quaternion_from_euler(radians(roll_deg), radians(pitch_deg), radians(yaw_deg))
    print(f"📐 RPY ({roll_deg}, {pitch_deg}, {yaw_deg}) → Quaternion {q}")
    qx, qy, qz, qw = round_quaternion(q, 3)

    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    print(f"📐 RPY ({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}) → Quaternion nach runden {q}")
    return pose

def attach_object_to_tcp(group, model_name="workobject"):
    """
    Fakes a grip by teleporting a Gazebo model to the robot's TCP pose.
    """
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    tcp_pose = group.get_current_pose().pose

    state_msg = ModelState()
    state_msg.model_name = model_name
    state_msg.pose = tcp_pose
    state_msg.reference_frame = "world"  # Wichtig

    try:
        result = set_state(state_msg)
        if result.success:
            rospy.loginfo(f"🧲 '{model_name}' attached to TCP.")
        else:
            rospy.logwarn(f"⚠️ Failed to attach '{model_name}': {result.status_message}")
    except rospy.ServiceException as e:
        rospy.logwarn(f"❌ Service call failed: {e}")


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("better_path_control", anonymous=True)

    sero_1 = moveit_commander.MoveGroupCommander("sero_1_arm")
    sero_2 = moveit_commander.MoveGroupCommander("sero_2_arm")
    sero_3 = moveit_commander.MoveGroupCommander("sero_3_arm")


    pose1_s3 = create_pose(x=0.0, y=1.1, z=0.7, roll_deg=0, pitch_deg=0, yaw_deg=0)
    move_to_absolute_pose(sero_3, pose1_s3)
    attach_object_to_tcp(sero_3, "workobject")

    pose2_s3 = create_pose(x=0.45, y=0.0, z=1.2, roll_deg=0, pitch_deg=0, yaw_deg=0)
    move_to_absolute_pose(sero_3, pose2_s3)

    pose3_s3 = create_pose(x=0.0, y=-0.45, z=1.2, roll_deg=0, pitch_deg=0, yaw_deg=0)
    move_to_absolute_pose(sero_3, pose3_s3)

    pose4_s3 = create_pose(x=-1, y=0.0, z=0.7, roll_deg=0, pitch_deg=0, yaw_deg=0)
    move_to_absolute_pose(sero_3, pose4_s3)

    moveit_commander.roscpp_shutdown()
