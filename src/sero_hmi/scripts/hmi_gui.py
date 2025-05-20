#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file hmi_gui.py
@package sero_hmi
@brief ImGui-based Human-Machine Interface (HMI) for controlling multiple SERO robots using MoveIt.

This script provides a graphical interface to:
• select a planning group for one of three SERO arms
• move the robot to a predefined home pose
• control the robot in Cartesian space (relative or absolute)
• rotate the TCP via RPY inputs
• visualize the current TCP position and select commands via ImGui buttons

Dependencies:
- rospy
- moveit_commander
- geometry_msgs.msg
- OpenGL / imgui (via pyimgui and GLFW)
"""

import os
import sys
import imgui
from imgui.integrations.glfw import GlfwRenderer
import glfw
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import (
    quaternion_matrix, euler_from_quaternion, quaternion_from_euler
)
from math import radians
from PIL import Image
import numpy as np
import OpenGL.GL as gl

def move_to_home(group_name):
    """
    @brief Moves the selected robot to its predefined home pose.

    @param group_name Name of the MoveIt planning group (e.g. "sero_1_arm").
    """
    move_group = moveit_commander.MoveGroupCommander(group_name)
    home_name = group_name.replace("_arm", "_home")
    rospy.loginfo(f"🏠 Bewege {group_name} zur Pose '{home_name}'")
    move_group.set_named_target(home_name)

    success = move_group.go(wait=True)
    move_group.stop()

    if success:
        rospy.loginfo("✅ Roboter ist in der Home-Position.")
    else:
        rospy.logwarn("❌ Bewegung zur Home-Pose fehlgeschlagen.")


def move_relative_rpy(group, droll_deg, dpitch_deg, dyaw_deg):
    """
    @brief Rotates the robot TCP relative to its current orientation.

    @param group MoveGroupCommander instance.
    @param droll_deg Roll offset in degrees.
    @param dpitch_deg Pitch offset in degrees.
    @param dyaw_deg Yaw offset in degrees.
    """

    try:
        base_pose = group.get_current_pose().pose

        # aktuelle Orientierung in Euler umwandeln
        q = base_pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # relative Änderung anwenden
        roll += radians(droll_deg)
        pitch += radians(dpitch_deg)
        yaw += radians(dyaw_deg)

        # zurück in Quaternion
        q_new = quaternion_from_euler(roll, pitch, yaw)

        target = Pose()
        target.position = base_pose.position
        target.orientation.x = q_new[0]
        target.orientation.y = q_new[1]
        target.orientation.z = q_new[2]
        target.orientation.w = q_new[3]

        group.set_start_state_to_current_state()
        group.set_max_velocity_scaling_factor(0.1)
        group.set_max_acceleration_scaling_factor(0.1)
        group.set_pose_target(target)

        success = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        if success:
            rospy.loginfo("✅ Relative RPY-Bewegung erfolgreich.")
        else:
            rospy.logwarn("❌ Bewegung mit RPY fehlgeschlagen.")
    except Exception as e:
        rospy.logwarn(f"Fehler in move_relative_rpy: {e}")


def move_relative(group, dx, dy, dz):
    """
    @brief Moves the robot TCP relatively in Cartesian space.

    @param group MoveGroupCommander instance.
    @param dx Relative X offset in meters.
    @param dy Relative Y offset in meters.
    @param dz Relative Z offset in meters.
    """

    try:
        base_pose = group.get_current_pose().pose  # ohne TCP-Link
        target = Pose()
        # Local → World Transformation
        rot = [base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w]
        transformed = tf.quaternion_matrix(rot).dot([dx, dy, dz, 0.0])  # 4D homogen

        target.position.x = base_pose.position.x + transformed[0]
        target.position.y = base_pose.position.y + transformed[1]
        target.position.z = base_pose.position.z + transformed[2]
        target.orientation = base_pose.orientation  # Orientierung beibehalten

        group.set_start_state_to_current_state()
        group.set_max_velocity_scaling_factor(0.1)
        group.set_max_acceleration_scaling_factor(0.1)

        group.set_joint_value_target(target, True)
        success = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        if success:
            rospy.loginfo("✅ Relative Bewegung erfolgreich.")
        else:
            rospy.logwarn("❌ Bewegung fehlgeschlagen.")
    except Exception as e:
        rospy.logwarn(f"Fehler in move_relative: {e}")

def move_to_absolute_pose(group, pose):
    """
    @brief Moves the robot TCP to a given absolute target pose.

    @param group MoveGroupCommander instance.
    @param pose Target geometry_msgs/Pose object in world coordinates.
    """

    try:
        group.set_start_state_to_current_state()
        group.set_max_velocity_scaling_factor(0.1)
        group.set_max_acceleration_scaling_factor(0.1)

        group.set_pose_target(pose)
        success = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        if success:
            rospy.loginfo("✅ Absolute Bewegung erfolgreich.")
        else:
            rospy.logwarn("❌ Absolute Pose nicht erreicht.")
    except Exception as e:
        rospy.logwarn(f"Fehler in move_to_absolute_pose: {e}")



def load_texture_from_png(path):
    """
    @brief Loads a PNG image as an OpenGL texture for ImGui.

    @param path Absolute path to the PNG image file.
    @return (texture_id, width, height) tuple.
    """
    
    image = Image.open(path).convert("RGBA")
    img_data = np.array(image, dtype=np.uint8)
    texture_id = gl.glGenTextures(1)
    gl.glBindTexture(gl.GL_TEXTURE_2D, texture_id)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
    gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGBA, image.width, image.height,
                    0, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, img_data)
    return texture_id, image.width, image.height


rospy.init_node("hmi_gui_py")
moveit_commander.roscpp_initialize([])

# Planning-Groups + zugehörige TCP-Links
planning_groups = ["sero_1_arm", "sero_2_arm", "sero_3_arm"]
tcp_links = ["sero_1_tcp", "sero_2_tcp", "sero_3_tcp"]
current_index = 0
group = moveit_commander.MoveGroupCommander(planning_groups[current_index])

# GUI Inputfelder für relative Bewegung
relative_x = 0.0
relative_y = 0.0
relative_z = 0.0

# GUI Inputfeld für Step-Size
step_size = 0.2  # initialer Wert

# ImGui Init
glfw.init()
window = glfw.create_window(1400, 800, "SERO HMI", None, None)
glfw.make_context_current(window)
imgui.create_context()
impl = GlfwRenderer(window)

# Bildpfade vorbereiten
pkg_dir = os.path.dirname(os.path.abspath(__file__))
image_paths = {
    "sero_1_arm": os.path.join(pkg_dir, "../resources/sero_1_arm.png"),
    "sero_2_arm": os.path.join(pkg_dir, "../resources/sero_2_arm.png"),
    "sero_3_arm": os.path.join(pkg_dir, "../resources/sero_3_arm.png")
}
# Texturen vorbereiten
textures = {}
for name, path in image_paths.items():
    tex_id, width, height = load_texture_from_png(path)
    textures[name] = (tex_id, width, height)


try:
    while not rospy.is_shutdown() and not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()

        imgui.new_frame()
        imgui.begin("SERO Robotersteuerung")

        # Linke Spalte
        imgui.begin_group()

        # Planning Group Auswahl
        changed, current_index = imgui.combo("Planning Group", current_index, planning_groups)
        if changed:
            group = moveit_commander.MoveGroupCommander(planning_groups[current_index])

        # Home-Button
        if imgui.button("Home-Position"):
            group_name = planning_groups[current_index]
            move_to_home(group_name)

        imgui.end_group()

        # Rechte Spalte
        imgui.same_line()

        imgui.begin_group()

        # Roboterbild
        current_group_name = planning_groups[current_index]
        if current_group_name in textures:
            tex_id, w, h = textures[current_group_name]
            imgui.image(tex_id, 200, 200)
        else:
            imgui.text("Kein Bild verfügbar")

        # Aktuelle TCP-Position
        try:
            current_pose = group.get_current_pose(tcp_links[current_index]).pose
            imgui.text("TCP Position:")
            imgui.text(f"X: {current_pose.position.x:.3f}")
            imgui.text(f"Y: {current_pose.position.y:.3f}")
            imgui.text(f"Z: {current_pose.position.z:.3f}")
        except:
            imgui.text("⚠️ Keine TCP-Pose verfügbar")

        imgui.end_group()

        # Eingabefelder für relative Zielverschiebung
        imgui.separator()

        imgui.text("Ziel relativ zu TCP:")
        _, relative_x = imgui.input_float("Delta X [m]", relative_x, step=0.01)
        _, relative_y = imgui.input_float("Delta Y [m]", relative_y, step=0.01)
        _, relative_z = imgui.input_float("Delta Z [m]", relative_z, step=0.01)

        if imgui.button("Move Relative to TCP"):
            move_relative(group, relative_x, relative_y, relative_z)

        # Button für absolute Bewegung
        # Eingabe für absolute Ziel-Pose
        imgui.separator()
        imgui.text("Zielpose absolut (Weltkoordinaten):")

        # Initialisierung der Pose-Werte
        if "abs_pose" not in locals():
            abs_pose = Pose()
            abs_pose.position.x = 0.5
            abs_pose.position.y = 0.0
            abs_pose.position.z = 0.5
            abs_pose.orientation = current_pose.orientation  # default: aktuelle TCP-Ori

        _, abs_pose.position.x = imgui.input_float("X [m]", abs_pose.position.x)
        _, abs_pose.position.y = imgui.input_float("Y [m]", abs_pose.position.y)
        _, abs_pose.position.z = imgui.input_float("Z [m]", abs_pose.position.z)

        # Button für Bewegung
        if imgui.button("Move to absolute Pose"):
            move_to_absolute_pose(group, abs_pose)

        # Steuerung per Buttons
        imgui.separator()
        imgui.text("Steuerung per Buttons:")

        # Eingabefeld für Step-Size
        
        _, step_size = imgui.slider_float("Step Size [m]", step_size, 0.01, 0.3, "%.3f")
        move = [0.0, 0.0, 0.0]

        if imgui.button("+X"): move[0] -= step_size
        imgui.same_line()
        if imgui.button("-X"): move[0] += step_size
        imgui.same_line()
        if imgui.button("+Y"): move[1] += step_size
        imgui.same_line()
        if imgui.button("-Y"): move[1] -= step_size

        if imgui.button("+Z"): move[2] += step_size
        imgui.same_line()
        if imgui.button("-Z"): move[2] -= step_size

        if any(move):
            try:
                base_pose = group.get_current_pose(tcp_links[current_index]).pose
                target = Pose()
                target.position.x = base_pose.position.x + move[0]
                target.position.y = base_pose.position.y + move[1]
                target.position.z = base_pose.position.z + move[2]
                target.orientation = base_pose.orientation

                group.set_joint_value_target(target, True)  # Approximate IK erlaubt mehr Freiheit
                success = group.plan()
                rospy.loginfo(f"Planung erfolgreich? {success[0]}")
                group.go(wait=True)
                group.stop()
                group.clear_pose_targets()
            except Exception as e:
                rospy.logwarn(f"Fehler bei Button-Bewegung: {e}")

        # Steuerung per RPY
        imgui.separator()
        imgui.text("Rotation (relativ zum TCP):")

        if "rot_step" not in locals():
            rot_step = 5.0  # Grad pro Klick

        _, rot_step = imgui.input_float("Rotationsschritt [°]", rot_step, 1.0)
        rot_step = max(1.0, min(45.0, rot_step))  # begrenzen

        rpy_move = [0.0, 0.0, 0.0]

        if imgui.button("+Roll"): rpy_move[0] += rot_step
        imgui.same_line()
        if imgui.button("-Roll"): rpy_move[0] -= rot_step
        imgui.same_line()
        if imgui.button("+Pitch"): rpy_move[1] += rot_step
        imgui.same_line()
        if imgui.button("-Pitch"): rpy_move[1] -= rot_step
        imgui.same_line()
        if imgui.button("+Yaw"): rpy_move[2] += rot_step
        imgui.same_line()
        if imgui.button("-Yaw"): rpy_move[2] -= rot_step

        if any(rpy_move):
            move_relative_rpy(group, *rpy_move)

        imgui.end()
        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

except KeyboardInterrupt:
    rospy.loginfo("🛑 GUI beendet durch Benutzer (Ctrl+C)")

finally:
    impl.shutdown()
    glfw.terminate()
    moveit_commander.roscpp_shutdown()
