#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import imgui
from imgui.integrations.glfw import GlfwRenderer
import glfw
import rospy
import moveit_commander
from geometry_msgs.msg import Pose


from PIL import Image
import numpy as np
import OpenGL.GL as gl

from geometry_msgs.msg import Pose

def move_to_absolute_position(group, x, y, z):
    """
    Bewegt den TCP an eine absolute Position (x, y, z) mit neutraler Orientierung.
    Verwendet Approximate IK via set_joint_value_target.
    """
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0  # neutrale Orientierung

    try:
        group.set_start_state_to_current_state()
        group.set_max_velocity_scaling_factor(0.1)
        group.set_max_acceleration_scaling_factor(0.1)

        group.set_joint_value_target(pose, True)
        success = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        if success:
            rospy.loginfo("✅ Absolute Position erreicht.")
        else:
            rospy.logwarn("❌ Bewegung zur Position fehlgeschlagen.")
    except Exception as e:
        rospy.logwarn(f"Fehler in move_to_absolute_position: {e}")


def move_relative(group, dx, dy, dz):
    """
    Bewegt den Roboter relativ zur aktuellen TCP-Position um dx, dy, dz (in m).
    Verwendet Approximate IK via set_joint_value_target(pose, True).
    """
    try:
        base_pose = group.get_current_pose().pose  # ohne TCP-Link
        target = Pose()
        target.position.x = base_pose.position.x + dx
        target.position.y = base_pose.position.y + dy
        target.position.z = base_pose.position.z + dz
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


def load_texture_from_png(path):
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

# ImGui Init
glfw.init()
window = glfw.create_window(1280, 720, "SERO HMI", None, None)
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

        # Auswahl der Planning Group
        imgui.begin_group()
        changed, current_index = imgui.combo("Planning Group", current_index, planning_groups)
        if changed:
            group = moveit_commander.MoveGroupCommander(planning_groups[current_index])
        imgui.end_group()

        imgui.same_line()

        imgui.begin_group()
        imgui.text(" ")
        imgui.text("Roboterbild:")
        current_group_name = planning_groups[current_index]
        if current_group_name in textures:
            tex_id, w, h = textures[current_group_name]
            imgui.image(tex_id, 200, 200)  # z.B. 200x200 Pixel
        else:
            imgui.text("Kein Bild verfügbar")
        imgui.end_group()

        if changed:
            group = moveit_commander.MoveGroupCommander(planning_groups[current_index])

        # Aktuelle TCP-Position anzeigen
        try:
            current_pose = group.get_current_pose(tcp_links[current_index]).pose
            imgui.text(f"Aktuelle TCP-Position:")
            imgui.text(f"X: {current_pose.position.x:.3f}")
            imgui.text(f"Y: {current_pose.position.y:.3f}")
            imgui.text(f"Z: {current_pose.position.z:.3f}")
        except:
            imgui.text("⚠️ Keine TCP-Pose verfügbar")

        # Eingabefelder für relative Zielverschiebung
        imgui.separator()
        imgui.text("Ziel relativ zu TCP:")
        _, relative_x = imgui.input_float("Delta X [m]", relative_x, step=0.01)
        _, relative_y = imgui.input_float("Delta Y [m]", relative_y, step=0.01)
        _, relative_z = imgui.input_float("Delta Z [m]", relative_z, step=0.01)

        if imgui.button("📦 Move Relative to TCP"):
            move_relative(group, relative_x, relative_y, relative_z)

        imgui.separator()
        imgui.text("Zielkoordinaten (absolut):")

        if "abs_x" not in locals():
            abs_x, abs_y, abs_z = 0.0, 0.0, 0.0

        _, abs_x = imgui.input_float("X [m]", abs_x, 0.01)
        _, abs_y = imgui.input_float("Y [m]", abs_y, 0.01)
        _, abs_z = imgui.input_float("Z [m]", abs_z, 0.01)

        if imgui.button("🎯 Move to Absolute Position"):
            move_to_absolute_position(group, abs_x, abs_y, abs_z)


        step_size = 0.2  # initialer Wert
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
