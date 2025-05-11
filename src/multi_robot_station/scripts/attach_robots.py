#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
import time

def wait_for_service():
    rospy.loginfo("📡 Warte auf /link_attacher_node/attach ...")
    rospy.wait_for_service('/link_attacher_node/attach')
    rospy.loginfo("✅ Service verfügbar.")

def attach(model1, link1, model2, link2):
    rospy.loginfo(f"🔗 Verbinde {model2}:{link2} mit {model1}:{link1}")
    try:
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2
        attach_srv.call(req)
        rospy.loginfo(f"✅ Verbunden: {model2} an {model1}")
    except rospy.ServiceException as e:
        rospy.logerr(f"❌ Service call fehlgeschlagen: {e}")

if __name__ == '__main__':
    rospy.init_node('multi_robot_attach')
    wait_for_service()

    time.sleep(5.0)  # Gib Gazebo etwas Zeit zum Spawnen

    # Roboter an Sockel koppeln
    attach("sero_1_sockel", "link", "sero_1", "base")
    attach("sero_2_sockel", "link", "sero_2", "base")
    attach("sero_3_sockel", "link", "sero_3", "base")
