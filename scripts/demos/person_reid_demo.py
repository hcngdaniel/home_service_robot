#!/usr/bin/env python3
import sys
sys.path.append("/home/pcms/catkin_ws/src/home_service_robot/scripts")
import core
import rospy


rospy.init_node("person_reid_demo")

cam = core.Astra("camera")
reid = core.openvino_models.
