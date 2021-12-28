#!/usr/bin/env python3
import rospy
from core import *


rospy.init_node("asdf")
rospy.loginfo("Node started")
manipulator = Manipulator()
while not rospy.is_shutdown():
    manipulator.move_to(0.134, 0.0, 0.240, 1)
    manipulator.move_to(0.134, 0.0, 0.300, 1)

