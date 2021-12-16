#!/usr/bin/env python3
import rospy
from core import *


rospy.init_node("asdf")
rospy.loginfo("Node started")
manipulator = Manipulator()
rospy.loginfo(manipulator.move_to(0.287, 0, 0.194, 1.5))
rospy.loginfo(manipulator.move_to(0.134, 0, 0.240, 1.5))

