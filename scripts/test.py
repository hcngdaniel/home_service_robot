#!/usr/bin/env python3
import core
import rospy

rospy.init_node("test")

manipulator = core.Manipulator()


print(manipulator.set_joint(0, 0.4, 0.15, -0.25, 1))
