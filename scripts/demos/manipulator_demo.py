#!/usr/bin/env python3
import rospy
import core


rospy.init_node("manipulator_demo")
manipulator = core.Manipulator()
pose_1 = (0.290, 0.0, 0.186)
pose_2 = (0.285, 0.0, -0.026)

while not rospy.is_shutdown():
    manipulator.move_to(*pose_1, 1.5, slices=1)
    manipulator.move_to(*pose_2, 1.5, slices=1)
    print(manipulator.open_gripper(1.5))
    print(manipulator.close_gripper(1.5))
