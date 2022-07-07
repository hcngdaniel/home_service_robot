#!/usr/bin/env python3
import rospy
import core


rospy.init_node("manipulator_demo")
manipulator = core.Manipulator()
pose_1 = (0.134, 0.0, 0.150)
pose_2 = (0.134, 0.0, 0.250)

while not rospy.is_shutdown():
    manipulator.move_to(*pose_1, 1.5, slices=1)
    manipulator.move_to(*pose_2, 1.5, slices=1)
    print(manipulator.open_gripper(1.5))
    print(manipulator.close_gripper(1.5))
