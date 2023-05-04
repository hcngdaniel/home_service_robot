#!/usr/bin/env python3
import rospy
import core
import time


rospy.init_node('navigation_demo')
rospy.loginfo("navigation_demo node started!")
time.sleep(1)

navigation = core.Navigation()
time.sleep(1)

init_pose = (1.01, 0.165, 0)
target_pose = (-0.162, 3.48, 0)

while not rospy.is_shutdown():
    navigation.move_to(*init_pose)
    print('hi')
    time.sleep(0.5)
    navigation.move_to(*target_pose)
    print('bye')
    time.sleep(0.5)
