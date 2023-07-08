#!/usr/bin/env python3
import sys
sys.path.append("/home/pcms/catkin_ws/src/home_service_robot/scripts")
import rospy
import core
import time


rospy.init_node('navigation_demo')
rospy.loginfo("navigation_demo node started!")
time.sleep(1)

navigation = core.Navigation()
time.sleep(1)

init_pose = (0.925, -2.98, 0)
target_pose = (2.36, -1, 1.57)

while not rospy.is_shutdown():
    navigation.move_to(*init_pose)
    print('hi')
    time.sleep(0.5)
    navigation.move_to(*target_pose)
    print('bye')
    time.sleep(0.5)
