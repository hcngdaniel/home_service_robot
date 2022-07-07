#!/usr/bin/env python3
import rospy
import core
import time


rospy.init_node('navigation_demo')
rospy.loginfo("navigation_demo node started!")
time.sleep(1)

navigation = core.Navigation()
time.sleep(1)

init_pose = (0.849346756935, -0.320832133293, navigation.zw_to_a(-0.635722699939, 0.771917514234))
target_pose = (0.527837617490, -2.63831138611, navigation.zw_to_a(-0.702896831634, 0.711291813589))

while not rospy.is_shutdown():
    navigation.move_to(*init_pose)
    time.sleep(0.5)
    navigation.move_to(*target_pose)
    time.sleep(0.5)
