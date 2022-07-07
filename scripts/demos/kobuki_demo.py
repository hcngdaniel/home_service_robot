#!/usr/bin/env python3
import rospy
import time
import core


rospy.init_node("turtlebot_demo")
kobuki = core.Kobuki()
rospy.Rate(1).sleep()

start_time = time.time()
while not rospy.is_shutdown():
    kobuki.move(0.1, 0)
    rospy.Rate(100).sleep()
    if time.time() - start_time > 1.5:
        break
kobuki.move(0, 0)
