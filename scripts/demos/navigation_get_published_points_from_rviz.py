#!/usr/bin/env python3
import rospy
import core
import time


rospy.init_node('get_published_points')
time.sleep(1)

navigation = core.Navigation()
time.sleep(1)

while not rospy.is_shutdown():
    point = navigation.get_clicked_point_in_rviz().point
    print(point.x, point.y, point.z)
