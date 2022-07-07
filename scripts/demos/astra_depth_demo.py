#!/usr/bin/env python3
import rospy
import core
import time
import cv2
import numpy as np


rospy.init_node("astra_depth_demo")
rospy.loginfo("astra_depth_demo node started!")

astra = core.Astra()

time.time()

while not rospy.is_shutdown():
    bgr = astra.read_rgb()
    depth = astra.read_depth()
    print("depth: ", depth[240][320])
    x, y, z = astra.get_real_xyz(320, 240, depth)
    print("coordinates: ", x, y, z)
    cv2.imshow("depth", cv2.flip(depth / np.max(depth), 0))
    cv2.imshow("bgr", cv2.flip(bgr, 0))
    cv2.waitKey(1)
    rospy.Rate(20).sleep()
