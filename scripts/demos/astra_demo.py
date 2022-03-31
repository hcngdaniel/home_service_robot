#!/usr/bin/env python3
import rospy
import core
import cv2


rospy.init_node("test2")
astra = core.Astra()

while not rospy.is_shutdown():
    rgb_img = astra.read_rgb()
    depth_img = astra.read_depth()
    cv2.imshow("rgb", rgb_img)
    cv2.imshow("depth", depth_img)
    if cv2.waitKey(16) in [27, ord('q')]:
        break
    print(astra.get_euclidean_distance(320, 240, depth_img))
