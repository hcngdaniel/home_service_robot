#!/usr/bin/env python3
import rospy
import core
import cv2


rospy.init_node("test2")
astra_top = core.Astra("CAM_UP")
astra_down = core.Astra("CAM_DOWN")

while not rospy.is_shutdown():
    rgb_img = astra_top.read_rgb()
    depth_img = astra_top.read_depth()
    cv2.imshow("top_rgb", rgb_img)
    cv2.imshow("top_depth", depth_img)
    rgb_img = astra_down.read_rgb()
    depth_img = astra_down.read_depth()
    cv2.imshow("down_rgb", rgb_img)
    cv2.imshow("down_depth", depth_img)
    if cv2.waitKey(16) in [27, ord('q')]:
        break
