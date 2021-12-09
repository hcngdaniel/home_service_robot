#!/usr/bin/env python3
import rospy
from core import *
import cv2


rospy.init_node("asdfr")
astra = Astra()
while not rospy.is_shutdown():
    if astra.read_rgb() is not None:
        cv2.imshow("frame", astra.read_rgb())
        cv2.waitKey(16)
