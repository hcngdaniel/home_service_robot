#!/usr/bin/env python3
import rospy
from core import *
import cv2
import numpy as np


rospy.init_node("asdf")
rospy.loginfo("Node started")
astra = Astra()
while not rospy.is_shutdown():
    cv2.imshow("frame", astra.read_depth() / np.max(astra.read_depth()))
    print(astra.get_euclidean_distance(350, 240, astra.read_depth()))
    cv2.waitKey(16)

