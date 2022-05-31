#!/usr/bin/env python3
import core
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image


rospy.init_node("hccd_demo")
rospy.loginfo("hccd_demo node started")

astra = core.Astra()
solution = core.neural_networks.Solution()
hccd = core.neural_networks.HCCD()
pub = rospy.Publisher("/hccd_demo", Image, queue_size=10)

GREEN = (0, 255, 0)
YELLOW = (0, 255, 255)
RED = (0, 0, 255)

while not rospy.is_shutdown():
    rgb = astra.read_rgb()
    results = solution.process(rgb, [hccd])
    if results:
        boxes = results[hccd.name]
        for box in boxes:
            if box.class_name == "green":
                cv2.rectangle()
