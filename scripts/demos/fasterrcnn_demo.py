#!/usr/bin/env python3
import core
import rospy
import cv2


rospy.init_node("fasterrcnn_demo")

cap = cv2.VideoCapture(0)
solution = core.neural_networks.Solution()
fasterrcnn = core.neural_networks.FasterRCNN()
print('hihihihi')

while cap.isOpened():
    # cv2.imshow("rgb", rgb)
    # cv2.waitKey(1)
    ret, rgb = cap.read()
    results = solution.process(rgb, [fasterrcnn])
    for result in results['fasterrcnn'].boxes:
        cv2.rectangle(rgb, (result.left, result.top), (result.right, result.bottom), (0, 0, 0), 2)
    cv2.imshow("rgb", rgb)
    cv2.waitKey(16)
    # rospy.Rate(20).sleep()
