#!/usr/bin/env python3
import core
import rospy
import cv2


rospy.init_node("fasterrcnn_demo")

cap = cv2.VideoCapture(0)
solution = core.neural_networks.Solution()
fasterrcnn = core.neural_networks.FasterRCNN()
print('node started!')

while cap.isOpened():
    ret, rgb = cap.read()
    results = solution.process(rgb, [fasterrcnn])
    for result in results['fasterrcnn'].boxes:
        if result.conf[0] > 0.8:
            cv2.rectangle(rgb, (result.left, result.top), (result.right, result.bottom), (0, 0, 0), 2)
            cv2.putText(rgb, result.class_name, (result.left, result.top), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1)
    cv2.imshow("rgb", rgb)
    cv2.waitKey(16)
    # rospy.Rate(20).sleep()
