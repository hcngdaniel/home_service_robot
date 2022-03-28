#!/usr/bin/env python3
import rospy
import core
import cv2
import numpy as np
import time

rospy.init_node('asdf')
rospy.loginfo('asdf node started')
hccd_solution = core.neural_networks.HCCD()
yolo_solution = core.neural_networks.Yolov5()
fasterrcnn_solution = core.neural_networks.FasterRCNN()
net_solutions = core.neural_networks.Solution()

colors = {'red': (0, 0, 255), 'yellow': (0, 255, 255), 'green': (0, 255, 0)}

cap = cv2.VideoCapture(0)
rate = rospy.Rate(20)
while cap.isOpened():
    success, frame = cap.read()
    if not success:
        continue
    results = net_solutions.process(frame, [hccd_solution, yolo_solution], rate=rate)
    for result in results['hccd'].boxes:
        cv2.rectangle(frame, (result.left, result.top), (result.right, result.bottom), colors[result.class_name], 2)
        cv2.putText(frame, result.class_name, (result.left, result.top), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    colors[result.class_name], 2)
    for result in results['yolov5'].boxes:
        if result.conf[0] < 0.5:
            continue
        cv2.rectangle(frame, (result.left, result.top), (result.right, result.bottom), (0, 0, 0), 2)
        cv2.putText(frame, result.class_name, (result.left, result.top), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 0, 0), 2)
    cv2.imshow('frame', frame)
    key = cv2.waitKey(16)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
