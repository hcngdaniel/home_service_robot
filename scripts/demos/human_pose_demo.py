#!/usr/bin/env python3
import sys
import rospy
import core
import cv2


rospy.init_node("human_pose_demo")
solution = core.neural_networks.Solution()
human_pose = core.neural_networks.HumanPose()

cap = cv2.VideoCapture(0)
while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break
    results = solution.process(frame, [human_pose])
    for point in results["human_pose"].points:
        cv2.circle(frame, (int(point.x), int(point.y)), 3, (0, 255, 0), -1)
    cv2.imshow("frame", frame)
    key = cv2.waitKey(16)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()
