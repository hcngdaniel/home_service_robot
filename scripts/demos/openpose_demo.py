#!/usr/bin/env python3
import sys
sys.path.append("/home/pcms/catkin_ws/src/home_service_robot/scripts")
import rospy
import core
import cv2

rospy.init_node("openpose_demo")

cam = core.Astra("ucam")
openpose = core.openvino_models.HumanPoseEstimation()
rate = rospy.Rate(30)

print("node started")

while not rospy.is_shutdown():
    frame = cam.read_rgb()

    detections = openpose.forward(frame)

    openpose.draw_poses(frame, detections, 0.1)

    print(detections.shape)

    cv2.imshow("frame", frame)
    key = cv2.waitKey(1)
    if key == ord(' '):
        cv2.imwrite("/home/pcms/Desktop/openpose.png", frame)

    rate.sleep()
