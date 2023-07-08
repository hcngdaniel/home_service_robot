#!/usr/bin/env python3
import sys
sys.path.append("/home/pcms/catkin_ws/src/home_service_robot/scripts")
import core
import rospy
import cv2


rospy.init_node("yolo_demo")

cam = core.Astra("ucam")
yolo = core.openvino_models.Yolov8("yolov8n")


def filter_detections(dets, cls):
    res = []
    for det in dets:
        if yolo.classes[int(det[5])] == cls:
            res.append(det)
    return res


while not rospy.is_shutdown():
    frame = cam.read_rgb()
    detections = yolo.forward(frame)[0]['det']
    detections = filter_detections(detections, "chair")
    yolo.draw_bounding_box({'det': detections}, frame)
    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    rospy.Rate(30).sleep()

